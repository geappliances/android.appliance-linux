// SPDX-License-Identifier: GPL-2.0
//
// Copyright 2020 BayLibre SAS

#include <linux/cdev.h>
#include <linux/dma-buf.h>
#include <linux/iommu.h>
#include <linux/iova.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/remoteproc.h>
#include <linux/rpmsg.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include "rpmsg_internal.h"

#include <uapi/linux/apu_rpmsg.h>

#include "apu_rpmsg.h"

/* Maximum of APU devices supported */
#define APU_DEV_MAX 2

#define dev_to_apu(dev) container_of(dev, struct rpmsg_apu, dev)
#define cdev_to_apu(i_cdev) container_of(i_cdev, struct rpmsg_apu, cdev)

struct rpmsg_apu {
	struct rpmsg_device *rpdev;
	struct cdev cdev;
	struct device dev;

	struct rproc *rproc;
	struct iommu_domain *domain;
	struct iova_domain *iovad;
	int iova_limit_pfn;
	wait_queue_head_t waitqueue;
	u8 available_response;
	spinlock_t ctx_lock;
	struct list_head requests;

	struct list_head buffers;
};

struct rpmsg_request {
	u8 ready;
	struct list_head node;
	struct list_head buffers;
	void *req;
};

struct apu_buffer {
	int fd;
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sg_table;
	u32 iova;

	struct rpmsg_apu *apu;
	struct list_head node;
	struct list_head req_node;
	struct kref refcount;
};

/*
 * Shared IOVA domain.
 * The MT8183 has two VP6 core but they are sharing the IOVA.
 * They could be used alone, or together. In order to avoid conflict,
 * create an IOVA domain that could be shared by those two core.
 * @iovad: The IOVA domain to share between the APU cores
 * @refcount: Allow to automatically release the IOVA domain once all the APU
 *            cores has been stopped
 */
struct apu_iova_domain {
	struct iova_domain iovad;
	struct kref refcount;
};

static dev_t rpmsg_major;
static DEFINE_IDA(rpmsg_minor_ida);
static DEFINE_IDA(req_ida);
static struct apu_iova_domain *apu_iovad;


static int apu_rpmsg_callback(struct rpmsg_device *rpdev, void *data, int count,
			      void *priv, u32 addr)
{
	struct rpmsg_apu *apu = dev_get_drvdata(&rpdev->dev);
	struct rpmsg_request *rpmsg_req;
	struct apu_dev_request *hdr = data;
	unsigned long flags;

	spin_lock_irqsave(&apu->ctx_lock, flags);
	list_for_each_entry(rpmsg_req, &apu->requests, node) {
		struct apu_request *tmp_hdr = rpmsg_req->req;

		if (hdr->id == tmp_hdr->id) {
			rpmsg_req->ready = 1;
			apu->available_response++;
			tmp_hdr->result = hdr->result;
			tmp_hdr->size_in = hdr->size_in;
			tmp_hdr->size_out = hdr->size_out;
			memcpy(tmp_hdr->data, hdr->data,
			       hdr->size_in+hdr->size_out);

			wake_up_interruptible(&apu->waitqueue);
			break;
		}
	}
	spin_unlock_irqrestore(&apu->ctx_lock, flags);

	return 0;
}

static struct apu_buffer *apu_device_memory_map(struct rpmsg_apu *apu,
						uint32_t fd)
{
	struct rpmsg_device *rpdev = apu->rpdev;
	struct apu_buffer *buffer;
	phys_addr_t phys;
	int total_buf_space;
	int iova_pfn;
	int ret;

	if (!fd)
		return ERR_PTR(-EINVAL);

	list_for_each_entry(buffer, &apu->buffers, node) {
		if (buffer->fd == fd) {
			kref_get(&buffer->refcount);
			return buffer;
		}
	}

	buffer = kmalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return ERR_PTR(-ENOMEM);

	kref_init(&buffer->refcount);
	buffer->fd = fd;
	buffer->apu = apu;
	INIT_LIST_HEAD(&buffer->req_node);
	INIT_LIST_HEAD(&buffer->node);

	buffer->dma_buf = dma_buf_get(buffer->fd);
	if (IS_ERR(buffer->dma_buf)) {
		dev_err(&rpdev->dev, "Failed to get dma_buf from fd: %ld\n",
			PTR_ERR(buffer->dma_buf));
		ret = PTR_ERR(buffer->dma_buf);
		goto err_free_buffer;
	}

	buffer->attachment = dma_buf_attach(buffer->dma_buf, &rpdev->dev);
	if (IS_ERR(buffer->attachment)) {
		dev_err(&rpdev->dev, "Failed to attach dma_buf\n");
		ret = PTR_ERR(buffer->attachment);
		goto err_dma_buf_put;
	}

	buffer->sg_table = dma_buf_map_attachment(buffer->attachment,
						   DMA_BIDIRECTIONAL);
	if (IS_ERR(buffer->sg_table)) {
		dev_err(&rpdev->dev, "Failed to map attachment\n");
		ret = PTR_ERR(buffer->sg_table);
		goto err_dma_buf_detach;
	}
	phys = page_to_phys(sg_page(buffer->sg_table->sgl));
	total_buf_space = sg_dma_len(buffer->sg_table->sgl);

	iova_pfn = alloc_iova_fast(apu->iovad, total_buf_space >> PAGE_SHIFT,
				   apu->iova_limit_pfn, true);
	if (!iova_pfn) {
		dev_err(&rpdev->dev, "Failed to allocate iova address\n");
		ret = -ENOMEM;
		goto err_dma_unmap_attachment;
	}

	buffer->iova = PFN_PHYS(iova_pfn);
	ret = iommu_map(apu->rproc->domain, buffer->iova, phys, total_buf_space,
			IOMMU_READ | IOMMU_WRITE | IOMMU_CACHE);
	if (ret) {
		dev_err(&rpdev->dev, "Failed to iommu map\n");
		goto err_free_iova;
	}

	list_add(&buffer->node, &apu->buffers);

	return buffer;

err_free_iova:
	free_iova(apu->iovad, iova_pfn);
err_dma_unmap_attachment:
	dma_buf_unmap_attachment(buffer->attachment,
				 buffer->sg_table,
				 DMA_BIDIRECTIONAL);
err_dma_buf_detach:
	dma_buf_detach(buffer->dma_buf, buffer->attachment);
err_dma_buf_put:
	dma_buf_put(buffer->dma_buf);
err_free_buffer:
	kfree(buffer);

	return ERR_PTR(ret);
}

static void apu_device_memory_unmap(struct kref *ref)
{
	struct apu_buffer *buffer = container_of(ref, struct apu_buffer,
						 refcount);
	struct rpmsg_apu *apu = buffer->apu;
	int total_buf_space;

	if (!buffer->fd)
		return;

	total_buf_space = sg_dma_len(buffer->sg_table->sgl);
	iommu_unmap(apu->rproc->domain, buffer->iova, total_buf_space);
	free_iova(apu->iovad, PHYS_PFN(buffer->iova));
	dma_buf_unmap_attachment(buffer->attachment,
				 buffer->sg_table,
				 DMA_BIDIRECTIONAL);
	dma_buf_detach(buffer->dma_buf, buffer->attachment);
	dma_buf_put(buffer->dma_buf);
	list_del(&buffer->node);
	kfree(buffer);
}

static int apu_iommu_mmap_ioctl(struct rpmsg_apu *apu, void __user *argp)
{
	struct apu_iommu_mmap apu_iommu_mmap;
	struct apu_buffer *buffer;
	int ret;

	if (copy_from_user(&apu_iommu_mmap, argp, sizeof(apu_iommu_mmap)))
		return -EFAULT;

	buffer = apu_device_memory_map(apu, apu_iommu_mmap.fd);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	apu_iommu_mmap.da = buffer->iova;
	if (copy_to_user(argp, &apu_iommu_mmap, sizeof(apu_iommu_mmap)))
		ret = -EFAULT;

	return 0;
}

static int apu_iommu_munmap_ioctl(struct rpmsg_apu *apu, void __user *argp)
{
	u32 fd;
	struct apu_buffer *buffer, *tmp;

	if (copy_from_user(&fd, argp, sizeof(fd)))
		return -EFAULT;

	list_for_each_entry_safe(buffer, tmp, &apu->buffers, node) {
		if (buffer->fd == fd) {
			kref_put(&buffer->refcount, apu_device_memory_unmap);
			return 0;
		}
	}

	return -EINVAL;
}

static int apu_send_request(struct rpmsg_apu *apu,
			    struct apu_request *req)
{
	int ret;
	struct rpmsg_device *rpdev = apu->rpdev;
	struct apu_dev_request *dev_req;
	struct apu_buffer *buffer, *tmp;
	struct rpmsg_request *rpmsg_req;
	unsigned long flags;

	int size = req->size_in + req->size_out +
		sizeof(u32) * req->count * 2 + sizeof(*dev_req);
	u32 *fd = (u32 *)(req->data + req->size_in + req->size_out);
	u32 *buffer_size = (u32 *)(fd + req->count);
	u32 *dev_req_da;
	u32 *dev_req_buffer_size;
	int i;

	dev_req = kmalloc(size, GFP_KERNEL);
	if (!dev_req)
		return -ENOMEM;

	dev_req->cmd = req->cmd;
	dev_req->size_in = req->size_in;
	dev_req->size_out = req->size_out;
	dev_req->count = req->count;
	dev_req_da = (u32 *)(dev_req->data + req->size_in + req->size_out);
	dev_req_buffer_size = (u32 *)(dev_req_da + dev_req->count);
	memcpy(dev_req->data, req->data, req->size_in);

	rpmsg_req = kzalloc(sizeof(*rpmsg_req), GFP_KERNEL);
	if (!rpmsg_req)
		return -ENOMEM;

	INIT_LIST_HEAD(&rpmsg_req->buffers);
	for (i = 0; i < req->count; i++) {
		buffer = apu_device_memory_map(apu, fd[i]);
		if (IS_ERR(buffer)) {
			ret = PTR_ERR(buffer);
			goto err_free_memory;
		}

		dev_req_da[i] = buffer->iova;
		dev_req_buffer_size[i] = buffer_size[i];
		list_add(&buffer->req_node, &rpmsg_req->buffers);
	}

	ret = ida_simple_get(&req_ida, 0, 0xffff, GFP_KERNEL);
	if (ret < 0)
		goto err_free_memory;

	dev_req->id = ret;

	req->id = dev_req->id;
	rpmsg_req->req = req;
	spin_lock_irqsave(&apu->ctx_lock, flags);
	list_add(&rpmsg_req->node, &apu->requests);
	spin_unlock_irqrestore(&apu->ctx_lock, flags);

	ret = rpmsg_send(rpdev->ept, dev_req, size);
	if (ret < 0)
		goto err;

	kfree(dev_req);

	return req->id;

err:
	list_del(&rpmsg_req->node);
	kfree(req);
	ida_simple_remove(&req_ida, dev_req->id);
err_free_memory:
	list_for_each_entry_safe(buffer, tmp, &rpmsg_req->buffers, req_node)
		kref_put(&buffer->refcount, apu_device_memory_unmap);
	kfree(rpmsg_req);
	kfree(dev_req);

	return ret;
}

unsigned int rpmsg_eptdev_poll(struct file *fp, struct poll_table_struct *wait)
{
	struct rpmsg_apu *apu = fp->private_data;
	unsigned long flags;

	poll_wait(fp, &apu->waitqueue, wait);
	spin_lock_irqsave(&apu->ctx_lock, flags);
	if (apu->available_response) {
		spin_unlock_irqrestore(&apu->ctx_lock, flags);
		return POLLIN;
	}
	spin_unlock_irqrestore(&apu->ctx_lock, flags);
	return 0;
}

static long rpmsg_eptdev_ioctl(struct file *fp, unsigned int cmd,
			       unsigned long arg)
{
	struct rpmsg_apu *apu = fp->private_data;
	struct apu_request apu_req;
	struct apu_request *apu_req_full;
	struct apu_buffer *buffer, *tmp;
	void __user *argp = (void __user *)arg;
	int len;
	int ret;
	unsigned long flags;
	struct rpmsg_request *rpmsg_req;

	ret = 0;

	switch (cmd) {
	case APU_SEND_REQ_IOCTL:
		/* Get the header */
		if (copy_from_user(&apu_req, argp,
				   sizeof(apu_req)))
			return -EFAULT;

		len = sizeof(*apu_req_full) + apu_req.size_in +
			apu_req.size_out + apu_req.count * sizeof(u32) * 2;
		apu_req_full = kzalloc(len, GFP_KERNEL);
		if (!apu_req_full)
			return -ENOMEM;

		/* Get the whole request */
		if (copy_from_user(apu_req_full, argp, len)) {
			kfree(apu_req_full);
			return -EFAULT;
		}

		ret = apu_send_request(apu, apu_req_full);

		break;
	case APU_GET_NEXT_AVAILABLE_IOCTL:
		ret = -ENOMSG;
		spin_lock_irqsave(&apu->ctx_lock, flags);
		list_for_each_entry(rpmsg_req, &apu->requests, node) {
			if (rpmsg_req->ready == 1) {
				struct apu_request *req =
					rpmsg_req->req;

				ret = 0;
				if (copy_to_user(argp, &req->id, sizeof(__u16)))
					ret = -EFAULT;
				break;
			}
		}
		spin_unlock_irqrestore(&apu->ctx_lock, flags);
		break;
	case APU_GET_RESP:
		/* Get the header */
		if (!argp)
			return -EINVAL;

		if (copy_from_user(&apu_req, argp,
				   sizeof(apu_req)))
			return -EFAULT;

		spin_lock_irqsave(&apu->ctx_lock, flags);
		list_for_each_entry(rpmsg_req, &apu->requests, node) {
			struct apu_request *req = rpmsg_req->req;

			if ((apu_req.id == req->id) &&
			    (rpmsg_req->ready == 1)) {
				int req_len = sizeof(struct apu_request) +
					req->size_in + req->size_out +
					req->count * sizeof(u32)*2;
				int apu_req_len = sizeof(struct apu_request) +
					req->size_in + req->size_out +
					req->count * sizeof(u32)*2;

				len = min(req_len, apu_req_len);
				if (copy_to_user(argp, req, len))
					ret = -EFAULT;
				apu->available_response--;
				ida_simple_remove(&req_ida, req->id);
				list_for_each_entry_safe(buffer, tmp, &rpmsg_req->buffers, req_node) {
					kref_put(&buffer->refcount, apu_device_memory_unmap);
					list_del(&buffer->req_node);
				}
				list_del(&rpmsg_req->node);
				kfree(rpmsg_req->req);
				kfree(rpmsg_req);
				break;
			}
		}
		spin_unlock_irqrestore(&apu->ctx_lock, flags);

		break;
	case APU_IOMMU_MMAP:
		ret = apu_iommu_mmap_ioctl(apu, argp);
		break;
	case APU_IOMMU_MUNMAP:
		ret = apu_iommu_munmap_ioctl(apu, argp);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int rpmsg_eptdev_open(struct inode *inode, struct file *filp)
{
	struct rpmsg_apu *apu = cdev_to_apu(inode->i_cdev);

	get_device(&apu->dev);
	filp->private_data = apu;

	return 0;
}

static int rpmsg_eptdev_release(struct inode *inode, struct file *filp)
{
	struct rpmsg_apu *apu = cdev_to_apu(inode->i_cdev);

	put_device(&apu->dev);

	return 0;
}

static const struct file_operations rpmsg_eptdev_fops = {
	.owner = THIS_MODULE,
	.open = rpmsg_eptdev_open,
	.release = rpmsg_eptdev_release,
	.unlocked_ioctl = rpmsg_eptdev_ioctl,
	.compat_ioctl = rpmsg_eptdev_ioctl,
	.poll = rpmsg_eptdev_poll,
};

static void iova_domain_release(struct kref *ref)
{
	put_iova_domain(&apu_iovad->iovad);
	kfree(apu_iovad);
	apu_iovad = NULL;
}

static struct fw_rsc_iova *apu_find_rcs_iova(struct rpmsg_apu *apu)
{
	struct rproc *rproc = apu->rproc;
	struct resource_table *table;
	struct fw_rsc_iova *rsc;
	int i;

	table = rproc->table_ptr;
	for (i = 0; i < table->num; i++) {
		int offset = table->offset[i];
		struct fw_rsc_hdr *hdr = (void *)table + offset;

		switch (hdr->type) {
		case RSC_VENDOR_IOVA:
			rsc = (void *)hdr + sizeof(*hdr);
				return rsc;
			break;
		default:
			continue;
		}
	}

	return NULL;
}

static int apu_reserve_iova(struct rpmsg_apu *apu, struct iova_domain *iovad)
{
	struct rproc *rproc = apu->rproc;
	struct resource_table *table;
	struct fw_rsc_carveout *rsc;
	int i;

	table = rproc->table_ptr;
	for (i = 0; i < table->num; i++) {
		int offset = table->offset[i];
		struct fw_rsc_hdr *hdr = (void *)table + offset;

		if (hdr->type == RSC_CARVEOUT) {
			struct iova *iova;

			rsc = (void *)hdr + sizeof(*hdr);
			iova = reserve_iova(iovad, PHYS_PFN(rsc->da),
					    PHYS_PFN(rsc->da + rsc->len));
			if (!iova) {
				dev_err(&apu->dev, "failed to reserve iova\n");
				return -ENOMEM;
			}
			dev_dbg(&apu->dev, "Reserve: %x - %x\n",
				rsc->da, rsc->da + rsc->len);
		}
	}

	return 0;
}

static int apu_init_iovad(struct rpmsg_apu *apu)
{
	struct fw_rsc_iova *rsc;

	if (!apu->rproc->table_ptr) {
		dev_err(&apu->dev,
			"No resource_table: has the firmware been loaded ?\n");
		return -ENODEV;
	}

	rsc = apu_find_rcs_iova(apu);
	if (!rsc) {
		dev_err(&apu->dev, "No iova range defined in resource_table\n");
		return -ENOMEM;
	}

	if (!apu_iovad) {
		apu_iovad = kzalloc(sizeof(*apu_iovad), GFP_KERNEL);
		if (!apu_iovad)
			return -ENOMEM;

		init_iova_domain(&apu_iovad->iovad, PAGE_SIZE,
				 PHYS_PFN(rsc->da));
		apu_reserve_iova(apu, &apu_iovad->iovad);
		kref_init(&apu_iovad->refcount);
	} else
		kref_get(&apu_iovad->refcount);

	apu->iovad = &apu_iovad->iovad;
	apu->iova_limit_pfn = PHYS_PFN(rsc->da + rsc->len) - 1;

	return 0;
}

static struct rproc *apu_get_rproc(struct rpmsg_device *rpdev)
{
	/*
	 * To work, the APU RPMsg driver need to get the rproc device.
	 * Currently, we only use virtio so we could use that to find the
	 * remoteproc parent.
	 */
	if (!rpdev->dev.parent && rpdev->dev.parent->bus) {
		dev_err(&rpdev->dev, "invalid rpmsg device\n");
		return ERR_PTR(-EINVAL);
	}

	if (strcmp(rpdev->dev.parent->bus->name, "virtio")) {
		dev_err(&rpdev->dev, "unsupported bus\n");
		return ERR_PTR(-EINVAL);
	}

	return vdev_to_rproc(dev_to_virtio(rpdev->dev.parent));
}

static void rpmsg_apu_release_device(struct device *dev)
{
	struct rpmsg_apu *apu = dev_to_apu(dev);

	ida_simple_remove(&rpmsg_minor_ida, MINOR(dev->devt));
	cdev_del(&apu->cdev);
}

static int apu_rpmsg_probe(struct rpmsg_device *rpdev)
{
	struct rpmsg_apu *apu;
	struct device *dev;
	int ret;

	apu = devm_kzalloc(&rpdev->dev, sizeof(*apu), GFP_KERNEL);
	if (!apu)
		return -ENOMEM;
	apu->rpdev = rpdev;
	INIT_LIST_HEAD(&apu->buffers);

	apu->rproc = apu_get_rproc(rpdev);
	if (IS_ERR_OR_NULL(apu->rproc))
		return PTR_ERR(apu->rproc);

	dev = &apu->dev;
	device_initialize(dev);
	dev->parent = &rpdev->dev;

	cdev_init(&apu->cdev, &rpmsg_eptdev_fops);
	apu->cdev.owner = THIS_MODULE;

	ret = ida_simple_get(&rpmsg_minor_ida, 0, APU_DEV_MAX, GFP_KERNEL);
	if (ret < 0)
		goto free_apu;
	dev->devt = MKDEV(MAJOR(rpmsg_major), ret);

	dev->id = apu->rproc->index;
	dev_set_name(&apu->dev, "apu%d", apu->rproc->index);

	init_waitqueue_head(&apu->waitqueue);
	spin_lock_init(&apu->ctx_lock);
	apu->available_response = 0;
	INIT_LIST_HEAD(&apu->requests);

	ret = cdev_add(&apu->cdev, dev->devt, 1);
	if (ret)
		goto free_minor_ida;

	/* We can now rely on the release function for cleanup */
	dev->release = rpmsg_apu_release_device;

	ret = device_add(dev);
	if (ret) {
		dev_err(&rpdev->dev, "device_add failed: %d\n", ret);
		put_device(dev);
	}

	/* Make device dma capable by inheriting from parent's capabilities */
	set_dma_ops(&rpdev->dev, get_dma_ops(apu->rproc->dev.parent));

	ret = dma_coerce_mask_and_coherent(&rpdev->dev,
					   dma_get_mask(apu->rproc->dev.parent));
	if (ret)
		goto err_put_device;

	rpdev->dev.iommu_group = apu->rproc->dev.parent->iommu_group;

	ret = apu_init_iovad(apu);

	dev_set_drvdata(&rpdev->dev, apu);

	return ret;

err_put_device:
	put_device(dev);
free_minor_ida:
	ida_simple_remove(&rpmsg_minor_ida, MINOR(dev->devt));
free_apu:
	put_device(dev);
	devm_kfree(&rpdev->dev, apu);

	return ret;
}

static void apu_rpmsg_remove(struct rpmsg_device *rpdev)
{
	struct rpmsg_apu *apu = dev_get_drvdata(&rpdev->dev);

	if (apu_iovad)
		kref_put(&apu_iovad->refcount, iova_domain_release);

	device_del(&apu->dev);
	put_device(&apu->dev);
	devm_kfree(&rpdev->dev, apu);
}

static const struct rpmsg_device_id apu_rpmsg_match[] = {
	{ APU_RPMSG_SERVICE_MT8183 },
	{}
};

static struct rpmsg_driver apu_rpmsg_driver = {
	.probe = apu_rpmsg_probe,
	.remove = apu_rpmsg_remove,
	.callback = apu_rpmsg_callback,
	.id_table = apu_rpmsg_match,
	.drv  = {
		.name  = "apu_rpmsg",
	},
};

static int __init apu_rpmsg_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&rpmsg_major, 0, APU_DEV_MAX, "apu");
	if (ret < 0) {
		pr_err("apu: failed to allocate char dev region\n");
		return ret;
	}

	return register_rpmsg_driver(&apu_rpmsg_driver);
}
arch_initcall(apu_rpmsg_init);

static void __exit apu_rpmsg_exit(void)
{
	unregister_rpmsg_driver(&apu_rpmsg_driver);
}
module_exit(apu_rpmsg_exit);


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("APU RPMSG driver");
