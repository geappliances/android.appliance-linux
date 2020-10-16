// SPDX-License-Identifier: GPL-2.0
/*
 * mtk_camsv_video.c - V4L2 video node support
 *
 * Copyright (c) 2020 BayLibre
 */

#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#include "mtk_camsv.h"

static inline struct mtk_camsv_video_device *
file_to_mtk_camsv_node(struct file *__file)
{
	return container_of(video_devdata(__file),
			    struct mtk_camsv_video_device, vdev);
}

static inline struct mtk_camsv_video_device *
mtk_camsv_vbq_to_vdev(struct vb2_queue *__vq)
{
	return container_of(__vq, struct mtk_camsv_video_device, vbq);
}

/* -----------------------------------------------------------------------------
 * Format Information
 */

static const struct mtk_camsv_format_info mtk_camsv_format_info[] = {
	{
		.fourcc = V4L2_PIX_FMT_MTISP_SBGGR12,
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.packed = true,
		.bpp = 12,
	}, {
		.fourcc = V4L2_PIX_FMT_MTISP_SGBRG12,
		.code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.packed = true,
		.bpp = 12,
	}, {
		.fourcc = V4L2_PIX_FMT_MTISP_SGRBG12,
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.packed = true,
		.bpp = 12,
	}, {
		.fourcc = V4L2_PIX_FMT_MTISP_SRGGB12,
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.packed = true,
		.bpp = 12,
	}, {
		.fourcc = V4L2_PIX_FMT_MTISP_SBGGR10,
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.packed = true,
		.bpp = 10,
	}, {
		.fourcc = V4L2_PIX_FMT_MTISP_SGBRG10,
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.packed = true,
		.bpp = 10,
	}, {
		.fourcc = V4L2_PIX_FMT_MTISP_SGRBG10,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.packed = true,
		.bpp = 10,
	}, {
		.fourcc = V4L2_PIX_FMT_MTISP_SRGGB10,
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.packed = true,
		.bpp = 10,
	}, {
		.fourcc = V4L2_PIX_FMT_SBGGR8,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.packed = true,
		.bpp = 8,
	}, {
		.fourcc = V4L2_PIX_FMT_SGBRG8,
		.code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.packed = true,
		.bpp = 8,
	}, {
		.fourcc = V4L2_PIX_FMT_SGRBG8,
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.packed = true,
		.bpp = 8,
	}, {
		.fourcc = V4L2_PIX_FMT_SRGGB8,
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.packed = true,
		.bpp = 8,
	}, {
		.fourcc = V4L2_PIX_FMT_YUYV,
		.code = MEDIA_BUS_FMT_YUYV8_1X16,
		.packed = true,
		.bpp = 16,
	}, {
		.fourcc = V4L2_PIX_FMT_YVYU,
		.code = MEDIA_BUS_FMT_YVYU8_1X16,
		.packed = true,
		.bpp = 16,
	}, {
		.fourcc = V4L2_PIX_FMT_UYVY,
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.packed = true,
		.bpp = 16,
	}, {
		.fourcc = V4L2_PIX_FMT_VYUY,
		.code = MEDIA_BUS_FMT_VYUY8_1X16,
		.packed = true,
		.bpp = 16,
	},
};

static const struct mtk_camsv_format_info *
mtk_camsv_format_info_by_fourcc(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_camsv_format_info); ++i) {
		const struct mtk_camsv_format_info *info =
			&mtk_camsv_format_info[i];

		if (info->fourcc == fourcc)
			return info;
	}

	return NULL;
}

static bool mtk_camsv_dev_find_fmt(const struct mtk_camsv_dev_node_desc *desc,
				   u32 format)
{
	unsigned int i;

	for (i = 0; i < desc->num_fmts; i++) {
		if (desc->fmts[i] == format)
			return true;
	}

	return false;
}

static unsigned int fourcc_to_mbus_format(unsigned int fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_MTISP_SBGGR12:
		return MEDIA_BUS_FMT_SBGGR12_1X12;
	case V4L2_PIX_FMT_MTISP_SGBRG12:
		return MEDIA_BUS_FMT_SGBRG12_1X12;
	case V4L2_PIX_FMT_MTISP_SGRBG12:
		return MEDIA_BUS_FMT_SGRBG12_1X12;
	case V4L2_PIX_FMT_MTISP_SRGGB12:
		return MEDIA_BUS_FMT_SRGGB12_1X12;
	case V4L2_PIX_FMT_MTISP_SBGGR10:
		return MEDIA_BUS_FMT_SBGGR10_1X10;
	case V4L2_PIX_FMT_MTISP_SGBRG10:
		return MEDIA_BUS_FMT_SGBRG10_1X10;
	case V4L2_PIX_FMT_MTISP_SGRBG10:
		return MEDIA_BUS_FMT_SGRBG10_1X10;
	case V4L2_PIX_FMT_MTISP_SRGGB10:
		return MEDIA_BUS_FMT_SRGGB10_1X10;
	case V4L2_PIX_FMT_SBGGR8:
		return MEDIA_BUS_FMT_SBGGR8_1X8;
	case V4L2_PIX_FMT_SGBRG8:
		return MEDIA_BUS_FMT_SGBRG8_1X8;
	case V4L2_PIX_FMT_SGRBG8:
		return MEDIA_BUS_FMT_SGRBG8_1X8;
	case V4L2_PIX_FMT_SRGGB8:
		return MEDIA_BUS_FMT_SRGGB8_1X8;
	case V4L2_PIX_FMT_YUYV:
		return MEDIA_BUS_FMT_YUYV8_1X16;
	case V4L2_PIX_FMT_YVYU:
		return MEDIA_BUS_FMT_YVYU8_1X16;
	case V4L2_PIX_FMT_UYVY:
		return MEDIA_BUS_FMT_UYVY8_1X16;
	case V4L2_PIX_FMT_VYUY:
		return MEDIA_BUS_FMT_VYUY8_1X16;
	default:
		return 0;
	}
}

static unsigned int calc_bpp(unsigned int fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_MTISP_SBGGR12:
	case V4L2_PIX_FMT_MTISP_SGBRG12:
	case V4L2_PIX_FMT_MTISP_SGRBG12:
	case V4L2_PIX_FMT_MTISP_SRGGB12:
		return 12;
	case V4L2_PIX_FMT_MTISP_SBGGR10:
	case V4L2_PIX_FMT_MTISP_SGBRG10:
	case V4L2_PIX_FMT_MTISP_SGRBG10:
	case V4L2_PIX_FMT_MTISP_SRGGB10:
		return 10;
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		return 8;
	default:
		return 0;
	}
}

static unsigned int calc_bpl(unsigned int width, unsigned int fourcc)
{
	unsigned int bpp = calc_bpp(fourcc);

	switch (fourcc) {
	case V4L2_PIX_FMT_MTISP_SBGGR12:
	case V4L2_PIX_FMT_MTISP_SGBRG12:
	case V4L2_PIX_FMT_MTISP_SGRBG12:
	case V4L2_PIX_FMT_MTISP_SRGGB12:
	case V4L2_PIX_FMT_MTISP_SBGGR10:
	case V4L2_PIX_FMT_MTISP_SGBRG10:
	case V4L2_PIX_FMT_MTISP_SGRBG10:
	case V4L2_PIX_FMT_MTISP_SRGGB10:
	case V4L2_PIX_FMT_SBGGR8:
	case V4L2_PIX_FMT_SGBRG8:
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_SRGGB8:
		return DIV_ROUND_UP(width * bpp, 8);
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		return width * 2;
	default:
		return 0;
	}
}

static void calc_bpl_size_pix_mp(struct v4l2_pix_format_mplane *pix_mp)
{
	int i;
	unsigned int bpl = calc_bpl(pix_mp->width, pix_mp->pixelformat);

	for (i = 0; i < pix_mp->num_planes; ++i) {
		pix_mp->plane_fmt[i].bytesperline = ALIGN(bpl, 2);
		pix_mp->plane_fmt[i].sizeimage =
			pix_mp->plane_fmt[i].bytesperline * pix_mp->height;
	}
}

static void
mtk_camsv_dev_load_default_fmt(struct mtk_camsv_dev *cam,
			       struct mtk_camsv_video_device *node)
{
	struct mtk_camsv_p1_device *p1_dev = dev_get_drvdata(cam->dev);
	struct v4l2_pix_format_mplane *fmt = &node->format;

	fmt->num_planes = p1_dev->conf->enableFH ? 2 : 1;
	fmt->pixelformat = node->desc->fmts[0];
	fmt->width = node->desc->def_width;
	fmt->height = node->desc->def_height;

	calc_bpl_size_pix_mp(fmt);

	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->field = V4L2_FIELD_NONE;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	node->fmtinfo = mtk_camsv_format_info_by_fourcc(fmt->pixelformat);
}

/* -----------------------------------------------------------------------------
 * VB2 Queue Operations
 */

static int mtk_camsv_vb2_queue_setup(struct vb2_queue *vq,
				     unsigned int *num_buffers,
				     unsigned int *num_planes,
				     unsigned int sizes[],
				     struct device *alloc_devs[])
{
	struct mtk_camsv_video_device *node = mtk_camsv_vbq_to_vdev(vq);
	unsigned int max_buffer_count = node->desc->max_buf_count;
	const struct v4l2_pix_format_mplane *fmt = &node->format;
	struct mtk_camsv_dev *cam = vb2_get_drv_priv(vq);
	struct mtk_camsv_p1_device *p1_dev = dev_get_drvdata(cam->dev);
	unsigned int size;
	unsigned int np_conf;
	unsigned int mbus_fmt;
	unsigned int i;

	/* Check the limitation of buffer size */
	if (max_buffer_count)
		*num_buffers = clamp_val(*num_buffers, 1, max_buffer_count);

	size = fmt->plane_fmt[0].sizeimage;
	/* Add for q.create_bufs with fmt.g_sizeimage(p) / 2 test */

	np_conf = p1_dev->conf->enableFH ? 2 : 1;

	if (*num_planes == 0) {
		*num_planes = np_conf;
		for (i = 0; i < *num_planes; ++i)
			sizes[i] = size;
	} else if (*num_planes != np_conf || sizes[0] < size) {
		return -EINVAL;
	}

	mbus_fmt = fourcc_to_mbus_format(fmt->pixelformat);
	mtk_camsv_setup(p1_dev, fmt->width, fmt->height,
			fmt->plane_fmt[0].bytesperline, mbus_fmt);

	return 0;
}

static int mtk_camsv_vb2_buf_init(struct vb2_buffer *vb)
{
	struct mtk_camsv_dev_buffer *buf = to_mtk_camsv_dev_buffer(vb);

	buf->daddr = 0ULL;

	return 0;
}

static int mtk_camsv_vb2_buf_prepare(struct vb2_buffer *vb)
{
	struct mtk_camsv_video_device *node =
		mtk_camsv_vbq_to_vdev(vb->vb2_queue);
	struct mtk_camsv_dev *cam = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_camsv_p1_device *p1_dev = dev_get_drvdata(cam->dev);
	struct mtk_camsv_dev_buffer *buf = to_mtk_camsv_dev_buffer(vb);
	const struct v4l2_pix_format_mplane *fmt = &node->format;
	u32 size;
	int i;

	for (i = 0; i < vb->num_planes; i++) {
		size = fmt->plane_fmt[i].sizeimage;
		if (vb2_plane_size(vb, i) < size) {
			dev_err(cam->dev, "plane size is too small:%lu<%u\n",
				vb2_plane_size(vb, i), size);
			return -EINVAL;
		}
	}

	buf->v4l2_buf.field = V4L2_FIELD_NONE;

	for (i = 0; i < vb->num_planes; i++) {
		size = fmt->plane_fmt[i].sizeimage;
		vb2_set_plane_payload(vb, i, size);
	}

	if (buf->daddr == 0ULL) {
		buf->daddr = vb2_dma_contig_plane_dma_addr(vb, 0);
		if (p1_dev->conf->enableFH)
			buf->fhaddr = vb2_dma_contig_plane_dma_addr(vb, 1);
	}

	return 0;
}

static void mtk_camsv_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct mtk_camsv_dev *cam = vb2_get_drv_priv(vb->vb2_queue);
	struct device *dev = cam->dev;
	struct mtk_camsv_p1_device *p1_dev = dev_get_drvdata(dev);
	struct mtk_camsv_dev_buffer *buf = to_mtk_camsv_dev_buffer(vb);

	mutex_lock(&p1_dev->protect_mutex);

	if (pm_runtime_get_sync(dev) < 0) {
		dev_err(dev, "failed to get pm_runtime\n");
		goto out;
	}

	/* added the buffer into the tracking list */
	list_add_tail(&buf->list, &cam->buf_list);

	/* update buffer internal address */
	writel(buf->daddr, p1_dev->regs + CAMSV_FBC_IMGO_ENQ_ADDR);
	if (p1_dev->conf->enableFH)
		writel(buf->fhaddr, p1_dev->regs + CAMSV_IMGO_FH_BASE_ADDR);

	writel(0x1U, p1_dev->regs + CAMSV_IMGO_FBC);

out:
	pm_runtime_put_autosuspend(dev);
	mutex_unlock(&p1_dev->protect_mutex);
}

static void mtk_camsv_vb2_return_all_buffers(struct mtk_camsv_dev *cam,
					     enum vb2_buffer_state state)
{
	struct device *dev = cam->dev;
	struct mtk_camsv_p1_device *p1_dev = dev_get_drvdata(dev);
	struct mtk_camsv_dev_buffer *buf, *buf_prev;

	mutex_lock(&p1_dev->protect_mutex);
	list_for_each_entry_safe(buf, buf_prev, &cam->buf_list, list) {
		buf->daddr = 0ULL;
		list_del(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, state);
	}
	mutex_unlock(&p1_dev->protect_mutex);
}

static void mtk_camsv_cmos_vf_enable(struct mtk_camsv_p1_device *p1_dev,
				     bool enable, bool pak_en)
{
	struct device *dev = p1_dev->dev;
	u32 mask = enable ? (u32)1 : ~(u32)1;
	u32 clk_en;

	mutex_lock(&p1_dev->protect_mutex);
	if (pm_runtime_get_sync(dev) < 0) {
		dev_err(dev, "failed to get pm_runtime\n");
		goto out;
	}

	if (enable) {
		clk_en = CAMSV_TG_DP_CLK_EN | CAMSV_DMA_DP_CLK_EN;
		if (pak_en)
			clk_en |= CAMSV_PAK_DP_CLK_EN;

		writel(clk_en, p1_dev->regs + CAMSV_CLK_EN);
		writel(readl(p1_dev->regs + CAMSV_TG_VF_CON) | mask,
		       p1_dev->regs + CAMSV_TG_VF_CON);
	} else {
		writel(readl(p1_dev->regs + CAMSV_TG_SEN_MODE) & mask,
		       p1_dev->regs + CAMSV_TG_SEN_MODE);
		writel(readl(p1_dev->regs + CAMSV_TG_VF_CON) & mask,
		       p1_dev->regs + CAMSV_TG_VF_CON);
	}

out:
	pm_runtime_put_autosuspend(dev);
	mutex_unlock(&p1_dev->protect_mutex);
}

static int mtk_camsv_verify_format(struct mtk_camsv_dev *cam,
				   struct mtk_camsv_video_device *node)
{
	struct v4l2_pix_format_mplane *pixfmt = &node->format;
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = node->id,
	};
	int ret;

	ret = v4l2_subdev_call(&cam->subdev, pad, get_fmt, NULL, &fmt);
	if (ret < 0)
		return ret == -ENOIOCTLCMD ? -EINVAL : ret;

	if (fourcc_to_mbus_format(pixfmt->pixelformat) != fmt.format.code ||
	    pixfmt->height != fmt.format.height ||
	    pixfmt->width != fmt.format.width)
		return -EINVAL;

	return 0;
}

static int mtk_camsv_vb2_start_streaming(struct vb2_queue *vq,
					 unsigned int count)
{
	struct mtk_camsv_dev *cam = vb2_get_drv_priv(vq);
	struct mtk_camsv_video_device *node = mtk_camsv_vbq_to_vdev(vq);
	struct device *dev = cam->dev;
	struct mtk_camsv_p1_device *p1_dev = dev_get_drvdata(dev);
	int ret;

	if (!node->enabled) {
		dev_err(dev, "Node:%d is not enabled\n", node->id);
		ret = -ENOLINK;
		goto fail_ret_buf;
	}

	/* Enable CMOS and VF */
	mtk_camsv_cmos_vf_enable(p1_dev, true, node->fmtinfo->packed);

	mutex_lock(&cam->op_lock);

	ret = mtk_camsv_verify_format(cam, node);
	if (ret < 0)
		goto fail_unlock;

	/* Start streaming of the whole pipeline now*/
	if (!cam->pipeline.streaming_count) {
		ret = media_pipeline_start(&node->vdev.entity, &cam->pipeline);
		if (ret) {
			dev_err(dev, "failed to start pipeline:%d\n", ret);
			goto fail_unlock;
		}
	}

	/* Media links are fixed after media_pipeline_start */
	cam->stream_count++;

	/* Stream on sub-devices node */
	ret = v4l2_subdev_call(&cam->subdev, video, s_stream, 1);
	if (ret)
		goto fail_no_stream;
	mutex_unlock(&cam->op_lock);

	return 0;

fail_no_stream:
	cam->stream_count--;
	if (cam->stream_count == 0)
		media_pipeline_stop(&node->vdev.entity);
fail_unlock:
	mutex_unlock(&cam->op_lock);
fail_ret_buf:
	mtk_camsv_vb2_return_all_buffers(cam, VB2_BUF_STATE_QUEUED);

	return ret;
}

static void mtk_camsv_vb2_stop_streaming(struct vb2_queue *vq)
{
	struct mtk_camsv_dev *cam = vb2_get_drv_priv(vq);
	struct mtk_camsv_video_device *node = mtk_camsv_vbq_to_vdev(vq);
	struct device *dev = cam->dev;
	struct mtk_camsv_p1_device *p1_dev = dev_get_drvdata(dev);

	/* Disable CMOS and VF */
	mtk_camsv_cmos_vf_enable(p1_dev, false, false);

	mutex_lock(&cam->op_lock);

	v4l2_subdev_call(&cam->subdev, video, s_stream, 0);

	mtk_camsv_vb2_return_all_buffers(cam, VB2_BUF_STATE_ERROR);
	cam->stream_count--;
	if (cam->stream_count) {
		mutex_unlock(&cam->op_lock);
		return;
	}

	mutex_unlock(&cam->op_lock);

	media_pipeline_stop(&node->vdev.entity);
}

static const struct vb2_ops mtk_camsv_vb2_ops = {
	.queue_setup = mtk_camsv_vb2_queue_setup,
	.buf_init = mtk_camsv_vb2_buf_init,
	.buf_prepare = mtk_camsv_vb2_buf_prepare,
	.buf_queue = mtk_camsv_vb2_buf_queue,
	.start_streaming = mtk_camsv_vb2_start_streaming,
	.stop_streaming = mtk_camsv_vb2_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

/* -----------------------------------------------------------------------------
 * V4L2 Video IOCTLs
 */

static int mtk_camsv_vidioc_querycap(struct file *file, void *fh,
				     struct v4l2_capability *cap)
{
	struct mtk_camsv_dev *cam = video_drvdata(file);

	strscpy(cap->driver, dev_driver_string(cam->dev), sizeof(cap->driver));
	strscpy(cap->card, dev_driver_string(cam->dev), sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(cam->dev));

	return 0;
}

static int mtk_camsv_vidioc_enum_fmt(struct file *file, void *fh,
				     struct v4l2_fmtdesc *f)
{
	struct mtk_camsv_video_device *node = file_to_mtk_camsv_node(file);

	if (f->index >= node->desc->num_fmts)
		return -EINVAL;

	/* f->description is filled in v4l_fill_fmtdesc function */
	f->pixelformat = node->desc->fmts[f->index];
	f->flags = 0;

	return 0;
}

static int mtk_camsv_vidioc_g_fmt(struct file *file, void *fh,
				  struct v4l2_format *f)
{
	struct mtk_camsv_video_device *node = file_to_mtk_camsv_node(file);

	f->fmt.pix_mp = node->format;

	return 0;
}

static int mtk_camsv_vidioc_try_fmt(struct file *file, void *fh,
				    struct v4l2_format *f)
{
	struct mtk_camsv_dev *cam = video_drvdata(file);
	struct mtk_camsv_video_device *node = file_to_mtk_camsv_node(file);
	struct mtk_camsv_p1_device *p1_dev = dev_get_drvdata(cam->dev);
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;

	/* Validate pixelformat */
	if (!mtk_camsv_dev_find_fmt(node->desc, pix_mp->pixelformat))
		pix_mp->pixelformat = node->desc->fmts[0];

	pix_mp->width = clamp_val(pix_mp->width, IMG_MIN_WIDTH, IMG_MAX_WIDTH);
	pix_mp->height = clamp_val(pix_mp->height, IMG_MIN_HEIGHT,
				   IMG_MAX_HEIGHT);

	pix_mp->num_planes = p1_dev->conf->enableFH ? 2 : 1;

	calc_bpl_size_pix_mp(pix_mp);

	/* Constant format fields */
	pix_mp->colorspace = V4L2_COLORSPACE_SRGB;
	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix_mp->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix_mp->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	return 0;
}

static int mtk_camsv_vidioc_s_fmt(struct file *file, void *fh,
				  struct v4l2_format *f)
{
	struct mtk_camsv_dev *cam = video_drvdata(file);
	struct mtk_camsv_video_device *node = file_to_mtk_camsv_node(file);
	int ret;

	if (vb2_is_busy(node->vdev.queue)) {
		dev_dbg(cam->dev, "%s: queue is busy\n", __func__);
		return -EBUSY;
	}

	ret = mtk_camsv_vidioc_try_fmt(file, fh, f);
	if (ret)
		return ret;

	/* Configure to video device */
	node->format = f->fmt.pix_mp;
	node->fmtinfo =
		mtk_camsv_format_info_by_fourcc(f->fmt.pix_mp.pixelformat);

	return 0;
}

static int mtk_camsv_vidioc_enum_framesizes(struct file *filp, void *priv,
					    struct v4l2_frmsizeenum *sizes)
{
	struct mtk_camsv_video_device *node = file_to_mtk_camsv_node(filp);

	if (sizes->index)
		return -EINVAL;

	if (!mtk_camsv_dev_find_fmt(node->desc, sizes->pixel_format))
		return -EINVAL;

	sizes->type = node->desc->frmsizes->type;
	memcpy(&sizes->stepwise, &node->desc->frmsizes->stepwise,
	       sizeof(sizes->stepwise));

	return 0;
}

static const struct v4l2_ioctl_ops mtk_camsv_v4l2_vcap_ioctl_ops = {
	.vidioc_querycap = mtk_camsv_vidioc_querycap,
	.vidioc_enum_framesizes = mtk_camsv_vidioc_enum_framesizes,
	.vidioc_enum_fmt_vid_cap = mtk_camsv_vidioc_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = mtk_camsv_vidioc_g_fmt,
	.vidioc_s_fmt_vid_cap_mplane = mtk_camsv_vidioc_s_fmt,
	.vidioc_try_fmt_vid_cap_mplane = mtk_camsv_vidioc_try_fmt,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations mtk_camsv_v4l2_fops = {
	.unlocked_ioctl = video_ioctl2,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = v4l2_compat_ioctl32,
#endif
};

/* -----------------------------------------------------------------------------
 * Init & Cleanup
 */

int mtk_camsv_video_register(struct mtk_camsv_dev *cam,
			     struct mtk_camsv_video_device *node)
{
	struct device *dev = cam->dev;
	struct video_device *vdev = &node->vdev;
	struct vb2_queue *vbq = &node->vbq;
	unsigned int output = V4L2_TYPE_IS_OUTPUT(node->desc->buf_type);
	unsigned int link_flags = node->desc->link_flags;
	int ret;

	/* Initialize mtk_camsv_video_device */
	if (link_flags & MEDIA_LNK_FL_IMMUTABLE)
		node->enabled = true;
	else
		node->enabled = false;
	mtk_camsv_dev_load_default_fmt(cam, node);

	cam->subdev_pads[MTK_CAMSV_CIO_PAD_NODE(node->id)].flags =
		output ? MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;

	/* Initialize media entities */
	ret = media_entity_pads_init(&vdev->entity, 1, &node->vdev_pad);
	if (ret) {
		dev_err(dev, "failed to initialize media pad:%d\n", ret);
		return ret;
	}
	node->vdev_pad.flags = output ? MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK;

	vbq->type = node->desc->buf_type;
	vbq->io_modes = VB2_MMAP | VB2_DMABUF;
	vbq->dev = dev;
	vbq->ops = &mtk_camsv_vb2_ops;
	vbq->mem_ops = &vb2_dma_contig_memops;
	vbq->buf_struct_size = sizeof(struct mtk_camsv_dev_buffer);
	vbq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	/* No minimum buffers limitation */
	vbq->min_buffers_needed = 0;
	vbq->drv_priv = cam;

	vbq->lock = &node->vdev_lock;
	ret = vb2_queue_init(vbq);
	if (ret) {
		dev_err(dev, "failed to init. vb2 queue:%d\n", ret);
		goto fail_media_clean;
	}

	/* Initialize vdev */
	snprintf(vdev->name, sizeof(vdev->name), "%s %s",
		 dev_driver_string(dev), node->desc->name);
	/* set cap/type/ioctl_ops of the video device */
	vdev->device_caps = node->desc->cap | V4L2_CAP_STREAMING;
	vdev->ioctl_ops = node->desc->ioctl_ops;
	vdev->fops = &mtk_camsv_v4l2_fops;
	vdev->release = video_device_release_empty;
	vdev->lock = &node->vdev_lock;
	vdev->v4l2_dev = &cam->v4l2_dev;
	vdev->queue = &node->vbq;
	vdev->vfl_dir = output ? VFL_DIR_TX : VFL_DIR_RX;
	vdev->entity.function = MEDIA_ENT_F_IO_V4L;
	vdev->entity.ops = NULL;
	video_set_drvdata(vdev, cam);

	/* Initialize miscellaneous variables */
	mutex_init(&node->vdev_lock);
	INIT_LIST_HEAD(&cam->buf_list);

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		dev_err(dev, "failed to register vde:%d\n", ret);
		goto fail_vb2_rel;
	}

	/* Create link between video node and the subdev pad */
	if (output)
		ret = media_create_pad_link(&vdev->entity, 0,
					    &cam->subdev.entity,
					    MTK_CAMSV_CIO_PAD_NODE(node->id),
					    link_flags);
	else
		ret = media_create_pad_link(&cam->subdev.entity,
					    MTK_CAMSV_CIO_PAD_NODE(node->id),
					    &vdev->entity, 0, link_flags);

	if (ret)
		goto fail_vdev_ureg;

	return 0;

fail_vdev_ureg:
	video_unregister_device(vdev);
fail_vb2_rel:
	mutex_destroy(&node->vdev_lock);
	vb2_queue_release(vbq);
fail_media_clean:
	media_entity_cleanup(&vdev->entity);

	return ret;
}

void mtk_camsv_video_unregister(struct mtk_camsv_video_device *node)
{
	video_unregister_device(&node->vdev);
	vb2_queue_release(&node->vbq);
	media_entity_cleanup(&node->vdev.entity);
	mutex_destroy(&node->vdev_lock);
}

static const u32 stream_out_fmts[] = {
	/* The 1st entry is the default image format */
	V4L2_PIX_FMT_MTISP_SGRBG12,
	V4L2_PIX_FMT_MTISP_SBGGR12,
	V4L2_PIX_FMT_MTISP_SGBRG12,
	V4L2_PIX_FMT_MTISP_SRGGB12,
	V4L2_PIX_FMT_MTISP_SGRBG10,
	V4L2_PIX_FMT_MTISP_SBGGR10,
	V4L2_PIX_FMT_MTISP_SGBRG10,
	V4L2_PIX_FMT_MTISP_SRGGB10,
	V4L2_PIX_FMT_SBGGR8,
	V4L2_PIX_FMT_SGBRG8,
	V4L2_PIX_FMT_SGRBG8,
	V4L2_PIX_FMT_SRGGB8,
	V4L2_PIX_FMT_UYVY,
	V4L2_PIX_FMT_VYUY,
	V4L2_PIX_FMT_YUYV,
	V4L2_PIX_FMT_YVYU,
};

static const struct mtk_camsv_dev_node_desc capture_queues[] = {
	{
		.id = MTK_CAMSV_P1_MAIN_STREAM_OUT,
		.name = "main stream",
		.cap = V4L2_CAP_VIDEO_CAPTURE_MPLANE,
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.link_flags = MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED,
		.fmts = stream_out_fmts,
		.num_fmts = ARRAY_SIZE(stream_out_fmts),
		.def_width = 1920,
		.def_height = 1080,
		.ioctl_ops = &mtk_camsv_v4l2_vcap_ioctl_ops,
		.frmsizes =
			&(struct v4l2_frmsizeenum){
				.index = 0,
				.type = V4L2_FRMSIZE_TYPE_CONTINUOUS,
				.stepwise = {
					.max_width = IMG_MAX_WIDTH,
					.min_width = IMG_MIN_WIDTH,
					.max_height = IMG_MAX_HEIGHT,
					.min_height = IMG_MIN_HEIGHT,
					.step_height = 1,
					.step_width = 1,
				},
			},
	},
};

void mtk_camsv_video_init_nodes(struct mtk_camsv_dev *cam)
{
	unsigned int node_idx;
	int i;

	node_idx = 0;
	for (i = 0; i < ARRAY_SIZE(capture_queues); i++)
		cam->vdev_nodes[node_idx++].desc = &capture_queues[i];

	vb2_dma_contig_set_max_seg_size(cam->dev, DMA_BIT_MASK(32));
}

void mtk_camsv_video_cleanup_nodes(struct mtk_camsv_dev *cam)
{
	vb2_dma_contig_clear_max_seg_size(cam->dev);
}
