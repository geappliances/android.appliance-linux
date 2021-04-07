// SPDX-License-Identifier: GPL-2.0
/*
 * mtk_camsv_video.c - V4L2 video node support
 *
 * Copyright (c) 2020 BayLibre
 */

#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/version.h>

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
file_to_mtk_camsv_video_device(struct file *__file)
{
	return container_of(video_devdata(__file),
			    struct mtk_camsv_video_device, vdev);
}

static inline struct mtk_camsv_video_device *
vb2_queue_to_mtk_camsv_video_device(struct vb2_queue *vq)
{
	return container_of(vq, struct mtk_camsv_video_device, vbq);
}

static inline struct mtk_camsv_dev_buffer *
to_mtk_camsv_dev_buffer(struct vb2_buffer *buf)
{
	return container_of(buf, struct mtk_camsv_dev_buffer, v4l2_buf.vb2_buf);
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

static const struct mtk_camsv_format_info *
mtk_camsv_format_info_by_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_camsv_format_info); ++i) {
		const struct mtk_camsv_format_info *info =
			&mtk_camsv_format_info[i];

		if (info->code == code)
			return info;
	}

	return NULL;
}

static bool mtk_camsv_dev_find_fmt(const struct mtk_camsv_vdev_desc *desc,
				   u32 format)
{
	unsigned int i;

	for (i = 0; i < desc->num_fmts; i++) {
		if (desc->fmts[i] == format)
			return true;
	}

	return false;
}

static void calc_bpl_size_pix_mp(const struct mtk_camsv_format_info *fmtinfo,
				 struct v4l2_pix_format_mplane *pix_mp)
{
	unsigned int bpl;
	unsigned int i;

	bpl = ALIGN(DIV_ROUND_UP(pix_mp->width * fmtinfo->bpp, 8), 2);

	for (i = 0; i < pix_mp->num_planes; ++i) {
		pix_mp->plane_fmt[i].bytesperline = bpl;
		pix_mp->plane_fmt[i].sizeimage = bpl * pix_mp->height;
	}
}

static void mtk_camsv_dev_load_default_fmt(struct mtk_camsv_dev *cam)
{
	struct mtk_camsv_video_device *vdev = &cam->vdev;
	struct v4l2_pix_format_mplane *fmt = &vdev->format;

	fmt->num_planes = cam->conf->enableFH ? 2 : 1;
	fmt->pixelformat = vdev->desc->fmts[0];
	fmt->width = vdev->desc->def_width;
	fmt->height = vdev->desc->def_height;

	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->field = V4L2_FIELD_NONE;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	vdev->fmtinfo = mtk_camsv_format_info_by_fourcc(fmt->pixelformat);

	calc_bpl_size_pix_mp(vdev->fmtinfo, fmt);
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
	struct mtk_camsv_video_device *vdev =
		vb2_queue_to_mtk_camsv_video_device(vq);
	unsigned int max_buffer_count = vdev->desc->max_buf_count;
	const struct v4l2_pix_format_mplane *fmt = &vdev->format;
	struct mtk_camsv_dev *cam = vb2_get_drv_priv(vq);
	unsigned int size;
	unsigned int np_conf;
	unsigned int i;

	/* Check the limitation of buffer size */
	if (max_buffer_count)
		*num_buffers = clamp_val(*num_buffers, 1, max_buffer_count);

	size = fmt->plane_fmt[0].sizeimage;
	/* Add for q.create_bufs with fmt.g_sizeimage(p) / 2 test */

	np_conf = cam->conf->enableFH ? 2 : 1;

	if (*num_planes == 0) {
		*num_planes = np_conf;
		for (i = 0; i < *num_planes; ++i)
			sizes[i] = size;
	} else if (*num_planes != np_conf || sizes[0] < size) {
		return -EINVAL;
	}

	mtk_camsv_setup(cam, fmt->width, fmt->height,
			fmt->plane_fmt[0].bytesperline, vdev->fmtinfo->code);

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
	struct mtk_camsv_video_device *vdev =
		vb2_queue_to_mtk_camsv_video_device(vb->vb2_queue);
	struct mtk_camsv_dev *cam = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_camsv_dev_buffer *buf = to_mtk_camsv_dev_buffer(vb);
	const struct v4l2_pix_format_mplane *fmt = &vdev->format;
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
		if (cam->conf->enableFH)
			buf->fhaddr = vb2_dma_contig_plane_dma_addr(vb, 1);
	}

	return 0;
}

static void mtk_camsv_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct mtk_camsv_dev *cam = vb2_get_drv_priv(vb->vb2_queue);
	struct device *dev = cam->dev;
	struct mtk_camsv_dev_buffer *buf = to_mtk_camsv_dev_buffer(vb);

	mutex_lock(&cam->protect_mutex);

	if (pm_runtime_get_sync(dev) < 0) {
		dev_err(dev, "failed to get pm_runtime\n");
		goto out;
	}

	/* added the buffer into the tracking list */
	list_add_tail(&buf->list, &cam->buf_list);

	/* update buffer internal address */
	writel(buf->daddr, cam->regs + CAMSV_FBC_IMGO_ENQ_ADDR);
	if (cam->conf->enableFH)
		writel(buf->fhaddr, cam->regs + CAMSV_IMGO_FH_BASE_ADDR);

	writel(0x1U, cam->regs + CAMSV_IMGO_FBC);

out:
	pm_runtime_put_autosuspend(dev);
	mutex_unlock(&cam->protect_mutex);
}

static void mtk_camsv_vb2_return_all_buffers(struct mtk_camsv_dev *cam,
					     enum vb2_buffer_state state)
{
	struct mtk_camsv_dev_buffer *buf, *buf_prev;

	mutex_lock(&cam->protect_mutex);
	list_for_each_entry_safe(buf, buf_prev, &cam->buf_list, list) {
		buf->daddr = 0ULL;
		list_del(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, state);
	}
	mutex_unlock(&cam->protect_mutex);
}

static void mtk_camsv_cmos_vf_enable(struct mtk_camsv_dev *camsv_dev,
				     bool enable, bool pak_en)
{
	struct device *dev = camsv_dev->dev;
	u32 mask = enable ? (u32)1 : ~(u32)1;
	u32 clk_en;

	mutex_lock(&camsv_dev->protect_mutex);
	if (pm_runtime_get_sync(dev) < 0) {
		dev_err(dev, "failed to get pm_runtime\n");
		goto out;
	}

	if (enable) {
		clk_en = CAMSV_TG_DP_CLK_EN | CAMSV_DMA_DP_CLK_EN;
		if (pak_en)
			clk_en |= CAMSV_PAK_DP_CLK_EN;

		writel(clk_en, camsv_dev->regs + CAMSV_CLK_EN);
		writel(readl(camsv_dev->regs + CAMSV_TG_VF_CON) | mask,
		       camsv_dev->regs + CAMSV_TG_VF_CON);
	} else {
		writel(readl(camsv_dev->regs + CAMSV_TG_SEN_MODE) & mask,
		       camsv_dev->regs + CAMSV_TG_SEN_MODE);
		writel(readl(camsv_dev->regs + CAMSV_TG_VF_CON) & mask,
		       camsv_dev->regs + CAMSV_TG_VF_CON);
	}

out:
	pm_runtime_put_autosuspend(dev);
	mutex_unlock(&camsv_dev->protect_mutex);
}

static int mtk_camsv_verify_format(struct mtk_camsv_dev *cam)
{
	struct mtk_camsv_video_device *vdev = &cam->vdev;
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = MTK_CAMSV_CIO_PAD_VIDEO,
	};
	int ret;

	ret = v4l2_subdev_call(&cam->subdev, pad, get_fmt, NULL, &fmt);
	if (ret < 0)
		return ret == -ENOIOCTLCMD ? -EINVAL : ret;

	if (vdev->fmtinfo->code != fmt.format.code ||
	    vdev->format.height != fmt.format.height ||
	    vdev->format.width != fmt.format.width)
		return -EINVAL;

	return 0;
}

static int mtk_camsv_vb2_start_streaming(struct vb2_queue *vq,
					 unsigned int count)
{
	struct mtk_camsv_dev *cam = vb2_get_drv_priv(vq);
	struct mtk_camsv_video_device *vdev =
		vb2_queue_to_mtk_camsv_video_device(vq);
	struct device *dev = cam->dev;
	int ret;

	if (!vdev->enabled) {
		dev_err(dev, "Video device is not enabled\n");
		ret = -ENOLINK;
		goto fail_ret_buf;
	}

	/* Enable CMOS and VF */
	mtk_camsv_cmos_vf_enable(cam, true, vdev->fmtinfo->packed);

	mutex_lock(&cam->op_lock);

	ret = mtk_camsv_verify_format(cam);
	if (ret < 0)
		goto fail_unlock;

	/* Start streaming of the whole pipeline now*/
	if (!cam->pipeline.streaming_count) {
		ret = media_pipeline_start(vdev->vdev.entity.pads,
					   &cam->pipeline);
		if (ret) {
			dev_err(dev, "failed to start pipeline:%d\n", ret);
			goto fail_unlock;
		}
	}

	/* Media links are fixed after media_pipeline_start */
	cam->stream_count++;

	cam->sequence = (unsigned int)-1;

	/* Stream on the sub-device */
	ret = v4l2_subdev_call(&cam->subdev, video, s_stream, 1);
	if (ret)
		goto fail_no_stream;
	mutex_unlock(&cam->op_lock);

	return 0;

fail_no_stream:
	cam->stream_count--;
	if (cam->stream_count == 0)
		media_pipeline_stop(vdev->vdev.entity.pads);
fail_unlock:
	mutex_unlock(&cam->op_lock);
fail_ret_buf:
	mtk_camsv_vb2_return_all_buffers(cam, VB2_BUF_STATE_QUEUED);

	return ret;
}

static void mtk_camsv_vb2_stop_streaming(struct vb2_queue *vq)
{
	struct mtk_camsv_dev *cam = vb2_get_drv_priv(vq);
	struct mtk_camsv_video_device *vdev =
		vb2_queue_to_mtk_camsv_video_device(vq);

	/* Disable CMOS and VF */
	mtk_camsv_cmos_vf_enable(cam, false, false);

	mutex_lock(&cam->op_lock);

	v4l2_subdev_call(&cam->subdev, video, s_stream, 0);

	mtk_camsv_vb2_return_all_buffers(cam, VB2_BUF_STATE_ERROR);
	cam->stream_count--;
	if (cam->stream_count) {
		mutex_unlock(&cam->op_lock);
		return;
	}

	mutex_unlock(&cam->op_lock);

	media_pipeline_stop(vdev->vdev.entity.pads);
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

	/* Minimum v4l2 api kernel version required by libcamera is 5.0.0 */
	if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 0, 0))
		cap->version = KERNEL_VERSION(5, 0, 0);

	return 0;
}

static int mtk_camsv_vidioc_enum_fmt(struct file *file, void *fh,
				     struct v4l2_fmtdesc *f)
{
	struct mtk_camsv_video_device *vdev = file_to_mtk_camsv_video_device(file);
	const struct mtk_camsv_format_info *fmtinfo;
	unsigned int i;

	/* If mbus_code is not set enumerate all supported formats. */
	if (!f->mbus_code) {
		if (f->index >= vdev->desc->num_fmts)
			return -EINVAL;

		/* f->description is filled in v4l_fill_fmtdesc function */
		f->pixelformat = vdev->desc->fmts[f->index];
		f->flags = 0;

		return 0;
	}

	/*
	 * Otherwise only enumerate supported pixel formats corresponding to
	 * that bus code.
	 */
	if (f->index)
		return -EINVAL;

	fmtinfo = mtk_camsv_format_info_by_code(f->mbus_code);
	if (!fmtinfo)
		return -EINVAL;

	for (i = 0; i < vdev->desc->num_fmts; ++i) {
		if (vdev->desc->fmts[i] == fmtinfo->fourcc) {
			f->pixelformat = fmtinfo->fourcc;
			f->flags = 0;
			return 0;
		}
	}

	return -EINVAL;
}

static int mtk_camsv_vidioc_g_fmt(struct file *file, void *fh,
				  struct v4l2_format *f)
{
	struct mtk_camsv_video_device *vdev = file_to_mtk_camsv_video_device(file);

	f->fmt.pix_mp = vdev->format;

	return 0;
}

static int mtk_camsv_vidioc_try_fmt(struct file *file, void *fh,
				    struct v4l2_format *f)
{
	struct mtk_camsv_dev *cam = video_drvdata(file);
	struct mtk_camsv_video_device *vdev = file_to_mtk_camsv_video_device(file);
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	const struct mtk_camsv_format_info *fmtinfo;

	/* Validate pixelformat */
	if (!mtk_camsv_dev_find_fmt(vdev->desc, pix_mp->pixelformat))
		pix_mp->pixelformat = vdev->desc->fmts[0];

	pix_mp->width = clamp_val(pix_mp->width, IMG_MIN_WIDTH, IMG_MAX_WIDTH);
	pix_mp->height = clamp_val(pix_mp->height, IMG_MIN_HEIGHT,
				   IMG_MAX_HEIGHT);

	pix_mp->num_planes = cam->conf->enableFH ? 2 : 1;

	fmtinfo = mtk_camsv_format_info_by_fourcc(pix_mp->pixelformat);
	calc_bpl_size_pix_mp(fmtinfo, pix_mp);

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
	struct mtk_camsv_video_device *vdev = file_to_mtk_camsv_video_device(file);
	int ret;

	if (vb2_is_busy(vdev->vdev.queue)) {
		dev_dbg(cam->dev, "%s: queue is busy\n", __func__);
		return -EBUSY;
	}

	ret = mtk_camsv_vidioc_try_fmt(file, fh, f);
	if (ret)
		return ret;

	/* Configure to video device */
	vdev->format = f->fmt.pix_mp;
	vdev->fmtinfo =
		mtk_camsv_format_info_by_fourcc(f->fmt.pix_mp.pixelformat);

	return 0;
}

static int mtk_camsv_vidioc_enum_framesizes(struct file *file, void *priv,
					    struct v4l2_frmsizeenum *sizes)
{
	struct mtk_camsv_video_device *vdev = file_to_mtk_camsv_video_device(file);

	if (sizes->index)
		return -EINVAL;

	if (!mtk_camsv_dev_find_fmt(vdev->desc, sizes->pixel_format))
		return -EINVAL;

	sizes->type = vdev->desc->frmsizes->type;
	memcpy(&sizes->stepwise, &vdev->desc->frmsizes->stepwise,
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

int mtk_camsv_video_register(struct mtk_camsv_dev *cam)
{
	struct device *dev = cam->dev;
	struct mtk_camsv_video_device *cam_vdev = &cam->vdev;
	struct video_device *vdev = &cam_vdev->vdev;
	struct vb2_queue *vbq = &cam_vdev->vbq;
	unsigned int output = V4L2_TYPE_IS_OUTPUT(cam_vdev->desc->buf_type);
	unsigned int link_flags = cam_vdev->desc->link_flags;
	int ret;

	/* Initialize mtk_camsv_video_device */
	if (link_flags & MEDIA_LNK_FL_IMMUTABLE)
		cam_vdev->enabled = true;
	else
		cam_vdev->enabled = false;
	mtk_camsv_dev_load_default_fmt(cam);

	cam->subdev_pads[MTK_CAMSV_CIO_PAD_VIDEO].flags =
		output ? MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;

	/* Initialize media entities */
	ret = media_entity_pads_init(&vdev->entity, 1, &cam_vdev->vdev_pad);
	if (ret) {
		dev_err(dev, "failed to initialize media pad:%d\n", ret);
		return ret;
	}
	cam_vdev->vdev_pad.flags = output ? MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK;

	vbq->type = cam_vdev->desc->buf_type;
	vbq->io_modes = VB2_MMAP | VB2_DMABUF;
	vbq->dev = dev;
	vbq->ops = &mtk_camsv_vb2_ops;
	vbq->mem_ops = &vb2_dma_contig_memops;
	vbq->buf_struct_size = sizeof(struct mtk_camsv_dev_buffer);
	/*
	 * TODO: The hardware supports SOF interrupts, switch to a SOF
	 * timestamp source would give better accuracy, but first requires
	 * extending the V4L2 API to support it.
	 */
	vbq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
			     | V4L2_BUF_FLAG_TSTAMP_SRC_EOF;

	/* No minimum buffers limitation */
	vbq->min_buffers_needed = 0;
	vbq->drv_priv = cam;

	vbq->lock = &cam_vdev->vdev_lock;
	ret = vb2_queue_init(vbq);
	if (ret) {
		dev_err(dev, "failed to init. vb2 queue:%d\n", ret);
		goto fail_media_clean;
	}

	/* Initialize vdev */
	snprintf(vdev->name, sizeof(vdev->name), "%s %s",
		 dev_name(dev), cam_vdev->desc->name);

	/* Set cap/type/ioctl_ops of the video device */
	vdev->device_caps = cam_vdev->desc->cap | V4L2_CAP_STREAMING
			  | V4L2_CAP_IO_MC;
	vdev->ioctl_ops = cam_vdev->desc->ioctl_ops;
	vdev->fops = &mtk_camsv_v4l2_fops;
	vdev->release = video_device_release_empty;
	vdev->lock = &cam_vdev->vdev_lock;
	vdev->v4l2_dev = cam->subdev.v4l2_dev;
	vdev->queue = &cam_vdev->vbq;
	vdev->vfl_dir = output ? VFL_DIR_TX : VFL_DIR_RX;
	vdev->entity.function = MEDIA_ENT_F_IO_V4L;
	vdev->entity.ops = NULL;
	video_set_drvdata(vdev, cam);

	/* Initialize miscellaneous variables */
	mutex_init(&cam_vdev->vdev_lock);
	INIT_LIST_HEAD(&cam->buf_list);

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		dev_err(dev, "failed to register vde:%d\n", ret);
		goto fail_vb2_rel;
	}

	/* Create link between the video pad and the subdev pad. */
	if (output)
		ret = media_create_pad_link(&vdev->entity, 0,
					    &cam->subdev.entity,
					    MTK_CAMSV_CIO_PAD_VIDEO,
					    link_flags);
	else
		ret = media_create_pad_link(&cam->subdev.entity,
					    MTK_CAMSV_CIO_PAD_VIDEO,
					    &vdev->entity, 0, link_flags);

	if (ret)
		goto fail_vdev_ureg;

	return 0;

fail_vdev_ureg:
	video_unregister_device(vdev);
fail_vb2_rel:
	mutex_destroy(&cam_vdev->vdev_lock);
	vb2_queue_release(vbq);
fail_media_clean:
	media_entity_cleanup(&vdev->entity);

	return ret;
}

void mtk_camsv_video_unregister(struct mtk_camsv_video_device *vdev)
{
	video_unregister_device(&vdev->vdev);
	vb2_queue_release(&vdev->vbq);
	media_entity_cleanup(&vdev->vdev.entity);
	mutex_destroy(&vdev->vdev_lock);
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

static const struct mtk_camsv_vdev_desc video_stream = {
	.name = "video stream",
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
};

void mtk_camsv_video_init_nodes(struct mtk_camsv_dev *cam)
{
	cam->vdev.desc = &video_stream;
	vb2_dma_contig_set_max_seg_size(cam->dev, DMA_BIT_MASK(32));
}
