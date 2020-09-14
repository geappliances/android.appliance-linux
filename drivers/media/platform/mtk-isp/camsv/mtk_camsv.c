// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 BayLibre
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/videodev2.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-dma-contig.h>

#include "mtk_camsv.h"

#define MTK_CAMSV_CIO_PAD_SRC 4
#define MTK_CAMSV_CIO_PAD_SINK 11

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

static const struct v4l2_format *
mtk_camsv_dev_find_fmt(struct mtk_camsv_dev_node_desc *desc, u32 format)
{
	int i;
	const struct v4l2_format *dev_fmt;

	for (i = 0; i < desc->num_fmts; i++) {
		dev_fmt = &desc->fmts[i];
		if (dev_fmt->fmt.pix_mp.pixelformat == format)
			return dev_fmt;
	}

	return NULL;
}

static unsigned int fourcc_to_mbus_format(unsigned int fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SRGGB10P:
		return MEDIA_BUS_FMT_SRGGB10_1X10;
	case V4L2_PIX_FMT_SGRBG8:
		return MEDIA_BUS_FMT_SGRBG8_1X8;
	case V4L2_PIX_FMT_YUYV:
		return MEDIA_BUS_FMT_YUYV8_2X8;
	case V4L2_PIX_FMT_YVYU:
		return MEDIA_BUS_FMT_YVYU8_2X8;
	case V4L2_PIX_FMT_UYVY:
		return MEDIA_BUS_FMT_UYVY8_2X8;
	case V4L2_PIX_FMT_VYUY:
		return MEDIA_BUS_FMT_VYUY8_2X8;
	default:
		return 0;
	}
}

static bool is_format_pak(unsigned int mbus_fmt)
{
	switch (mbus_fmt) {
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
		return true;
	case MEDIA_BUS_FMT_YUYV8_2X8:
		return false;
	default:
		return true;
	}
}

static unsigned int calc_bpp(unsigned int fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SRGGB10P:
		return 10;
	case V4L2_PIX_FMT_SGRBG8:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		return 8;
	default:
		return 0;
	}
}

static unsigned int calc_bpl(unsigned int width, unsigned int fourcc)
{
	unsigned int bpp = calc_bpp(fourcc);

	switch (fourcc) {
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SRGGB10P:
	case V4L2_PIX_FMT_SGRBG8:
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
			       struct mtk_camsv_dev_node_desc *queue_desc,
			       struct v4l2_format *dest)
{
	struct mtk_camsv_p1_device *p1_dev = dev_get_drvdata(cam->dev);

	const struct v4l2_format *default_fmt =
		&queue_desc->fmts[queue_desc->default_fmt_idx];

	dest->type = queue_desc->buf_type;
	dest->fmt.pix_mp.num_planes = p1_dev->conf->enableFH ? 2 : 1;
	dest->fmt.pix_mp.pixelformat = default_fmt->fmt.pix_mp.pixelformat;
	dest->fmt.pix_mp.width = default_fmt->fmt.pix_mp.width;
	dest->fmt.pix_mp.height = default_fmt->fmt.pix_mp.height;

	calc_bpl_size_pix_mp(&dest->fmt.pix_mp);

	dest->fmt.pix_mp.colorspace = V4L2_COLORSPACE_SRGB;
	dest->fmt.pix_mp.field = V4L2_FIELD_NONE;
	dest->fmt.pix_mp.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	dest->fmt.pix_mp.quantization = V4L2_QUANTIZATION_DEFAULT;
	dest->fmt.pix_mp.xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static struct v4l2_subdev *
mtk_camsv_cio_get_active_sensor(struct mtk_camsv_dev *cam)
{
	struct media_device *mdev = cam->seninf->entity.graph_obj.mdev;
	struct device *dev = cam->dev;
	struct media_entity *entity;
	struct v4l2_subdev *sensor;

	sensor = NULL;
	media_device_for_each_entity(entity, mdev) {
		if (entity->function == MEDIA_ENT_F_CAM_SENSOR &&
		    entity->stream_count) {
			sensor = media_entity_to_v4l2_subdev(entity);
			dev_dbg(dev, "sensor found: %s\n", entity->name);
			break;
		}
	}

	if (!sensor)
		dev_err(dev, "no sensor connected\n");

	return sensor;
}

static int mtk_camsv_cio_stream_on(struct mtk_camsv_dev *cam)
{
	struct device *dev = cam->dev;
	int ret;

	if (!cam->seninf) {
		dev_err(dev, "no seninf connected\n");
		return -ENODEV;
	}

	/* Seninf must stream on first */
	ret = v4l2_subdev_call(cam->seninf, video, s_stream, 1);
	if (ret) {
		dev_err(dev, "failed to stream on %s:%d\n",
			cam->seninf->entity.name, ret);
		goto fail_seninf_off;
	}

	/* Get active sensor from graph topology */
	cam->sensor = mtk_camsv_cio_get_active_sensor(cam);
	if (!cam->sensor || cam->is_testmode)
		goto no_sensor;

	ret = v4l2_subdev_call(cam->sensor, video, s_stream, 1);
	if (ret) {
		dev_err(dev, "failed to stream on %s:%d\n",
			cam->sensor->entity.name, ret);
		goto fail_sensor_off;
	}

	/* No sensor, use seninf test pattern mode */
no_sensor:
	cam->streaming = true;

	return 0;

fail_sensor_off:
	v4l2_subdev_call(cam->sensor, video, s_stream, 0);
fail_seninf_off:
	v4l2_subdev_call(cam->seninf, video, s_stream, 0);

	return ret;
}

static int mtk_camsv_cio_stream_off(struct mtk_camsv_dev *cam)
{
	struct device *dev = cam->dev;
	int ret;

	if (cam->sensor && !cam->is_testmode) {
		ret = v4l2_subdev_call(cam->sensor, video, s_stream, 0);
		if (ret) {
			dev_err(dev, "failed to stream off %s:%d\n",
				cam->sensor->entity.name, ret);
			return ret;
		}
	}

	if (cam->seninf) {
		ret = v4l2_subdev_call(cam->seninf, video, s_stream, 0);
		if (ret) {
			dev_err(dev, "failed to stream off %s:%d\n",
				cam->seninf->entity.name, ret);
			return ret;
		}
	}

	cam->streaming = false;

	return 0;
}

static int mtk_camsv_sd_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mtk_camsv_dev *cam =
		container_of(sd, struct mtk_camsv_dev, subdev);

	if (enable) {
		/* Align vb2_core_streamon design */
		if (cam->streaming) {
			dev_warn(cam->dev, "already streaming on\n");
			return 0;
		}
		return mtk_camsv_cio_stream_on(cam);
	}

	if (!cam->streaming) {
		dev_warn(cam->dev, "already streaming off\n");
		return 0;
	}

	return mtk_camsv_cio_stream_off(cam);
}

static int mtk_camsv_sd_subscribe_event(struct v4l2_subdev *subdev,
					struct v4l2_fh *fh,
					struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_FRAME_SYNC:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	default:
		return -EINVAL;
	}

	return 0;
}

static int mtk_camsv_media_link_setup(struct media_entity *entity,
				      const struct media_pad *local,
				      const struct media_pad *remote, u32 flags)
{
	struct mtk_camsv_dev *cam =
		container_of(entity, struct mtk_camsv_dev, subdev.entity);
	u32 pad = local->index;

	/*
	 * The video nodes exposed by the driver have pads indexes
	 * from 0 to MTK_CAMSV_P1_TOTAL_NODES - 1.
	 */
	if (pad < MTK_CAMSV_P1_TOTAL_NODES)
		cam->vdev_nodes[pad].enabled = !!(flags & MEDIA_LNK_FL_ENABLED);

	return 0;
}

static int mtk_camsv_vb2_queue_setup(struct vb2_queue *vq,
				     unsigned int *num_buffers,
				     unsigned int *num_planes,
				     unsigned int sizes[],
				     struct device *alloc_devs[])
{
	struct mtk_camsv_video_device *node = mtk_camsv_vbq_to_vdev(vq);
	unsigned int max_buffer_count = node->desc.max_buf_count;
	const struct v4l2_format *fmt = &node->vdev_fmt;
	struct mtk_camsv_dev *cam = vb2_get_drv_priv(vq);
	struct device *dev = cam->dev;
	struct mtk_camsv_p1_device *p1_dev = dev_get_drvdata(dev);
	unsigned int size;
	unsigned int np_conf;
	unsigned int bpl;
	unsigned int mbus_fmt;
	unsigned int i;

	/* Check the limitation of buffer size */
	if (max_buffer_count)
		*num_buffers = clamp_val(*num_buffers, 1, max_buffer_count);

	size = fmt->fmt.pix_mp.plane_fmt[0].sizeimage;
	/* Add for q.create_bufs with fmt.g_sizeimage(p) / 2 test */

	np_conf = p1_dev->conf->enableFH ? 2 : 1;

	if (*num_planes == 0) {
		*num_planes = np_conf;
		for (i = 0; i < *num_planes; ++i)
			sizes[i] = size;
	} else if (*num_planes != np_conf || sizes[0] < size) {
		return -EINVAL;
	}

	mbus_fmt = fourcc_to_mbus_format(fmt->fmt.pix_mp.pixelformat);
	bpl = calc_bpl(fmt->fmt.pix_mp.width, fmt->fmt.pix_mp.pixelformat);
	mtk_camsv_setup(dev, fmt->fmt.pix_mp.width, fmt->fmt.pix_mp.height, bpl,
			mbus_fmt);

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
	const struct v4l2_format *fmt = &node->vdev_fmt;
	u32 size;
	int i;

	for (i = 0; i < vb->num_planes; i++) {
		size = fmt->fmt.pix_mp.plane_fmt[i].sizeimage;
		if (vb2_plane_size(vb, i) < size) {
			dev_err(cam->dev, "plane size is too small:%lu<%u\n",
				vb2_plane_size(vb, i), size);
			return -EINVAL;
		}
	}

	buf->v4l2_buf.field = V4L2_FIELD_NONE;

	for (i = 0; i < vb->num_planes; i++) {
		size = fmt->fmt.pix_mp.plane_fmt[i].sizeimage;
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

static int mtk_camsv_vb2_start_streaming(struct vb2_queue *vq,
					 unsigned int count)
{
	struct mtk_camsv_dev *cam = vb2_get_drv_priv(vq);
	struct mtk_camsv_video_device *node = mtk_camsv_vbq_to_vdev(vq);
	struct device *dev = cam->dev;
	struct mtk_camsv_p1_device *p1_dev = dev_get_drvdata(dev);
	bool pak_en;
	unsigned int mbus_fmt;
	int ret;

	if (!node->enabled) {
		dev_err(dev, "Node:%d is not enabled\n", node->id);
		ret = -ENOLINK;
		goto fail_ret_buf;
	}

	/* Enable CMOS and VF */
	mbus_fmt = fourcc_to_mbus_format(node->vdev_fmt.fmt.pix_mp.pixelformat);
	pak_en = is_format_pak(mbus_fmt);
	mtk_camsv_cmos_vf_enable(p1_dev, true, pak_en);

	mutex_lock(&cam->op_lock);

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

	if (f->index >= node->desc.num_fmts)
		return -EINVAL;

	/* f->description is filled in v4l_fill_fmtdesc function */
	f->pixelformat = node->desc.fmts[f->index].fmt.pix_mp.pixelformat;
	f->flags = 0;

	return 0;
}

static int mtk_camsv_vidioc_g_fmt(struct file *file, void *fh,
				  struct v4l2_format *f)
{
	struct mtk_camsv_video_device *node = file_to_mtk_camsv_node(file);

	f->fmt = node->vdev_fmt.fmt;

	return 0;
}

static int mtk_camsv_vidioc_try_fmt(struct file *file, void *fh,
				    struct v4l2_format *f)
{
	struct mtk_camsv_dev *cam = video_drvdata(file);
	struct mtk_camsv_video_device *node = file_to_mtk_camsv_node(file);
	struct mtk_camsv_p1_device *p1_dev = dev_get_drvdata(cam->dev);
	const struct v4l2_format *dev_fmt, *tmp;
	unsigned int def_pixfmt;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct v4l2_subdev_pad_config pad_cfg;
	struct v4l2_subdev_format sd_format = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
	};
	int ret;

	/* Validate pixelformat */
	dev_fmt =
		mtk_camsv_dev_find_fmt(&node->desc, f->fmt.pix_mp.pixelformat);
	if (!dev_fmt) {
		tmp = &node->desc.fmts[node->desc.default_fmt_idx];
		def_pixfmt = tmp->fmt.pix_mp.pixelformat;
		f->fmt.pix_mp.pixelformat = def_pixfmt;
	}

	pix_mp->width = clamp_val(pix_mp->width, IMG_MIN_WIDTH, IMG_MAX_WIDTH);
	pix_mp->height =
		clamp_val(pix_mp->height, IMG_MIN_HEIGHT, IMG_MAX_HEIGHT);

	v4l2_fill_mbus_format_mplane(&sd_format.format, pix_mp);
	sd_format.format.code = fourcc_to_mbus_format(pix_mp->pixelformat);

	if (cam->sensor && !cam->is_testmode) {
		ret = v4l2_subdev_call(cam->sensor, pad, set_fmt, &pad_cfg,
				       &sd_format);
		if (ret < 0)
			return ret;
	}

	ret = v4l2_subdev_call(cam->seninf, pad, set_fmt, &pad_cfg, &sd_format);
	if (ret < 0)
		return ret;

	v4l2_fill_pix_format_mplane(pix_mp, &sd_format.format);

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

	struct v4l2_subdev_format sd_format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = 11, /* TODO: use a macro for test pattern */
	};

	int ret;

	if (vb2_is_busy(node->vdev.queue)) {
		dev_dbg(cam->dev, "%s: queue is busy\n", __func__);
		return -EBUSY;
	}

	ret = mtk_camsv_vidioc_try_fmt(file, fh, f);
	if (ret)
		return ret;

	v4l2_fill_mbus_format_mplane(&sd_format.format, &f->fmt.pix_mp);
	sd_format.format.code =
		fourcc_to_mbus_format(f->fmt.pix_mp.pixelformat);

	if (cam->sensor && !cam->is_testmode) {
		ret = v4l2_subdev_call(cam->sensor, pad, set_fmt, NULL,
				       &sd_format);
		if (ret < 0)
			return ret;
	}

	ret = v4l2_subdev_call(cam->seninf, pad, set_fmt, NULL, &sd_format);
	if (ret < 0)
		return ret;

	/* Configure to video device */
	node->vdev_fmt = *f;

	return 0;
}

static int mtk_camsv_vidioc_enum_framesizes(struct file *filp, void *priv,
					    struct v4l2_frmsizeenum *sizes)
{
	struct mtk_camsv_video_device *node = file_to_mtk_camsv_node(filp);
	const struct v4l2_format *dev_fmt;

	dev_fmt = mtk_camsv_dev_find_fmt(&node->desc, sizes->pixel_format);
	if (!dev_fmt || sizes->index)
		return -EINVAL;

	sizes->type = node->desc.frmsizes->type;
	memcpy(&sizes->stepwise, &node->desc.frmsizes->stepwise,
	       sizeof(sizes->stepwise));

	return 0;
}

static const struct v4l2_subdev_core_ops mtk_camsv_subdev_core_ops = {
	.subscribe_event = mtk_camsv_sd_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops mtk_camsv_subdev_video_ops = {
	.s_stream = mtk_camsv_sd_s_stream,
};

static const struct v4l2_subdev_ops mtk_camsv_subdev_ops = {
	.core = &mtk_camsv_subdev_core_ops,
	.video = &mtk_camsv_subdev_video_ops,
};

static const struct media_entity_operations mtk_camsv_media_entity_ops = {
	.link_setup = mtk_camsv_media_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

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

static int camsv_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mtk_camsv_dev *cam =
		container_of(ctrl->handler, struct mtk_camsv_dev, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		if (ctrl->val == TEST_PATTERN_SENINF)
			cam->is_testmode = true;
		else if (ctrl->val == TEST_PATTERN_DISABLED)
			cam->is_testmode = false;
		else
			return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops camsv_ctrl_ops = {
	.s_ctrl = camsv_set_ctrl,
};

static const char *const camsv_test_pattern_menu[] = {
	"No test pattern", "Test pattern from seninf"
};

static int camsv_initialize_controls(struct mtk_camsv_dev *cam)
{
	struct v4l2_ctrl_handler *handler;
	int ret;

	handler = &cam->ctrl_handler;
	ret = v4l2_ctrl_handler_init(handler, 2);
	if (ret)
		return ret;

	v4l2_ctrl_new_std_menu_items(handler, &camsv_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(camsv_test_pattern_menu) - 1, 0,
				     0, camsv_test_pattern_menu);

	cam->is_testmode = false;

	if (handler->error) {
		ret = handler->error;
		dev_err(cam->dev, "Failed to init controls(%d)\n", ret);
		v4l2_ctrl_handler_free(handler);
		return ret;
	}

	cam->subdev.ctrl_handler = handler;

	return 0;
}

static int mtk_camsv_media_register(struct mtk_camsv_dev *cam,
				    struct media_device *media_dev)
{
	/* Reserved MTK_CAMSV_CIO_PAD_SINK + 1 pads to use */
	unsigned int num_pads = MTK_CAMSV_CIO_PAD_SINK + 1;
	struct device *dev = cam->dev;
	int i, ret;

	media_dev->dev = cam->dev;
	strscpy(media_dev->model, dev_driver_string(dev),
		sizeof(media_dev->model));
	snprintf(media_dev->bus_info, sizeof(media_dev->bus_info),
		 "platform:%s", dev_name(dev));
	media_dev->hw_revision = 0;
	media_device_init(media_dev);

	ret = media_device_register(media_dev);
	if (ret) {
		dev_err(dev, "failed to register media device:%d\n", ret);
		return ret;
	}

	/* Initialize subdev pads */
	cam->subdev_pads = devm_kcalloc(dev, num_pads,
					sizeof(*cam->subdev_pads), GFP_KERNEL);
	if (!cam->subdev_pads) {
		dev_err(dev, "failed to allocate subdev_pads\n");
		ret = -ENOMEM;
		goto fail_media_unreg;
	}

	ret = media_entity_pads_init(&cam->subdev.entity, num_pads,
				     cam->subdev_pads);
	if (ret) {
		dev_err(dev, "failed to initialize media pads:%d\n", ret);
		goto fail_media_unreg;
	}

	/* Initialize all pads with MEDIA_PAD_FL_SOURCE */
	for (i = 0; i < num_pads; i++)
		cam->subdev_pads[i].flags = MEDIA_PAD_FL_SOURCE;

	/* Customize the last one pad as CIO sink pad. */
	cam->subdev_pads[MTK_CAMSV_CIO_PAD_SINK].flags = MEDIA_PAD_FL_SINK;

	return 0;

fail_media_unreg:
	media_device_unregister(media_dev);
	media_device_cleanup(media_dev);

	return ret;
}

static int mtk_camsv_video_register_device(struct mtk_camsv_dev *cam,
					   struct mtk_camsv_video_device *node)
{
	struct device *dev = cam->dev;
	struct video_device *vdev = &node->vdev;
	struct vb2_queue *vbq = &node->vbq;
	unsigned int output = V4L2_TYPE_IS_OUTPUT(node->desc.buf_type);
	unsigned int link_flags = node->desc.link_flags;
	int ret;

	/* Initialize mtk_camsv_video_device */
	if (link_flags & MEDIA_LNK_FL_IMMUTABLE)
		node->enabled = true;
	else
		node->enabled = false;
	mtk_camsv_dev_load_default_fmt(cam, &node->desc, &node->vdev_fmt);

	cam->subdev_pads[node->id].flags =
		output ? MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;

	/* Initialize media entities */
	ret = media_entity_pads_init(&vdev->entity, 1, &node->vdev_pad);
	if (ret) {
		dev_err(dev, "failed to initialize media pad:%d\n", ret);
		return ret;
	}
	node->vdev_pad.flags = output ? MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK;

	vbq->type = node->desc.buf_type;
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
		 dev_driver_string(dev), node->desc.name);
	/* set cap/type/ioctl_ops of the video device */
	vdev->device_caps = node->desc.cap | V4L2_CAP_STREAMING;
	vdev->ioctl_ops = node->desc.ioctl_ops;
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
		&cam->subdev.entity, node->id, link_flags);
	else
		ret = media_create_pad_link(&cam->subdev.entity, node->id,
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

static void
mtk_camsv_video_unregister_device(struct mtk_camsv_video_device *node)
{
	video_unregister_device(&node->vdev);
	vb2_queue_release(&node->vbq);
	media_entity_cleanup(&node->vdev.entity);
	mutex_destroy(&node->vdev_lock);
}

static int mtk_camsv_v4l2_register(struct mtk_camsv_dev *cam)
{
	struct device *dev = cam->dev;
	int registered_nodes, i, ret;

	/* Set up media device & pads */
	ret = mtk_camsv_media_register(cam, &cam->media_dev);
	if (ret)
		return ret;

	/* Set up v4l2 device */
	cam->v4l2_dev.mdev = &cam->media_dev;

	ret = v4l2_device_register(dev, &cam->v4l2_dev);
	if (ret) {
		dev_err(dev, "failed to register V4L2 device:%d\n", ret);
		goto fail_media_unreg;
	}

	/* Initialize subdev */
	v4l2_subdev_init(&cam->subdev, &mtk_camsv_subdev_ops);

	ret = camsv_initialize_controls(cam);
	if (ret) {
		dev_err(dev, "Failed to initialize controls\n");
		goto fail_media_unreg;
	}

	cam->subdev.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	cam->subdev.entity.ops = &mtk_camsv_media_entity_ops;
	cam->subdev.flags =
		V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	snprintf(cam->subdev.name, sizeof(cam->subdev.name), "%s",
		 dev_driver_string(dev));
	v4l2_set_subdevdata(&cam->subdev, cam);

	ret = v4l2_device_register_subdev(&cam->v4l2_dev, &cam->subdev);
	if (ret) {
		dev_err(dev, "failed to initialize subdev:%d\n", ret);
		goto fail_clean_media_entiy;
	}

	/* Create video nodes and links */
	for (i = 0; i < MTK_CAMSV_P1_TOTAL_NODES; i++) {
		struct mtk_camsv_video_device *node = &cam->vdev_nodes[i];

		node->id = node->desc.id;
		ret = mtk_camsv_video_register_device(cam, node);
		if (ret) {
			registered_nodes = i - 1;
			goto fail_vdev_unreg;
		}
	}
	vb2_dma_contig_set_max_seg_size(dev, DMA_BIT_MASK(32));

	return 0;

fail_vdev_unreg:
	for (i = registered_nodes; i >= 0; i--)
		mtk_camsv_video_unregister_device(&cam->vdev_nodes[i]);
fail_clean_media_entiy:
	media_entity_cleanup(&cam->subdev.entity);
	v4l2_device_unregister(&cam->v4l2_dev);
fail_media_unreg:
	media_device_unregister(&cam->media_dev);
	media_device_cleanup(&cam->media_dev);

	return ret;
}

static int mtk_camsv_v4l2_unregister(struct mtk_camsv_dev *cam)
{
	int i;

	for (i = 0; i < MTK_CAMSV_P1_TOTAL_NODES; i++)
		mtk_camsv_video_unregister_device(&cam->vdev_nodes[i]);

	vb2_dma_contig_clear_max_seg_size(cam->dev);
	v4l2_device_unregister_subdev(&cam->subdev);
	v4l2_device_unregister(&cam->v4l2_dev);
	media_entity_cleanup(&cam->subdev.entity);
	media_device_unregister(&cam->media_dev);
	media_device_cleanup(&cam->media_dev);

	return 0;
}

static int mtk_camsv_dev_notifier_bound(struct v4l2_async_notifier *notifier,
					struct v4l2_subdev *sd,
					struct v4l2_async_subdev *asd)
{
	struct mtk_camsv_dev *cam =
		container_of(notifier, struct mtk_camsv_dev, notifier);

	if (!(sd->entity.function & MEDIA_ENT_F_VID_IF_BRIDGE)) {
		dev_err(cam->dev, "no MEDIA_ENT_F_VID_IF_BRIDGE function\n");
		return -ENODEV;
	}

	cam->seninf = sd;

	return 0;
}

static void mtk_camsv_dev_notifier_unbind(struct v4l2_async_notifier *notifier,
					  struct v4l2_subdev *sd,
					  struct v4l2_async_subdev *asd)
{
	struct mtk_camsv_dev *cam =
		container_of(notifier, struct mtk_camsv_dev, notifier);

	cam->seninf = NULL;
}

static int mtk_camsv_dev_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct mtk_camsv_dev *cam =
		container_of(notifier, struct mtk_camsv_dev, notifier);
	struct device *dev = cam->dev;
	int ret;

	if (!cam->seninf) {
		dev_err(dev, "No seninf subdev\n");
		return -ENODEV;
	}
	ret = media_create_pad_link(&cam->seninf->entity, MTK_CAMSV_CIO_PAD_SRC,
				    &cam->subdev.entity, MTK_CAMSV_CIO_PAD_SINK,
				    MEDIA_LNK_FL_IMMUTABLE |
					    MEDIA_LNK_FL_ENABLED);
	if (ret) {
		dev_err(dev, "failed to create pad link %s %s err:%d\n",
			cam->seninf->entity.name, cam->subdev.entity.name, ret);
		return ret;
	}
	ret = v4l2_device_register_subdev_nodes(&cam->v4l2_dev);
	if (ret) {
		dev_err(dev, "failed to initialize subdev nodes:%d\n", ret);
		return ret;
	}

	return ret;
}

static const struct v4l2_async_notifier_operations mtk_camsv_v4l2_async_ops = {
	.bound = mtk_camsv_dev_notifier_bound,
	.unbind = mtk_camsv_dev_notifier_unbind,
	.complete = mtk_camsv_dev_notifier_complete,
};

static int mtk_camsv_v4l2_async_register(struct mtk_camsv_dev *cam)
{
	struct device *dev = cam->dev;
	int ret;

	v4l2_async_notifier_init(&cam->notifier);

	ret = v4l2_async_notifier_parse_fwnode_endpoints(dev,
		&cam->notifier, sizeof(struct v4l2_async_subdev), NULL);

	if (ret) {
		dev_err(dev, "failed to parse fwnode endpoints:%d\n", ret);
		return ret;
	}

	cam->notifier.ops = &mtk_camsv_v4l2_async_ops;
	ret = v4l2_async_notifier_register(&cam->v4l2_dev, &cam->notifier);
	if (ret) {
		dev_err(dev, "failed to register async notifier : %d\n", ret);
		v4l2_async_notifier_cleanup(&cam->notifier);
	}

	return ret;
}

static void mtk_camsv_v4l2_async_unregister(struct mtk_camsv_dev *cam)
{
	v4l2_async_notifier_unregister(&cam->notifier);
	v4l2_async_notifier_cleanup(&cam->notifier);
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

static const struct v4l2_format stream_out_fmts[] = {
	/* The 1st entry is the default image format */
	{
		.fmt.pix_mp = {
			.width = 1920,
			.height = 1080,
			.pixelformat = V4L2_PIX_FMT_SRGGB10,
		},
	},

	{
		.fmt.pix_mp = {
			.width = 1920,
			.height = 1080,
			.pixelformat = V4L2_PIX_FMT_SGRBG8,
		},
	},

	{
		.fmt.pix_mp = {
			.width = 1920,
			.height = 1080,
			.pixelformat = V4L2_PIX_FMT_YUYV,
		},
	},
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
		.default_fmt_idx = 0,
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

static void mtk_camsv_dev_queue_setup(struct mtk_camsv_dev *cam)
{
	unsigned int node_idx;
	int i;

	node_idx = 0;
	for (i = 0; i < ARRAY_SIZE(capture_queues); i++)
		cam->vdev_nodes[node_idx++].desc = capture_queues[i];
}

int mtk_camsv_dev_init(struct platform_device *pdev, struct mtk_camsv_dev *cam)
{
	int ret;

	cam->dev = &pdev->dev;
	mtk_camsv_dev_queue_setup(cam);

	mutex_init(&cam->op_lock);

	/* v4l2 sub-device registration */
	ret = mtk_camsv_v4l2_register(cam);
	if (ret) {
		mutex_destroy(&cam->op_lock);
		return ret;
	}

	ret = mtk_camsv_v4l2_async_register(cam);
	if (ret)
		goto fail_v4l2_unreg;

	return 0;

fail_v4l2_unreg:
	mutex_destroy(&cam->op_lock);
	mtk_camsv_v4l2_unregister(cam);
	return ret;
}

void mtk_camsv_dev_cleanup(struct mtk_camsv_dev *cam)
{
	mtk_camsv_v4l2_async_unregister(cam);
	mtk_camsv_v4l2_unregister(cam);
	mutex_destroy(&cam->op_lock);
}
