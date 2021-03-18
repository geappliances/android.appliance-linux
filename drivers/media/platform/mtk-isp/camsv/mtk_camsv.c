// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 BayLibre
 */

#include <linux/platform_device.h>

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "mtk_camsv.h"

static inline struct mtk_camsv_dev *to_mtk_camsv_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mtk_camsv_dev, subdev);
}

static const u32 mtk_camsv_mbus_formats[] = {
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_UYVY8_1X16,
	MEDIA_BUS_FMT_VYUY8_1X16,
	MEDIA_BUS_FMT_YUYV8_1X16,
	MEDIA_BUS_FMT_YVYU8_1X16,
};

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */

static int mtk_camsv_cio_stream_on(struct mtk_camsv_dev *cam)
{
	struct device *dev = cam->dev;
	struct media_pad *seninf_pad;
	int ret;

	if (!cam->seninf) {
		seninf_pad = media_entity_remote_pad(
				&cam->subdev_pads[MTK_CAMSV_CIO_PAD_SENINF]);
		if (!seninf_pad) {
			dev_err(dev, "%s: No SENINF connected\n", __func__);
			return -ENOLINK;
		}
		cam->seninf = media_entity_to_v4l2_subdev(seninf_pad->entity);
	}

	/* Seninf must stream on first */
	ret = v4l2_subdev_call(cam->seninf, video, s_stream, 1);
	if (ret) {
		dev_err(dev, "failed to stream on %s:%d\n",
			cam->seninf->entity.name, ret);
		return ret;
	}

	cam->streaming = true;

	return 0;
}

static int mtk_camsv_cio_stream_off(struct mtk_camsv_dev *cam)
{
	struct device *dev = cam->dev;
	int ret;

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
	struct mtk_camsv_dev *cam = to_mtk_camsv_dev(sd);

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

static struct v4l2_mbus_framefmt *
mtk_camsv_get_pad_format(struct mtk_camsv_dev *cam,
			 struct v4l2_subdev_pad_config *cfg,
			 unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&cam->subdev, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &cam->formats[pad];
	default:
		return NULL;
	}
}

static int mtk_camsv_init_cfg(struct v4l2_subdev *sd,
			      struct v4l2_subdev_pad_config *cfg)
{
	static const struct v4l2_mbus_framefmt def_format = {
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.width = IMG_DEF_WIDTH,
		.height = IMG_DEF_HEIGHT,
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.xfer_func = V4L2_XFER_FUNC_DEFAULT,
		.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
		.quantization = V4L2_QUANTIZATION_DEFAULT,
	};

	struct mtk_camsv_dev *cam = to_mtk_camsv_dev(sd);
	u32 which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	struct v4l2_mbus_framefmt *format;
	unsigned int i;

	for (i = 0; i < sd->entity.num_pads; i++) {
		format = mtk_camsv_get_pad_format(cam, cfg, i, which);
		*format = def_format;
	}

	return 0;
}

static int mtk_camsv_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(mtk_camsv_mbus_formats))
		return -EINVAL;

	code->code = mtk_camsv_mbus_formats[code->index];

	return 0;
}

static int mtk_camsv_get_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
{
	struct mtk_camsv_dev *cam = to_mtk_camsv_dev(sd);

	fmt->format = *mtk_camsv_get_pad_format(cam, cfg, fmt->pad, fmt->which);

	return 0;
}

static int mtk_camsv_set_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *fmt)
{
	struct mtk_camsv_dev *cam = to_mtk_camsv_dev(sd);
	struct v4l2_mbus_framefmt *format;
	unsigned int i;

	/*
	 * We only support pass-through mode, the format on source pads can't
	 * be modified.
	 */
	if (fmt->pad != MTK_CAMSV_CIO_PAD_SENINF)
		return mtk_camsv_get_fmt(sd, cfg, fmt);

	for (i = 0; i < ARRAY_SIZE(mtk_camsv_mbus_formats); ++i) {
		if (mtk_camsv_mbus_formats[i] == fmt->format.code)
			break;
	}

	if (i == ARRAY_SIZE(mtk_camsv_mbus_formats))
		fmt->format.code = mtk_camsv_mbus_formats[0];

	format = mtk_camsv_get_pad_format(cam, cfg, fmt->pad, fmt->which);
	format->width = fmt->format.width;
	format->height = fmt->format.height;
	format->code = fmt->format.code;

	fmt->format = *format;

	/* Propagate the format to the source pads. */
	for (i = 0; i < MTK_CAMSV_TOTAL_NODES; ++i) {
		unsigned int pad = MTK_CAMSV_CIO_PAD_NODE(i);

		format = mtk_camsv_get_pad_format(cam, cfg, pad, fmt->which);
		format->width = fmt->format.width;
		format->height = fmt->format.height;
		format->code = fmt->format.code;
	}

	return 0;
}

static int mtk_camsv_subdev_registered(struct v4l2_subdev *sd)
{
	struct mtk_camsv_dev *cam = to_mtk_camsv_dev(sd);
	int i, ret;

	/* Create video nodes and links */
	for (i = 0; i < MTK_CAMSV_TOTAL_NODES; ++i) {
		struct mtk_camsv_video_device *node = &cam->vdev_nodes[i];

		node->id = node->desc->id;
		ret = mtk_camsv_video_register(cam, node);
		if (ret)
			goto fail_vdev_unreg;
	}

	return 0;

fail_vdev_unreg:
	for (i--; i >= 0; --i)
		mtk_camsv_video_unregister(&cam->vdev_nodes[i]);

	return ret;
}

static const struct v4l2_subdev_video_ops mtk_camsv_subdev_video_ops = {
	.s_stream = mtk_camsv_sd_s_stream,
};

static const struct v4l2_subdev_pad_ops mtk_camsv_subdev_pad_ops = {
	.init_cfg = mtk_camsv_init_cfg,
	.enum_mbus_code = mtk_camsv_enum_mbus_code,
	.set_fmt = mtk_camsv_set_fmt,
	.get_fmt = mtk_camsv_get_fmt,
	.link_validate = v4l2_subdev_link_validate_default,
};

static const struct v4l2_subdev_ops mtk_camsv_subdev_ops = {
	.video = &mtk_camsv_subdev_video_ops,
	.pad = &mtk_camsv_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops mtk_camsv_internal_ops = {
	.registered = mtk_camsv_subdev_registered,
};

/* -----------------------------------------------------------------------------
 * Media Entity Operations
 */

static int mtk_camsv_media_link_setup(struct media_entity *entity,
				      const struct media_pad *local,
				      const struct media_pad *remote, u32 flags)
{
	struct mtk_camsv_dev *cam =
		container_of(entity, struct mtk_camsv_dev, subdev.entity);
	u32 pad = local->index;

	/*
	 * The video nodes exposed by the driver have pads indexes
	 * from 0 to MTK_CAMSV_TOTAL_NODES - 1.
	 */
	if (pad < MTK_CAMSV_TOTAL_NODES)
		cam->vdev_nodes[pad].enabled = !!(flags & MEDIA_LNK_FL_ENABLED);

	return 0;
}

static const struct media_entity_operations mtk_camsv_media_entity_ops = {
	.link_setup = mtk_camsv_media_link_setup,
	.link_validate = v4l2_subdev_link_validate,
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
};

/* -----------------------------------------------------------------------------
 * Init & Cleanup
 */

static int mtk_camsv_v4l2_register(struct mtk_camsv_dev *cam)
{
	struct device *dev = cam->dev;
	int ret;
	unsigned int i;

	/* Initialize subdev pads */
	ret = media_entity_pads_init(&cam->subdev.entity,
				     ARRAY_SIZE(cam->subdev_pads),
				     cam->subdev_pads);
	if (ret) {
		dev_err(dev, "failed to initialize media pads:%d\n", ret);
		return ret;
	}

	cam->subdev_pads[MTK_CAMSV_CIO_PAD_SENINF].flags = MEDIA_PAD_FL_SINK;
	for (i = 0; i < MTK_CAMSV_TOTAL_NODES; ++i)
		cam->subdev_pads[MTK_CAMSV_CIO_PAD_NODE(i)].flags =
			MEDIA_PAD_FL_SOURCE;

	/* Initialize subdev */
	v4l2_subdev_init(&cam->subdev, &mtk_camsv_subdev_ops);

	cam->subdev.dev = dev;
	cam->subdev.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	cam->subdev.entity.ops = &mtk_camsv_media_entity_ops;
	cam->subdev.internal_ops = &mtk_camsv_internal_ops;
	cam->subdev.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	strscpy(cam->subdev.name, dev_name(dev), V4L2_SUBDEV_NAME_SIZE);
	v4l2_set_subdevdata(&cam->subdev, cam);

	mtk_camsv_init_cfg(&cam->subdev, NULL);

	ret = v4l2_async_register_subdev(&cam->subdev);
	if (ret) {
		dev_err(dev, "failed to initialize subdev:%d\n", ret);
		goto fail_clean_media_entiy;
	}

	return 0;

fail_clean_media_entiy:
	media_entity_cleanup(&cam->subdev.entity);

	return ret;
}

static void mtk_camsv_v4l2_unregister(struct mtk_camsv_dev *cam)
{
	int i;

	for (i = 0; i < MTK_CAMSV_TOTAL_NODES; i++)
		mtk_camsv_video_unregister(&cam->vdev_nodes[i]);

	vb2_dma_contig_clear_max_seg_size(cam->dev);

	media_entity_cleanup(&cam->subdev.entity);
	v4l2_async_unregister_subdev(&cam->subdev);
}

int mtk_camsv_dev_init(struct mtk_camsv_dev *camsv_dev)
{
	int ret;

	mtk_camsv_video_init_nodes(camsv_dev);

	mutex_init(&camsv_dev->op_lock);

	/* v4l2 sub-device registration */
	ret = mtk_camsv_v4l2_register(camsv_dev);
	if (ret) {
		mutex_destroy(&camsv_dev->op_lock);
		return ret;
	}

	return ret;
}

void mtk_camsv_dev_cleanup(struct mtk_camsv_dev *cam)
{
	mtk_camsv_v4l2_unregister(cam);
	mutex_destroy(&cam->op_lock);
}
