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

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */

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
		return ret;
	}

	/* Get active sensor from graph topology */
	cam->sensor = mtk_camsv_cio_get_active_sensor(cam);

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

static const struct v4l2_subdev_video_ops mtk_camsv_subdev_video_ops = {
	.s_stream = mtk_camsv_sd_s_stream,
};

static const struct v4l2_subdev_ops mtk_camsv_subdev_ops = {
	.video = &mtk_camsv_subdev_video_ops,
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
	 * from 0 to MTK_CAMSV_P1_TOTAL_NODES - 1.
	 */
	if (pad < MTK_CAMSV_P1_TOTAL_NODES)
		cam->vdev_nodes[pad].enabled = !!(flags & MEDIA_LNK_FL_ENABLED);

	return 0;
}

static const struct media_entity_operations mtk_camsv_media_entity_ops = {
	.link_setup = mtk_camsv_media_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

/* -----------------------------------------------------------------------------
 * Async Subdev Notifier
 */

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
	int seninf_pad;
	int ret;

	if (!cam->seninf) {
		dev_err(dev, "No seninf subdev\n");
		return -ENODEV;
	}

	seninf_pad = media_get_pad_index(&cam->seninf->entity, false,
					 PAD_SIGNAL_DEFAULT);
	if (seninf_pad < 0)
		return -ENODEV;

	ret = media_create_pad_link(&cam->seninf->entity, seninf_pad,
				    &cam->subdev.entity,
				    MTK_CAMSV_CIO_PAD_SENINF,
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

/* -----------------------------------------------------------------------------
 * Init & Cleanup
 */

static int mtk_camsv_media_register(struct mtk_camsv_dev *cam,
				    struct media_device *media_dev)
{
	struct device *dev = cam->dev;
	unsigned int i;
	int ret;

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
	ret = media_entity_pads_init(&cam->subdev.entity,
				     ARRAY_SIZE(cam->subdev_pads),
				     cam->subdev_pads);
	if (ret) {
		dev_err(dev, "failed to initialize media pads:%d\n", ret);
		goto fail_media_unreg;
	}

	cam->subdev_pads[MTK_CAMSV_CIO_PAD_SENINF].flags = MEDIA_PAD_FL_SINK;
	for (i = 0; i < MTK_CAMSV_P1_TOTAL_NODES; ++i)
		cam->subdev_pads[MTK_CAMSV_CIO_PAD_NODE(i)].flags =
			MEDIA_PAD_FL_SOURCE;

	return 0;

fail_media_unreg:
	media_device_unregister(media_dev);
	media_device_cleanup(media_dev);

	return ret;
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

	cam->subdev.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	cam->subdev.entity.ops = &mtk_camsv_media_entity_ops;
	cam->subdev.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
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

		node->id = node->desc->id;
		ret = mtk_camsv_video_register(cam, node);
		if (ret) {
			registered_nodes = i - 1;
			goto fail_vdev_unreg;
		}
	}

	return 0;

fail_vdev_unreg:
	for (i = registered_nodes; i >= 0; i--)
		mtk_camsv_video_unregister(&cam->vdev_nodes[i]);
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
		mtk_camsv_video_unregister(&cam->vdev_nodes[i]);

	mtk_camsv_video_cleanup_nodes(cam);

	v4l2_device_unregister_subdev(&cam->subdev);
	v4l2_device_unregister(&cam->v4l2_dev);
	media_entity_cleanup(&cam->subdev.entity);
	media_device_unregister(&cam->media_dev);
	media_device_cleanup(&cam->media_dev);

	return 0;
}

int mtk_camsv_dev_init(struct platform_device *pdev, struct mtk_camsv_dev *cam)
{
	int ret;

	cam->dev = &pdev->dev;
	mtk_camsv_video_init_nodes(cam);

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
