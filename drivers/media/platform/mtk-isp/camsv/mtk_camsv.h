/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 BayLibre
 */

#ifndef __MTK_CAMSV_H__
#define __MTK_CAMSV_H__

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/time64.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>
#include <soc/mediatek/smi.h>

#include "mtk_camsv_regs.h"

#define IMG_MAX_WIDTH			5376
#define IMG_MAX_HEIGHT			4032
#define IMG_DEF_WIDTH			1920
#define IMG_DEF_HEIGHT			1080
#define IMG_MIN_WIDTH			80
#define IMG_MIN_HEIGHT			60

enum TEST_MODE { TEST_PATTERN_DISABLED = 0x0, TEST_PATTERN_SENINF };

/*
 * ID enum value for struct mtk_camsv_dev_node_desc:id
 * or mtk_camsv_video_device:id
 */
enum {
	MTK_CAMSV_P1_MAIN_STREAM_OUT = 0,
	MTK_CAMSV_P1_TOTAL_NODES
};

#define MTK_CAMSV_CIO_PAD_SENINF	0
#define MTK_CAMSV_CIO_PAD_NODE(n)	((n) + 1)
#define MTK_CAMSV_CIO_NUM_PADS		(MTK_CAMSV_P1_TOTAL_NODES + 1)

struct mtk_camsv_dev_buffer {
	struct vb2_v4l2_buffer v4l2_buf;
	struct list_head list;
	dma_addr_t daddr;
	dma_addr_t fhaddr;
};

struct mtk_camsv_sparams {
	unsigned int w_factor;
	unsigned int module_en_pak;
	unsigned int fmt_sel;
	unsigned int pak;
	unsigned int imgo_stride;
};

static inline struct mtk_camsv_dev_buffer *
to_mtk_camsv_dev_buffer(struct vb2_buffer *buf)
{
	return container_of(buf, struct mtk_camsv_dev_buffer, v4l2_buf.vb2_buf);
}

/*
 * struct mtk_camsv_dev_node_desc - MTK camera device node descriptor
 *
 * @id: id of the node
 * @name: name of the node
 * @cap: supported V4L2 capabilities
 * @buf_type: supported V4L2 buffer type
 * @link_flags: default media link flags
 * @def_width: the default format width
 * @def_height: the default format height
 * @num_fmts: the number of supported node formats
 * @max_buf_count: maximum VB2 buffer count
 * @ioctl_ops:  mapped to v4l2_ioctl_ops
 * @fmts: supported format
 * @frmsizes: supported V4L2 frame size number
 *
 */
struct mtk_camsv_dev_node_desc {
	u8 id;
	const char *name;
	u32 cap;
	u32 buf_type;
	u32 link_flags;
	u32 def_width;
	u32 def_height;
	u8 num_fmts;
	u8 max_buf_count;
	const struct v4l2_ioctl_ops *ioctl_ops;
	const u32 *fmts;
	const struct v4l2_frmsizeenum *frmsizes;
};

/*
 * struct mtk_camsv_video_device - Mediatek video device structure
 *
 * @id: Id for index of mtk_camsv_dev:vdev_nodes array
 * @enabled: Indicate the video device is enabled or not
 * @desc: The node description of video device
 * @vdev_fmt: The V4L2 format of video device
 * @vdev_pad: The media pad graph object of video device
 * @vbq: A videobuf queue of video device
 * @vdev: The video device instance
 * @vdev_lock: Serializes vb2 queue and video device operations
 *
 */
struct mtk_camsv_video_device {
	unsigned int id;
	unsigned int enabled;
	const struct mtk_camsv_dev_node_desc *desc;
	struct v4l2_format vdev_fmt;
	struct media_pad vdev_pad;
	struct vb2_queue vbq;
	struct video_device vdev;
	/* Serializes vb2 queue and video device operations */
	struct mutex vdev_lock;
};

/*
 * struct mtk_camsv_dev - Mediatek camera device structure.
 *
 * @dev: Pointer to device.
 * @pipeline: Media pipeline information.
 * @media_dev: Media device instance.
 * @subdev: The V4L2 sub-device instance.
 * @v4l2_dev: The V4L2 device driver instance.
 * @notifier: The v4l2_device notifier data.
 * @subdev_pads: Media pads of this sub-device.
 * @formats: Media bus format for all pads.
 * @vdev_nodes: The array list of mtk_camsv_video_device nodes.
 * @seninf: Pointer to the seninf sub-device.
 * @streaming: Indicate the overall streaming status is on or off.
 * @stream_count: Number of streaming video nodes
 * @op_lock: Serializes driver's VB2 callback operations.
 *
 */
struct mtk_camsv_dev {
	struct device *dev;
	struct media_pipeline pipeline;
	struct media_device media_dev;
	struct v4l2_subdev subdev;
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct media_pad subdev_pads[MTK_CAMSV_CIO_NUM_PADS];
	struct v4l2_mbus_framefmt formats[MTK_CAMSV_CIO_NUM_PADS];
	struct mtk_camsv_video_device vdev_nodes[MTK_CAMSV_P1_TOTAL_NODES];
	struct v4l2_subdev *seninf;
	unsigned int streaming;
	unsigned int stream_count;

	struct mutex op_lock;

	struct list_head buf_list;
};

static inline struct mtk_camsv_dev *to_mtk_camsv_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mtk_camsv_dev, subdev);
}

struct mtk_camsv_conf {
	unsigned int tg_sen_mode;
	unsigned int module_en;
	unsigned int pak;
	unsigned int dma_special_fun;
	unsigned int imgo_con;
	unsigned int imgo_con2;
	unsigned int imgo_con3;
	bool enableFH;
};

struct mtk_camsv_p1_device {
	struct device *dev;
	struct mtk_camsv_dev camsv_dev;
	void __iomem *regs;
	struct clk *camsys_cam_cgpdn;
	struct clk *camsys_camtg_cgpdn;
	struct clk *camsys_camsv0;
	struct device *larb_ipu;
	struct device *larb_cam;
	unsigned int irq;
	const struct mtk_camsv_conf *conf;

	struct mutex protect_mutex;
};

void mtk_camsv_setup(struct device *dev, u32 width, u32 height, u32 bpl,
		     u32 mbus_fmt);

int mtk_camsv_dev_init(struct platform_device *pdev,
		       struct mtk_camsv_dev *camsv_dev);

void mtk_camsv_dev_cleanup(struct mtk_camsv_dev *camsv_dev);

int mtk_camsv_video_register(struct mtk_camsv_dev *cam,
			     struct mtk_camsv_video_device *node);
void mtk_camsv_video_unregister(struct mtk_camsv_video_device *node);
void mtk_camsv_video_init_nodes(struct mtk_camsv_dev *cam);
void mtk_camsv_video_cleanup_nodes(struct mtk_camsv_dev *cam);

#endif /* __MTK_CAMSV_H__ */
