/*
 * Copyright (c) 2016 MediaTek Inc.
 * Author: PC Chen <pc.chen@mediatek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/slab.h>

#include "mtk_vcodec_intr.h"
#include "vdec_vcu_if.h"
#include "vdec_drv_base.h"
#include "mtk_vcodec_dec.h"
#include "vdec_drv_if.h"

#define DEC_MAX_FB_NUM				32U

/**
 * struct vdec_fb - vdec decode frame buffer information
 * @vdec_fb_va  : virtual address of struct vdec_fb
 * @y_fb_dma    : dma address of Y frame buffer
 * @c_fb_dma    : dma address of C frame buffer
 * @poc         : picture order count of frame buffer
 * @reserved    : for 8 bytes alignment
 */
struct dec_fb {
	uint64_t vdec_fb_va;
	uint64_t y_fb_dma;
	uint64_t c_fb_dma;
	int32_t poc;
	uint32_t reserved;
};

/**
 * struct ring_fb_list - ring frame buffer list
 * @fb_list   : frame buffer arrary
 * @read_idx  : read index
 * @write_idx : write index
 * @count     : buffer count in list
 */
struct ring_fb_list {
	struct dec_fb fb_list[DEC_MAX_FB_NUM];
	unsigned int read_idx;
	unsigned int write_idx;
	unsigned int count;
	unsigned int reserved;
};

/**
 * struct vdec_dec_info - decode information
 * @dpb_sz		: decoding picture buffer size
 * @vdec_changed_info  : some changed flags
 * @bs_dma		: Input bit-stream buffer dma address
 * @bs_fd               : Input bit-stream buffer dmabuf fd
 * @fb_dma		: Y frame buffer dma address
 * @fb_fd             : Y frame buffer dmabuf fd
 * @vdec_fb_va		: VDEC frame buffer struct virtual address
 * @fb_num_planes	: frame buffer plane count
 * @reserved		: reserved variable for 64bit align
 */
struct vdec_dec_info {
	uint32_t dpb_sz;
	uint32_t vdec_changed_info;
	uint64_t bs_dma;
	uint64_t bs_fd;
	uint64_t fb_dma[VIDEO_MAX_PLANES];
	uint64_t fb_fd[VIDEO_MAX_PLANES];
	uint64_t vdec_fb_va;
	uint32_t fb_num_planes;
	uint32_t index;
};

/**
 * struct vdec_vsi - shared memory for decode information exchange
 *                        between VCU and Host.
 *                        The memory is allocated by VCU and mapping to Host
 *                        in vcu_dec_init()
 * @ppl_buf_dma : HW working buffer ppl dma address
 * @mv_buf_dma  : HW working buffer mv dma address
 * @list_free   : free frame buffer ring list
 * @list_disp   : display frame buffer ring list
 * @dec		: decode information
 * @pic		: picture information
 * @crop        : crop information
 */
struct vdec_vsi {
	struct ring_fb_list list_free;
	struct ring_fb_list list_disp;
	struct vdec_dec_info dec;
	struct vdec_pic_info pic;
	struct mtk_color_desc color_desc;
	struct v4l2_rect crop;
	char crc_path[256];
	char golden_path[256];
};

/**
 * struct vdec_inst - decoder instance
 * @num_nalu : how many nalus be decoded
 * @ctx      : point to mtk_vcodec_ctx
 * @vcu      : VCU instance
 * @vsi      : VCU shared information
 */
struct vdec_inst {
	unsigned int num_nalu;
	struct mtk_vcodec_ctx *ctx;
	struct vdec_vcu_inst vcu;
	struct vdec_vsi *vsi;
};

static void put_fb_to_free(struct vdec_inst *inst, struct vdec_fb *fb)
{
	struct ring_fb_list *list;

	if (fb != NULL) {
		list = &inst->vsi->list_free;
		if (list->count == DEC_MAX_FB_NUM) {
			mtk_vcodec_err(inst, "[FB] put fb free_list full");
			return;
		}

		mtk_vcodec_debug(inst,
				"[FB] put fb into free_list @(%p, %llx)",
				fb->fb_base[0].va,
				(u64)fb->fb_base[1].dma_addr);

		list->fb_list[list->write_idx].vdec_fb_va = (u64)(uintptr_t)fb;
		list->write_idx = (list->write_idx == DEC_MAX_FB_NUM - 1U) ?
				  0U : list->write_idx + 1U;
		list->count++;
	}
}

static void get_pic_info(struct vdec_inst *inst,
			 struct vdec_pic_info *pic)
{
	unsigned int i = 0;
	uint32_t num_planes = 0;

	pic->pic_w = inst->vsi->pic.pic_w;
	pic->pic_h = inst->vsi->pic.pic_h;
	pic->buf_w = inst->vsi->pic.buf_w;
	pic->buf_h = inst->vsi->pic.buf_h;
	pic->bitdepth = inst->vsi->pic.bitdepth;
	pic->ufo_mode = inst->vsi->pic.ufo_mode;
	num_planes = inst->vsi->dec.fb_num_planes;

	for (i = 0; i < num_planes; i++)
		pic->fb_sz[i] = inst->vsi->pic.fb_sz[i];

	mtk_vcodec_debug(inst, "pic(%d, %d), buf(%d, %d), bitdepth = %d\n",
			 pic->pic_w, pic->pic_h, pic->buf_w, pic->buf_h,
			 pic->bitdepth);
	mtk_vcodec_debug(inst, "Y/C(%d, %d)", pic->fb_sz[0], pic->fb_sz[1]);
}

static void get_crop_info(struct vdec_inst *inst, struct v4l2_crop *cr)
{
	cr->c.left	= inst->vsi->crop.left;
	cr->c.top	= inst->vsi->crop.top;
	cr->c.width	= inst->vsi->crop.width;
	cr->c.height	= inst->vsi->crop.height;
	mtk_vcodec_debug(inst, "l=%d, t=%d, w=%d, h=%d",
			 cr->c.left, cr->c.top, cr->c.width, cr->c.height);
}

static void get_dpb_size(struct vdec_inst *inst, unsigned int *dpb_sz)
{
	*dpb_sz = inst->vsi->dec.dpb_sz;
	mtk_vcodec_debug(inst, "sz=%d", *dpb_sz);
}

static int vdec_init(struct mtk_vcodec_ctx *ctx, unsigned long *h_vdec)
{
	struct vdec_inst *inst = NULL;
	int err;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst || !ctx)
		return -ENOMEM;

	inst->ctx = ctx;

	switch (ctx->q_data[MTK_Q_DATA_SRC].fmt->fourcc) {
	case V4L2_PIX_FMT_H264:
		inst->vcu.id = IPI_VDEC_H264;
	break;
	case V4L2_PIX_FMT_HEVC:
		inst->vcu.id = IPI_VCU_VDEC_H265;
	break;
	case V4L2_PIX_FMT_VP8:
		inst->vcu.id = IPI_VDEC_VP8;
	break;
	case V4L2_PIX_FMT_VP9:
		inst->vcu.id = IPI_VDEC_VP9;
	break;
	case V4L2_PIX_FMT_MPEG4:
		inst->vcu.id = IPI_VCU_VDEC_MPEG4;
	break;
	case V4L2_PIX_FMT_MPEG1:
	case V4L2_PIX_FMT_MPEG2:
		inst->vcu.id = IPI_VCU_VDEC_MPEG12;
	break;
	default:
		mtk_vcodec_err(inst, "vdec_init no fourcc");
	break;
	}

	inst->vcu.dev = vpu_get_plat_device(ctx->dev->plat_dev);
	inst->vcu.ctx = ctx;
	inst->vcu.handler = vcu_dec_ipi_handler;

	err = vcu_dec_init(&inst->vcu);
	if (err != 0) {
		mtk_vcodec_err(inst, "vdec_init err=%d", err);
		goto error_free_inst;
	}

	inst->vsi = (struct vdec_vsi *)inst->vcu.vsi;

	mtk_vcodec_debug(inst, "Decoder Instance >> %p", inst);

	*h_vdec = (unsigned long)inst;
	return 0;

error_free_inst:
	kfree(inst);

	return err;
}

static void vdec_deinit(unsigned long h_vdec)
{
	struct vdec_inst *inst = (struct vdec_inst *)h_vdec;

	mtk_vcodec_debug_enter(inst);

	vcu_dec_deinit(&inst->vcu);

	kfree(inst);
}

static int vdec_decode(unsigned long h_vdec, struct mtk_vcodec_mem *bs,
			       struct vdec_fb *fb, unsigned int *src_chg)
{
	struct vdec_inst *inst = (struct vdec_inst *)h_vdec;
	struct vdec_vcu_inst *vcu = &inst->vcu;
	int ret = 0;
	unsigned int data[2];
	uint64_t vdec_fb_va;
	uint64_t fb_dma[VIDEO_MAX_PLANES] = { 0 };
	uint32_t num_planes;
	unsigned int i = 0;

	num_planes = fb ? inst->vsi->dec.fb_num_planes : 0U;

	for (i = 0; i < num_planes; i++)
		fb_dma[i] = fb ? (u64)fb->fb_base[i].dma_addr : 0UL;

	vdec_fb_va = (u64)(uintptr_t)fb;

	mtk_vcodec_debug(inst, "+ [%d] FB y_dma=%llx c_dma=%llx va=%p num_planes %d",
			 inst->num_nalu, fb_dma[0], fb_dma[1], fb, num_planes);

	/* bs NULL means flush decoder */
	if (bs == NULL)
		return vcu_dec_reset(vcu);

	mtk_vcodec_debug(inst, "+ BS dma=0x%llx dmabuf=%p",
			 (uint64_t)bs->dma_addr, bs->dmabuf);

	inst->vsi->dec.bs_dma = (uint64_t)bs->dma_addr;

	for (i = 0; i < num_planes; i++)
		inst->vsi->dec.fb_dma[i] = fb_dma[i];

	inst->vsi->dec.vdec_fb_va = vdec_fb_va;
	inst->vsi->dec.bs_fd = (uint64_t)get_mapped_fd(bs->dmabuf);

	if (fb != NULL) {
		inst->vsi->dec.index = fb->index;
		for (i = 0; i < num_planes; i++) {
			inst->vsi->dec.fb_fd[i] =
				(uint64_t)get_mapped_fd(fb->fb_base[i].dmabuf);
		}
	} else {
		inst->vsi->dec.index = 0xFF;
		for (i = 0; i < num_planes; i++)
			inst->vsi->dec.fb_fd[i] = 0;
	}

	mtk_vcodec_debug(inst, "+ FB y_fd=%llx c_fd=%llx BS fd=%llx",
			 inst->vsi->dec.fb_fd[0], inst->vsi->dec.fb_fd[1],
			 inst->vsi->dec.bs_fd);

	data[0] = (unsigned int)bs->size;
	data[1] = (unsigned int)bs->length;
	ret = vcu_dec_start(vcu, data, 2);

	*src_chg = inst->vsi->dec.vdec_changed_info;

	if ((*src_chg & VDEC_NEED_SEQ_HEADER) != 0U)
		mtk_vcodec_err(inst, "- need first seq header -");
	else if ((*src_chg & VDEC_RES_CHANGE) != 0U)
		mtk_vcodec_debug(inst, "- resolution changed -");
	else if ((*src_chg & VDEC_HW_NOT_SUPPORT) != 0U)
		mtk_vcodec_err(inst, "- unsupported -");
	/*ack timeout means vpud has crashed*/
	if (ret == -EIO) {
		mtk_vcodec_err(inst, "- IPI msg ack timeout  -");
		*src_chg = *src_chg | VDEC_HW_NOT_SUPPORT;
	}

	if (ret < 0 || ((*src_chg & VDEC_HW_NOT_SUPPORT) != 0U)
		|| ((*src_chg & VDEC_NEED_SEQ_HEADER) != 0U))
		goto err_free_fb_out;

	mtk_vcodec_debug(inst, "\n - NALU[%d] -\n", inst->num_nalu);
	inst->num_nalu++;
	return ret;

err_free_fb_out:
	put_fb_to_free(inst, fb);
	mtk_vcodec_err(inst, "\n - NALU[%d] err=%d -\n", inst->num_nalu, ret);
	return ret;
}

static void vdec_get_fb(struct vdec_inst *inst,
			     struct ring_fb_list *list,
			     bool disp_list, struct vdec_fb **out_fb)
{
	unsigned long vdec_fb_va;
	struct vdec_fb *fb;

	if (list->count == 0) {
		mtk_vcodec_debug(inst, "[FB] there is no %s fb",
				 disp_list ? "disp" : "free");
		*out_fb = NULL;
		return;
	}

	vdec_fb_va = (unsigned long)list->fb_list[list->read_idx].vdec_fb_va;
	fb = (struct vdec_fb *)vdec_fb_va;
	if (disp_list)
		fb->status |= FB_ST_DISPLAY;
	else
		fb->status |= FB_ST_FREE;

	*out_fb = fb;
	mtk_vcodec_debug(inst, "[FB] get %s fb st=%d poc=%d %llx",
			 disp_list ? "disp" : "free",
			 fb->status, list->fb_list[list->read_idx].poc,
			 list->fb_list[list->read_idx].vdec_fb_va);

	list->read_idx = (list->read_idx == DEC_MAX_FB_NUM - 1U) ?
			 0U : list->read_idx + 1U;
	list->count--;
}

static int vdec_get_param(unsigned long h_vdec,
			       enum vdec_get_param_type type, void *out)
{
	struct vdec_inst *inst = (struct vdec_inst *)h_vdec;
	int ret = 0;

	switch (type) {
	case GET_PARAM_DISP_FRAME_BUFFER:
		vdec_get_fb(inst, &inst->vsi->list_disp, true, out);
		break;

	case GET_PARAM_FREE_FRAME_BUFFER:
		vdec_get_fb(inst, &inst->vsi->list_free, false, out);
		break;

	case GET_PARAM_PIC_INFO:
		get_pic_info(inst, out);
		break;

	case GET_PARAM_DPB_SIZE:
		get_dpb_size(inst, out);
		break;

	case GET_PARAM_CROP_INFO:
		get_crop_info(inst, out);
		break;

	default:
		mtk_vcodec_err(inst, "invalid get parameter type=%d", type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int vdec_set_param(unsigned long h_vdec,
			       enum vdec_set_param_type type, void *in)
{
	struct vdec_inst *inst = (struct vdec_inst *)h_vdec;
	uint64_t size;
	int ret = 0;

	switch (type) {
	case SET_PARAM_FRAME_SIZE:
		vcu_dec_set_param(&inst->vcu, (unsigned int)type, in, 2U);
		break;
	case SET_PARAM_DECODE_MODE:
		vcu_dec_set_param(&inst->vcu, (unsigned int)type, in, 1U);
		break;
	case SET_PARAM_SET_FIXED_MAX_OUTPUT_BUFFER:
	case SET_PARAM_UFO_MODE:
		break;
	case SET_PARAM_CRC_PATH:
		size = strlen((char *)*(uintptr_t *)in);
		memcpy(inst->vsi->crc_path, (void *)*(uintptr_t *)in, size);
		break;
	case SET_PARAM_GOLDEN_PATH:
		size = strlen((char *)*(uintptr_t *)in);
		memcpy(inst->vsi->golden_path, (void *)*(uintptr_t *)in, size);
		break;
	case SET_PARAM_FB_NUM_PLANES:
		inst->vsi->dec.fb_num_planes = *(unsigned int *)in;
		break;
	default:
		mtk_vcodec_err(inst, "invalid set parameter type=%d\n", type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static struct vdec_common_if vdec_if = {
	vdec_init,
	vdec_decode,
	vdec_get_param,
	vdec_set_param,
	vdec_deinit,
};

struct vdec_common_if *get_dec_common_if(void)
{
	return &vdec_if;
}
