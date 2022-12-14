// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 BayLibre
 */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "mtk_camsv.h"
#include "mtk_camsv_regs.h"

#define MTK_CAMSV_AUTOSUSPEND_DELAY_MS 100

static const struct mtk_cam_conf camsv_conf = {
	.tg_sen_mode = 0x00010002U, /* TIME_STP_EN = 1. DBL_DATA_BUS = 1 */
	.module_en = 0x40000001U, /* enable double buffer and TG */
	.dma_special_fun = 0x61000000U, /* enable RDMA insterlace */
	.imgo_con = 0x80000080U, /* DMA FIFO depth and burst */
	.imgo_con2 = 0x00020002U, /* DMA priority */
	.imgo_con3 = 0x00020002U, /* DMA pre-priority */
	.enableFH = false, /* Frame Header disabled */
};

static void fmt_to_sparams(u32 mbus_fmt, struct mtk_cam_sparams *sparams)
{
	switch (mbus_fmt) {
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		sparams->w_factor = 1;
		sparams->module_en_pak = 0x4;
		sparams->fmt_sel = 0x2;
		sparams->pak = 0x12;
		sparams->imgo_stride = 0x01030000;
		break;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
		sparams->w_factor = 1;
		sparams->module_en_pak = 0x4;
		sparams->fmt_sel = 0x1;
		sparams->pak = 0x11;
		sparams->imgo_stride = 0x01030000;
		break;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
		sparams->w_factor = 1;
		sparams->module_en_pak = 0x4;
		sparams->fmt_sel = 0x0;
		sparams->pak = 0x10;
		sparams->imgo_stride = 0x01030000;
		break;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_VYUY8_1X16:
	case MEDIA_BUS_FMT_YUYV8_1X16:
	case MEDIA_BUS_FMT_YVYU8_1X16:
		sparams->w_factor = 2;
		sparams->module_en_pak = 0x8;
		sparams->fmt_sel = 0x3;
		sparams->pak = 0x0;
		sparams->imgo_stride = 0x01010000;
		break;
	default:
		break;
	}
}

static void mtk_camsv_update_buffers_add(struct mtk_cam_dev *cam_dev,
				struct mtk_cam_dev_buffer *buf)
{
	writel(buf->daddr, cam_dev->regs + CAMSV_FBC_IMGO_ENQ_ADDR);
	if (cam_dev->conf->enableFH)
		writel(buf->fhaddr, cam_dev->regs + CAMSV_IMGO_FH_BASE_ADDR);

	writel(0x1U, cam_dev->regs + CAMSV_IMGO_FBC);
}

static void mtk_camsv_cmos_vf_hw_enable(struct mtk_cam_dev *cam_dev, bool pak_en)
{
	unsigned int val;

	val = CAMSV_TG_DP_CLK_EN | CAMSV_DMA_DP_CLK_EN;
	if (pak_en)
		val |= CAMSV_PAK_DP_CLK_EN;
	writel(val, cam_dev->regs + CAMSV_CLK_EN);

	val = readl(cam_dev->regs + CAMSV_TG_VF_CON)
	      | CAMSV_TG_VF_CON_VFDATA_EN;
	writel(val, cam_dev->regs + CAMSV_TG_VF_CON);
}

static void mtk_camsv_cmos_vf_hw_disable(struct mtk_cam_dev *cam_dev, bool pak_en)
{
	unsigned int val;

	val = readl(cam_dev->regs + CAMSV_TG_SEN_MODE)
	      & ~CAMSV_TG_SEN_MODE_CMOS_EN;
	writel(val, cam_dev->regs + CAMSV_TG_SEN_MODE);

	val = readl(cam_dev->regs + CAMSV_TG_VF_CON)
	      & ~CAMSV_TG_VF_CON_VFDATA_EN;
	writel(val, cam_dev->regs + CAMSV_TG_VF_CON);
}

static void mtk_camsv_setup(struct mtk_cam_dev *cam_dev, u32 w, u32 h, u32 bpl,
		     u32 mbus_fmt)
{
	const struct mtk_cam_conf *conf = cam_dev->conf;
	int poll_num = 1000;
	u32 int_en = INT_ST_MASK_CAMSV;
	struct mtk_cam_sparams sparams;

	fmt_to_sparams(mbus_fmt, &sparams);

	mutex_lock(&cam_dev->protect_mutex);

	if (pm_runtime_get_sync(cam_dev->dev) < 0) {
		dev_err(cam_dev->dev, "failed to get pm_runtime\n");
		mutex_unlock(&cam_dev->protect_mutex);
		return;
	}

	writel(conf->tg_sen_mode, cam_dev->regs + CAMSV_TG_SEN_MODE);

	writel(0x1U, cam_dev->regs + CAMSV_TG_TIME_STAMP_CTL);

	writel((w * sparams.w_factor) << 16U,
	       cam_dev->regs + CAMSV_TG_SEN_GRAB_PXL);

	writel(h << 16U, cam_dev->regs + CAMSV_TG_SEN_GRAB_LIN);

	/* YUV_U2S_DIS: disable YUV sensor unsigned to signed */
	writel(0x1000U, cam_dev->regs + CAMSV_TG_PATH_CFG);

	/* Reset CAMSV */
	writel(CAMSV_SW_RST, cam_dev->regs + CAMSV_SW_CTL);
	writel(0x0U, cam_dev->regs + CAMSV_SW_CTL);
	writel(CAMSV_IMGO_RST_TRIG, cam_dev->regs + CAMSV_SW_CTL);

	while (readl(cam_dev->regs + CAMSV_SW_CTL) !=
		       (CAMSV_IMGO_RST_TRIG | CAMSV_IMGO_RST_ST) &&
	       poll_num++ < 1000)
		;

	writel(0x0U, cam_dev->regs + CAMSV_SW_CTL);

	writel(int_en, cam_dev->regs + CAMSV_INT_EN);

	writel(conf->module_en | sparams.module_en_pak,
	       cam_dev->regs + CAMSV_MODULE_EN);
	writel(sparams.fmt_sel, cam_dev->regs + CAMSV_FMT_SEL);
	writel(sparams.pak, cam_dev->regs + CAMSV_PAK);

	/* Reset Frame Header */
	if (conf->enableFH)
		writel(0x0U, cam_dev->regs + CAMSV_DMA_FH_EN);

	writel(readl(cam_dev->regs + CAMSV_DMA_RSV1) & 0x7fffffff,
	       cam_dev->regs + CAMSV_DMA_RSV1);
	writel(0xffffffffU, cam_dev->regs + CAMSV_DMA_RSV6);

	/* DMA performance : CQ ultra LSCI and BPCI. Multiplane ID */
	writel(conf->dma_special_fun, cam_dev->regs + CAMSV_SPECIAL_FUN_EN);

	writel(0x0U, cam_dev->regs + CAMSV_FBC_IMGO_CTL1);
	writel(0x00010000U, cam_dev->regs + CAMSV_FBC_IMGO_CTL1);

	writel(bpl - 1U, cam_dev->regs + CAMSV_IMGO_XSIZE);
	writel(h - 1U, cam_dev->regs + CAMSV_IMGO_YSIZE);

	writel(sparams.imgo_stride | bpl, cam_dev->regs + CAMSV_IMGO_STRIDE);

	writel(conf->imgo_con, cam_dev->regs + CAMSV_IMGO_CON);
	writel(conf->imgo_con2, cam_dev->regs + CAMSV_IMGO_CON2);
	writel(conf->imgo_con3, cam_dev->regs + CAMSV_IMGO_CON3);

	/* CMOS_EN first */
	writel(readl(cam_dev->regs + CAMSV_TG_SEN_MODE) | 0x1U,
	       cam_dev->regs + CAMSV_TG_SEN_MODE);

	/* then CAMSV_FBC_IMGO_CTL1 : FBC_EN=1 , DMA_RING_EN=1 */
	writel(readl(cam_dev->regs + CAMSV_FBC_IMGO_CTL1) | 0x00408000U,
	       cam_dev->regs + CAMSV_FBC_IMGO_CTL1);

	/* finally, CAMSV_MODULE_EN : IMGO_EN */
	writel(readl(cam_dev->regs + CAMSV_MODULE_EN) | 0x00000010U,
	       cam_dev->regs + CAMSV_MODULE_EN);

	/* CAMSV_DMA_FH_EN : FRAME_HEADER_EN_IMGO */
	if (conf->enableFH)
		writel(0x1U, cam_dev->regs + CAMSV_DMA_FH_EN);

	pm_runtime_put_autosuspend(cam_dev->dev);
	mutex_unlock(&cam_dev->protect_mutex);
}

static irqreturn_t isp_irq_camsv(int irq, void *data)
{
	struct mtk_cam_dev *cam_dev = (struct mtk_cam_dev *)data;
	struct mtk_cam_dev_buffer *buf;
	unsigned int irq_status;

	mutex_lock(&cam_dev->protect_mutex);

	irq_status = readl(cam_dev->regs + CAMSV_INT_STATUS);

	if (irq_status & INT_ST_MASK_CAMSV_ERR) {
		dev_err(cam_dev->dev, "irq error 0x%x\n",
			(unsigned int)(irq_status & INT_ST_MASK_CAMSV_ERR));
	}

	/* De-queue frame */
	if (irq_status & CAMSV_IRQ_SW_PASS1_DON) {
		cam_dev->sequence++;

		buf = list_first_entry_or_null(&cam_dev->buf_list,
					       struct mtk_cam_dev_buffer,
					       list);
		if (buf) {
			buf->v4l2_buf.sequence = cam_dev->sequence;
			buf->v4l2_buf.vb2_buf.timestamp = ktime_get_ns();
			vb2_buffer_done(&buf->v4l2_buf.vb2_buf,
					VB2_BUF_STATE_DONE);
			list_del(&buf->list);
		}
	}

	mutex_unlock(&cam_dev->protect_mutex);

	return IRQ_HANDLED;
}

static int mtk_camsv_runtime_suspend(struct device *dev)
{
	struct mtk_cam_dev *cam_dev = dev_get_drvdata(dev);

	clk_disable_unprepare(cam_dev->camsys_camsv);
	clk_disable_unprepare(cam_dev->camsys_camtg_cgpdn);
	clk_disable_unprepare(cam_dev->camsys_cam_cgpdn);

	if (cam_dev->larb_ipu != NULL)
		mtk_smi_larb_put(cam_dev->larb_ipu);

	if (cam_dev->larb_cam != NULL)
		mtk_smi_larb_put(cam_dev->larb_cam);

	return 0;
}

static int mtk_camsv_runtime_resume(struct device *dev)
{
	struct mtk_cam_dev *cam_dev = dev_get_drvdata(dev);
	int ret;

	if (cam_dev->larb_ipu != NULL && cam_dev->larb_cam != NULL) {
		ret = mtk_smi_larb_get(cam_dev->larb_ipu);
		if (ret) {
			dev_err(dev, "failed to get larb index 1, err %d", ret);
			return ret;
		}

		ret = mtk_smi_larb_get(cam_dev->larb_cam);
		if (ret) {
			dev_err(dev, "failed to get larb index 2, err %d", ret);
			mtk_smi_larb_put(cam_dev->larb_ipu);
			return ret;
		}
	}

	clk_prepare_enable(cam_dev->camsys_cam_cgpdn);
	clk_prepare_enable(cam_dev->camsys_camtg_cgpdn);
	clk_prepare_enable(cam_dev->camsys_camsv);

	return 0;
}

static struct mtk_cam_hw_functions mtk_camsv_hw_functions = {
	.mtk_cam_setup = mtk_camsv_setup,
	.mtk_cam_update_buffers_add = mtk_camsv_update_buffers_add,
	.mtk_cam_cmos_vf_hw_enable = mtk_camsv_cmos_vf_hw_enable,
	.mtk_cam_cmos_vf_hw_disable = mtk_camsv_cmos_vf_hw_disable,
};

static int mtk_camsv_probe(struct platform_device *pdev)
{
	struct mtk_cam_dev *cam_dev;
	struct device_node *larb_node1 = NULL;
	struct device_node *larb_node2 = NULL;
	struct platform_device *larb_pdev1 = NULL;
	struct platform_device *larb_pdev2 = NULL;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;

	if (!iommu_present(&platform_bus_type))
		return -EPROBE_DEFER;

	cam_dev = devm_kzalloc(dev, sizeof(*cam_dev), GFP_KERNEL);
	if (!cam_dev)
		return -ENOMEM;

	cam_dev->conf = (struct mtk_cam_conf *)of_device_get_match_data(dev);
	if (!cam_dev->conf)
		return -ENODEV;

	cam_dev->dev = dev;
	dev_set_drvdata(dev, cam_dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cam_dev->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(cam_dev->regs)) {
		dev_err(dev, "failed to map register base\n");
		return PTR_ERR(cam_dev->regs);
	}

	larb_node1 = of_parse_phandle(pdev->dev.of_node, "mediatek,larb", 0);
	larb_node2 = of_parse_phandle(pdev->dev.of_node, "mediatek,larb", 1);
	if (larb_node1 == NULL || larb_node2 == NULL) {
		dev_err(dev, "Missing Mediatek,larb in %s node\n",
			pdev->dev.of_node->full_name);
		return -EINVAL;
	}

	larb_pdev1 = of_find_device_by_node(larb_node1);
	larb_pdev2 = of_find_device_by_node(larb_node2);
	if (larb_pdev1 == NULL || !larb_pdev1->dev.driver ||
	    larb_pdev2 == NULL || !larb_pdev2->dev.driver) {
		of_node_put(larb_node1);
		dev_warn(dev, "Waiting for larb device %s\n",
			larb_node1->full_name);
		return -EPROBE_DEFER;
	}
	of_node_put(larb_node1);
	of_node_put(larb_node2);

	cam_dev->larb_ipu = &larb_pdev1->dev;
	cam_dev->larb_cam = &larb_pdev2->dev;

	cam_dev->camsys_cam_cgpdn = devm_clk_get(dev, "camsys_cam_cgpdn");
	if (IS_ERR(cam_dev->camsys_cam_cgpdn)) {
		dev_err(dev, "failed to get camsys_cam_cgpdn clock\n");
		return PTR_ERR(cam_dev->camsys_cam_cgpdn);
	}
	ret = clk_prepare(cam_dev->camsys_cam_cgpdn);
	if (ret < 0) {
		dev_err(dev, "failed to prepare camsys_cam_cgpdn clock\n");
		cam_dev->camsys_cam_cgpdn = ERR_PTR(-EINVAL);
		return ret;
	}

	cam_dev->camsys_camtg_cgpdn = devm_clk_get(dev, "camsys_camtg_cgpdn");
	if (IS_ERR(cam_dev->camsys_camtg_cgpdn)) {
		dev_err(dev, "failed to get camsys_camtg_cgpdn clock\n");
		ret = PTR_ERR(cam_dev->camsys_camtg_cgpdn);
		goto err_clk;
	}
	ret = clk_prepare(cam_dev->camsys_camtg_cgpdn);
	if (ret < 0) {
		dev_err(dev, "failed to prepare camsys_camtg_cgpdn clock\n");
		cam_dev->camsys_camtg_cgpdn = ERR_PTR(-EINVAL);
		goto err_clk;
	}

	cam_dev->camsys_camsv = devm_clk_get(dev, "camsys_camsv");
	if (IS_ERR(cam_dev->camsys_camsv)) {
		dev_err(dev, "failed to get camsys_camsv clock\n");
		ret = PTR_ERR(cam_dev->camsys_camsv);
		goto err_clk;
	}
	ret = clk_prepare(cam_dev->camsys_camsv);
	if (ret < 0) {
		dev_err(dev, "failed to prepare camsys_camsv clock\n");
		cam_dev->camsys_camsv = ERR_PTR(-EINVAL);
		goto err_clk;
	}

	cam_dev->irq = platform_get_irq(pdev, 0);
	ret = devm_request_threaded_irq(dev, cam_dev->irq, NULL, isp_irq_camsv,
				IRQF_SHARED | IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				dev_name(dev), cam_dev);
	if (ret) {
		dev_err(dev, "failed to request irq=%d\n", cam_dev->irq);
		goto err_clk;
	}

	cam_dev->hw_functions = &mtk_camsv_hw_functions;

	/* initialise protection mutex */
	mutex_init(&cam_dev->protect_mutex);

	/* initialise runtime power management */
	pm_runtime_set_autosuspend_delay(dev, MTK_CAMSV_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "failed to set PM\n");
		goto err_destroy_mutex;
	}

	/* Initialize the v4l2 common part */
	ret = mtk_cam_dev_init(cam_dev);
	if (ret)
		goto err_destroy_mutex;

	return 0;

err_destroy_mutex:
	mutex_destroy(&cam_dev->protect_mutex);

err_clk:
	if (cam_dev->camsys_cam_cgpdn)
		clk_unprepare(cam_dev->camsys_cam_cgpdn);
	if (cam_dev->camsys_camtg_cgpdn)
		clk_unprepare(cam_dev->camsys_camtg_cgpdn);
	if (cam_dev->camsys_camsv)
		clk_unprepare(cam_dev->camsys_camsv);

	return ret;
}

static int mtk_camsv_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_cam_dev *cam_dev = dev_get_drvdata(dev);

	mtk_cam_dev_cleanup(cam_dev);
	pm_runtime_put_autosuspend(dev);
	pm_runtime_disable(dev);
	mutex_destroy(&cam_dev->protect_mutex);

	return 0;
}

static const struct dev_pm_ops mtk_camsv_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(mtk_camsv_runtime_suspend,
				mtk_camsv_runtime_resume, NULL)
};

static const struct of_device_id mtk_camsv_of_ids[] = {
	{
		.compatible = "mediatek,mt8183-camsv",
		.data = &camsv_conf,
	},
	{}
};
MODULE_DEVICE_TABLE(of, mtk_camsv_of_ids);

static struct platform_driver mtk_camsv_driver = {
	.probe = mtk_camsv_probe,
	.remove = mtk_camsv_remove,
	.driver = {
		.name = "mtk-camsv",
		.of_match_table = of_match_ptr(mtk_camsv_of_ids),
		.pm = &mtk_camsv_pm_ops,
	}
};

module_platform_driver(mtk_camsv_driver);

MODULE_DESCRIPTION("Mediatek CAMSV driver");
MODULE_LICENSE("GPL v2");
