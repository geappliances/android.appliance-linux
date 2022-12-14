/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __SOC_MEDIATEK_SCPSYS_EXT_H
#define __SOC_MEDIATEK_SCPSYS_EXT_H

#define MAX_STEPS	4

#define BUS_PROT(_type, _set_ofs, _clr_ofs,			\
		_en_ofs, _sta_ofs, _mask, _clr_ack_mask) {	\
		.type = _type,					\
		.set_ofs = _set_ofs,				\
		.clr_ofs = _clr_ofs,				\
		.en_ofs = _en_ofs,				\
		.sta_ofs = _sta_ofs,				\
		.mask = _mask,					\
		.clr_ack_mask = _clr_ack_mask,			\
	}

enum regmap_type {
	INVALID_TYPE = 0,
	IFR_TYPE,
	SMI_TYPE,
};

struct bus_prot {
	enum regmap_type type;
	u32 set_ofs;
	u32 clr_ofs;
	u32 en_ofs;
	u32 sta_ofs;
	u32 mask;
	u32 clr_ack_mask;
};

int mtk_scpsys_ext_set_bus_protection(const struct bus_prot *bp_table,
	struct regmap *infracfg, struct regmap *smi_common);
int mtk_scpsys_ext_clear_bus_protection(const struct bus_prot *bp_table,
	struct regmap *infracfg, struct regmap *smi_common);

#endif /* __SOC_MEDIATEK_SCPSYS_EXT_H */
