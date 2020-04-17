/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright 2020 BayLibre SAS
 */

#ifndef __APU_RPMSG_H__
#define __APU_RPMSG_H__

/*
 * Firmware request, must be aligned with the one defined in firmware.
 * @id: Request id, used in the case of reply, to find the pending request
 * @cmd: The command id to execute in the firmware
 * @result: The result of the command executed on the firmware
 * @size: The size of the data available in this request
 * @count: The number of shared buffer
 * @data: Contains the data attached with the request if size is greater than
 *        zero, and the addresses of shared buffers if count is greater than
 *        zero. Both the data and the shared buffer could be read and write
 *        by the APU.
 */
struct  apu_dev_request {
	u16 id;
	u16 cmd;
	u16 result;
	u16 size_in;
	u16 size_out;
	u16 count;
	u8 data[0];
} __packed;

#define APU_RPMSG_SERVICE_MT8183 "rpmsg-mt8183-apu0"
#define APU_CTRL_SRC 1
#define APU_CTRL_DST 1

/* Vendor specific resource table entry */
#define RSC_VENDOR_IOVA 128

/*
 * Firmware IOVA resource table entry
 * Define a range of virtual device address that could mapped using the IOMMU.
 * @da: Start virtual device address
 * @len: Length of the virtual device address
 * @name: name of the resource
 */
struct fw_rsc_iova {
	u32 da;
	u32 len;
	u32 reserved;
	u8 name[32];
} __packed;

#endif /* __APU_RPMSG_H__ */
