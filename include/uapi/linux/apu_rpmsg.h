/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Copyright (c) 2020 BayLibre
 */

#ifndef _UAPI_RPMSG_APU_H_
#define _UAPI_RPMSG_APU_H_

#include <linux/ioctl.h>
#include <linux/types.h>

/*
 * Structure containing the APU request from userspace application
 * @cmd: The id of the command to execute on the APU
 * @result: The result of the command executed on the APU
 * @size: The size of the data available in this request
 * @count: The number of shared buffer
 * @data: Contains the data attached with the request if size is greater than
 *        zero, and the files descriptors of shared buffers if count is greater
 *        than zero. Both the data and the shared buffer could be read and write
 *        by the APU.
 */
struct apu_request {
	__u16 id;
	__u16 cmd;
	__u16 result;
	__u16 size_in;
	__u16 size_out;
	__u16 count;
	__u16 reserved;
	__u8 data[0];
};

struct apu_iommu_mmap {
	__u32 fd;
	__u32 da;
};

/* Send synchronous request to an APU */

#define APU_SEND_REQ_IOCTL		_IOW(0xb7, 0x2, struct apu_request)
#define APU_GET_NEXT_AVAILABLE_IOCTL	_IOR(0xb7, 0x3, __u16)
#define APU_GET_RESP			_IOWR(0xb7, 0x4, struct apu_request)
#define APU_IOMMU_MMAP			_IOWR(0xb7, 0x5, struct apu_iommu_mmap)
#define APU_IOMMU_MUNMAP		_IOWR(0xb7, 0x6, __u32)

#endif
