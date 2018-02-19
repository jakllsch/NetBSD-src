/* $NetBSD$ */

/*
 * Copyright (c) 2010 Jonathan A. Kollasch
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _DEV_VIRTIO_VIRTIO_BLK_H_
#define _DEV_VIRTIO_VIRTIO_BLK_H_

#define VIRTIO_ID_BLOCK		2

#define VIRTIO_BLK_REQUESTQ	0

#define VIRTIO_BLK_F_BARRIER	__BIT(0)
#define VIRTIO_BLK_F_SIZE_MAX	__BIT(1)
#define VIRTIO_BLK_F_SEG_MAX	__BIT(2)
#define VIRTIO_BLK_F_GEOMETRY	__BIT(4)
#define VIRTIO_BLK_F_RO		__BIT(5)
#define VIRTIO_BLK_F_BLK_SIZE	__BIT(6)
#define VIRTIO_BLK_F_SECTOR_MAX	__BIT(10)

struct virtio_blk_config {
	uint64_t capacity;
	uint32_t size_max;
	uint32_t seg_max;
	/* struct virtio_blk_geometry { */
		uint16_t cylinders;
		uint8_t heads;
		uint8_t sectors;
	/* } geometry; */
	uint32_t blk_size;
	uint32_t sectors_max;
} __packed;

#define VIRTIO_BLK_CONFIG(x) (offsetof(struct virtio_blk_config, x))

struct virtio_blk_req {
#define VIRTIO_BLK_T_IN 0
#define VIRTIO_BLK_T_OUT 1
#define VIRTIO_BLK_T_BARRIER 0x80000000
	uint32_t type;
	uint32_t ioprio;
	uint64_t sector;
	/* char data[][512]; */
#define VIRTIO_BLK_S_OK 0
#define VIRTIO_BLK_S_IOERR 1
	uint8_t  status;
	uint8_t  _pad[15];
} __packed;

#endif /* !_DEV_VIRTIO_VIRTIO_VBLK_H_ */
