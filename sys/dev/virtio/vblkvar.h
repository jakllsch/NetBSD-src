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

#ifndef _DEV_VIRTIO_VBLKVAR_H_
#define _DEV_VIRTIO_VBLKVAR_H_

struct vblk_softc {
	struct virtio_common	sc_vc;
	struct virtio_virtq	sc_rq;
	device_t		sc_child;
	bus_dmamap_t		sc_vbbdm;
	void *			sc_vbbs;
	kmutex_t		sc_mutex;
	kcondvar_t		sc_vbb_cv;
	uint64_t		sc_lbamax;
	uint32_t		sc_outstanding;
	SIMPLEQ_HEAD(, vblk_bio)	sc_vbb_free;
};

struct vblk_attach_args {
	uint64_t blksperunit;
	uint32_t blksize;
	uint32_t maxxfer;
	uint16_t cylinders;
	uint8_t heads;
	uint8_t sectors;
};

void vblk_attach(struct vblk_softc *);
int vblk_intr(struct vblk_softc *);

int vblk_dobio(device_t, struct buf *, void (*)(device_t, struct buf *, int));

#endif /* !_DEV_VIRTIO_VBLKVAR_H_ */
