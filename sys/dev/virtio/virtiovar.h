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

#ifndef _DEV_VIRTIO_VIRTIOVAR_H_
#define _DEV_VIRTIO_VIRTIOVAR_H_

struct virtio_common {
	device_t		vc_dev;
	bus_size_t		vc_ios;
	bus_space_tag_t		vc_iot;
	bus_space_handle_t	vc_ioh;
	bus_dma_tag_t		vc_bdt;
};

struct virtio_virtq {
	struct vring	vq_vr;
	bus_size_t	vq_sz;
	void		*vq_rva;
	bus_dma_tag_t   *vq_bdt;
	bus_dmamap_t	vq_bdm;
	uint16_t	vq_qsz;
	uint16_t	vq_last;
	uint16_t	vq_queue;
};

void virtio_reset(struct virtio_common *);
uint32_t virtio_features_read(struct virtio_common *);
void virtio_features_write(struct virtio_common *, uint32_t);
int  virtio_status_read(struct virtio_common *);
void virtio_status_write(struct virtio_common *, int);
void virtio_config_read(struct virtio_common *, void *, int, size_t);
void virtio_config_write(struct virtio_common *, const void *, int, size_t);
int  virtio_virtq_init(struct virtio_common *, struct virtio_virtq *, uint16_t);
void virtio_virtq_notify(struct virtio_common *, struct virtio_virtq *);

#endif /* !_DEV_VIRTIO_VIRTIOVAR_H_ */
