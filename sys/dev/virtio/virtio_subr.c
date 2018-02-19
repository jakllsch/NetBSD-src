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

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <uvm/uvm_extern.h>
#include <dev/virtio/virtio_ring.h>
#include <dev/virtio/virtiovar.h>
#include <dev/pci/virtio/virtio_pci.h>

void
virtio_reset(struct virtio_common *vc)
{
	virtio_status_write(vc, 0);
}

int
virtio_status_read(struct virtio_common *vc)
{
	return bus_space_read_1(vc->vc_iot, vc->vc_ioh, VIRTIO_PCI_STATUS);
}

void
virtio_status_write(struct virtio_common *vc, int value)
{
	bus_space_write_1(vc->vc_iot, vc->vc_ioh, VIRTIO_PCI_STATUS, value);
}

uint32_t
virtio_features_read(struct virtio_common *vc)
{
	return bus_space_read_stream_4(vc->vc_iot, vc->vc_ioh, VIRTIO_PCI_HOST_FEATURES);
}

void
virtio_features_write(struct virtio_common *vc, uint32_t feat)
{
	bus_space_write_stream_4(vc->vc_iot, vc->vc_ioh, VIRTIO_PCI_GUEST_FEATURES, feat);
}

void
virtio_config_read(struct virtio_common *vc, void *buf, int off, size_t len)
{
	off += VIRTIO_PCI_CONFIG;

	bus_space_read_region_stream_1(vc->vc_iot, vc->vc_ioh, off, buf, len);
}

void
virtio_config_write(struct virtio_common *vc, const void *buf, int off, size_t len)
{
	off += VIRTIO_PCI_CONFIG;

	bus_space_write_region_stream_1(vc->vc_iot, vc->vc_ioh, off, buf, len);
}

int
virtio_virtq_init(struct virtio_common *vc, struct virtio_virtq *vq, uint16_t q)
{
	bus_dma_segment_t seg;
	int rseg;
	int err;

	vq->vq_queue = q;

	/* select virtq */
	bus_space_write_2(vc->vc_iot, vc->vc_ioh, VIRTIO_PCI_QUEUE_SEL, q);
	vq->vq_qsz = bus_space_read_2(vc->vc_iot, vc->vc_ioh, VIRTIO_PCI_QUEUE_NUM);
	vq->vq_sz = vring_size(vq->vq_qsz, VIRTIO_PCI_VRING_ALIGN);

	err = bus_dmamem_alloc(vc->vc_bdt, vq->vq_sz, PAGE_SIZE, 0, &seg, 1, &rseg, BUS_DMA_NOWAIT);
	if (err) {
		aprint_error_dev(vc->vc_dev, "unable to allocate virtq %d, error %d\n", q, err);
		return err;
	}

	err = bus_dmamem_map(vc->vc_bdt, &seg, rseg, vq->vq_sz, &vq->vq_rva, BUS_DMA_NOWAIT|BUS_DMA_COHERENT);
	if (err) {
		aprint_error_dev(vc->vc_dev, "unable to map virtq %d, error %d\n", q, err);
		bus_dmamem_free(vc->vc_bdt, &seg, rseg);
		return err;
	}

	err = bus_dmamap_create(vc->vc_bdt, vq->vq_sz, 1, vq->vq_sz, 0, BUS_DMA_NOWAIT, &vq->vq_bdm);
	if (err) {
		aprint_error_dev(vc->vc_dev, "unable to create virtq %d map, error %d\n", q, err);
		bus_dmamem_unmap(vc->vc_bdt, vq->vq_rva, vq->vq_sz);
		bus_dmamem_free(vc->vc_bdt, &seg, rseg);
		return err;
	}

	err = bus_dmamap_load(vc->vc_bdt, vq->vq_bdm, vq->vq_rva, vq->vq_sz, NULL, BUS_DMA_NOWAIT);
	if (err) {
		aprint_error_dev(vc->vc_dev, "unable to load virtq %d map, error %d\n", q, err);
		bus_dmamap_destroy(vc->vc_bdt, vq->vq_bdm);
		bus_dmamem_unmap(vc->vc_bdt, vq->vq_rva, vq->vq_sz);
		bus_dmamem_free(vc->vc_bdt, &seg, rseg);
		return err;
	}

	aprint_debug_dev(vc->vc_dev, "vring q %x n %x v %p p %lx\n", q, vq->vq_qsz, vq->vq_rva, vq->vq_bdm->dm_segs[0].ds_addr);

	memset(vq->vq_rva, 0, vq->vq_sz);
	vring_init(&vq->vq_vr, vq->vq_qsz, vq->vq_rva, VIRTIO_PCI_VRING_ALIGN);

	bus_space_write_4(vc->vc_iot, vc->vc_ioh, VIRTIO_PCI_QUEUE_PFN,
		vq->vq_bdm->dm_segs[0].ds_addr >> VIRTIO_PCI_QUEUE_ADDR_SHIFT);

	return 0;
}

void
virtio_virtq_notify(struct virtio_common *vc, struct virtio_virtq *vq)
{
	bus_space_write_stream_2(vc->vc_iot, vc->vc_ioh, VIRTIO_PCI_QUEUE_NOTIFY, vq->vq_queue);
}
