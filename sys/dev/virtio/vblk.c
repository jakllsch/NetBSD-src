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
#include <sys/device.h>
#include <sys/buf.h>
#include <sys/atomic.h>

#include <dev/pci/pcivar.h>
#include <dev/virtio/virtio_config.h>
#include <dev/virtio/virtio_blk.h>
#include <dev/virtio/virtio_ring.h>
#include <dev/virtio/virtiovar.h>
#include <dev/virtio/vblkvar.h>

#include <uvm/uvm_extern.h>

#define VRING_DESC_F_INDIRECT 4

#define VBLK_NOPEN 32
#define VBLK_NDESC 30

struct vblk_bio {
	struct vring_desc desc[VBLK_NDESC];
	struct virtio_blk_req req;
	void		(*done)(device_t, struct buf *, int);
	device_t	dev;
	struct buf	*bp;
	SIMPLEQ_ENTRY(vblk_bio)	chain;
	bus_dmamap_t    dm_xfer;
	uint16_t	idx;
};

#define VBLK_BIO_SIZE \
	((sizeof(struct vblk_bio) + 0x3ff) & ~0x3ff)

void vblk_attach(struct vblk_softc *);
int vblk_intr(struct vblk_softc *);

static int vblk_print(void *, const char *);
static void vblk_vbb_free(struct vblk_softc *, struct vblk_bio *);
static struct vblk_bio * vblk_vbb_alloc(struct vblk_softc *, bool);
static int vblk_nextdesc(struct vblk_softc *);

void
vblk_attach(struct vblk_softc *sc)
{
	struct vblk_attach_args vaa;
	bus_dma_segment_t seg;
	void *v;
	int ret, rseg;
	uint32_t features;

	virtio_reset(&sc->sc_vc);
	virtio_status_write(&sc->sc_vc, VIRTIO_CONFIG_S_ACKNOWLEDGE);
	virtio_status_write(&sc->sc_vc, VIRTIO_CONFIG_S_DRIVER|VIRTIO_CONFIG_S_ACKNOWLEDGE);

	features = virtio_features_read(&sc->sc_vc);
	features &= VIRTIO_BLK_F_GEOMETRY | VIRTIO_BLK_F_BLK_SIZE | VIRTIO_BLK_F_SECTOR_MAX;
	virtio_features_write(&sc->sc_vc, features);

	virtio_virtq_init(&sc->sc_vc, &sc->sc_rq, VIRTIO_BLK_REQUESTQ);

	virtio_status_write(&sc->sc_vc, VIRTIO_CONFIG_S_DRIVER_OK|VIRTIO_CONFIG_S_DRIVER|VIRTIO_CONFIG_S_ACKNOWLEDGE);

	/* ---- */

	SIMPLEQ_INIT(&sc->sc_vbb_free);
	//SIMPLEQ_INIT(&sc->sc_vbb_queue);

	mutex_init(&sc->sc_mutex, MUTEX_DEFAULT, IPL_BIO);
	cv_init(&sc->sc_vbb_cv, "vblkwait");

	int size;

	size = VBLK_BIO_SIZE * VBLK_NOPEN;

	ret = bus_dmamem_alloc(sc->sc_vc.vc_bdt, size,
		PAGE_SIZE, 0, &seg, 1, &rseg, BUS_DMA_NOWAIT);
	if (ret)
		return;

	ret = bus_dmamem_map(sc->sc_vc.vc_bdt, &seg, rseg, size,
		&v, BUS_DMA_NOWAIT | BUS_DMA_COHERENT);
	if (ret)
		return;

	ret = bus_dmamap_create(sc->sc_vc.vc_bdt, size, 1,
		size, 0,  BUS_DMA_NOWAIT, &sc->sc_vbbdm);
	if (ret)
		return;

	ret = bus_dmamap_load(sc->sc_vc.vc_bdt, sc->sc_vbbdm, v,
		size, NULL, BUS_DMA_NOWAIT);
	if (ret)
		return;

	struct vblk_bio *vbb;
	int i; int error;

	//memset(v, 0, size);

	printf("v %p\n", v);

	sc->sc_vbbs = v;

	vbb = (struct vblk_bio *) sc->sc_vbbs;

	for (i = 0; i < VBLK_NOPEN; i++, vbb = (struct vblk_bio *)((char *)vbb + VBLK_BIO_SIZE)) {
		paddr_t addr;
		addr = sc->sc_vbbdm->dm_segs[0].ds_addr + VBLK_BIO_SIZE * i;
		vbb->idx = i;
		printf("vbb %d %p 0x%016lx\n", i, vbb, addr);
		error = bus_dmamap_create(sc->sc_vc.vc_bdt, MAXPHYS, VBLK_NDESC - 2, MAXPHYS, 0, BUS_DMA_NOWAIT | BUS_DMA_ALLOCNOW, &vbb->dm_xfer);
		if (error != 0)
			panic("failed to create dmamap");

		SIMPLEQ_INSERT_TAIL(&sc->sc_vbb_free, vbb, chain);
	}

	virtio_config_read(&sc->sc_vc, &sc->sc_lbamax, VIRTIO_BLK_CONFIG(capacity), sizeof(sc->sc_lbamax));

	vaa.blksperunit = sc->sc_lbamax;

	vaa.cylinders = vaa.heads = vaa.sectors = 0;
	if (features & VIRTIO_BLK_F_GEOMETRY) {
		virtio_config_read(&sc->sc_vc, &vaa.cylinders, VIRTIO_BLK_CONFIG(cylinders), sizeof(vaa.cylinders));
		virtio_config_read(&sc->sc_vc, &vaa.heads, VIRTIO_BLK_CONFIG(cylinders), sizeof(vaa.heads));
		virtio_config_read(&sc->sc_vc, &vaa.sectors, VIRTIO_BLK_CONFIG(cylinders), sizeof(vaa.sectors));
		aprint_normal_dev(sc->sc_vc.vc_dev, "c %" PRIu16 " h %" PRIu8 " s %" PRIu8 "\n", vaa.cylinders, vaa.heads, vaa.sectors);
	}

	vaa.blksize = 512;
	if (features & VIRTIO_BLK_F_BLK_SIZE) {
#if 0
		virtio_config_read(&sc->sc_vc, &vaa.blksize, VIRTIO_BLK_CONFIG(blk_size), sizeof(vaa.blksize));
#endif
	}

	vaa.maxxfer = MAXPHYS;
	if (features & VIRTIO_BLK_F_SECTOR_MAX) {
		virtio_config_read(&sc->sc_vc, &vaa.maxxfer, VIRTIO_BLK_CONFIG(blk_size), sizeof(vaa.maxxfer));
		vaa.maxxfer = MIN(vaa.maxxfer * vaa.blksize, MAXPHYS);
	}

	aprint_normal_dev(sc->sc_vc.vc_dev, "blocks %" PRIu64 " maxxfer %" PRIu32 "\n", sc->sc_lbamax, vaa.maxxfer);

	sc->sc_child = config_found_ia(sc->sc_vc.vc_dev, "vblk", &vaa, vblk_print);

	//printf("%s ok\n",  device_xname(sc->sc_vc.vc_dev));

	return;
}

int
vblk_intr(struct vblk_softc *sc)
{
	//printf("ring used idx %x\n", sc->sc_vr.used->idx);

	//printf("ring avail idx %x\n", sc->sc_vr.avail->idx);

	bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_rq.vq_bdm, 0, sc->sc_rq.vq_bdm->dm_mapsize, BUS_DMASYNC_POSTWRITE);

	while (sc->sc_rq.vq_last != sc->sc_rq.vq_vr.used->idx) {
		mutex_enter(&sc->sc_mutex);
		struct vblk_bio *vbb;
		void (*done)(device_t, struct buf *, int);
		device_t dev;
		struct buf *bp;
		int status;
		//printf("> %03x >\n", sc->sc_rq.vq_vr.used->ring[sc->sc_rq.vq_last % sc->sc_rq.vq_vr.num].id);
		//printf("%x %x\n", sc->sc_vrlast, sc->sc_vr.num);
		//printf("%016lx - %016lx\n", sc->sc_vr.desc[sc->sc_vr.used->ring[sc->sc_vrlast % sc->sc_vr.num].id].addr, sc->sc_vbbdm->dm_segs[0].ds_addr);


		vbb = (void*)((char *)sc->sc_vbbs + (sc->sc_rq.vq_vr.desc[sc->sc_rq.vq_vr.used->ring[sc->sc_rq.vq_last % sc->sc_rq.vq_vr.num].id].addr - sc->sc_vbbdm->dm_segs[0].ds_addr));
		//printf("vbb %p i %d\n", vbb, vbb->idx);
		//printf("len %x\n", sc->sc_rq.vq_vr.used->ring[sc->sc_rq.vq_last % sc->sc_rq.vq_vr.num].len);
		//vbb->bp->b_resid = vbb->bp->b_bcount - sc->sc_vr.used->ring[sc->sc_vrlast % sc->sc_vr.num].len;
		bus_dmamap_sync(sc->sc_vc.vc_bdt, vbb->dm_xfer, 0, vbb->dm_xfer->dm_mapsize, (vbb->bp->b_flags & B_READ) ? BUS_DMASYNC_POSTREAD : BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->sc_vc.vc_bdt, vbb->dm_xfer);
		atomic_dec_32(&sc->sc_outstanding);

		done = vbb->done;
		dev = vbb->dev;
		bp = vbb->bp;
		status = (vbb->req.status != VIRTIO_BLK_S_OK) ? EIO : 0;

		//memset(vbb, 0, sizeof(vbb));

		int i;
		struct vring_desc *vr;
		i = sc->sc_rq.vq_vr.used->ring[sc->sc_rq.vq_last % sc->sc_rq.vq_vr.num].id;
		while ((sc->sc_rq.vq_vr.desc[i].flags & VRING_DESC_F_NEXT) != 0) {
			vr = &sc->sc_rq.vq_vr.desc[i];
			//printf("%02x %016lx %08x %04hx %04hx\n", i, vr->addr, vr->len, vr->flags, vr->next);
			i = sc->sc_rq.vq_vr.desc[i].next;
			vr->addr = 0;
			vr->len = 0;
			vr->flags = 0;
			vr->next = 0;
		}
		vr = &sc->sc_rq.vq_vr.desc[i];
		//printf("%02x %016lx %08x %04hx %04hx\n", i, vr->addr, vr->len, vr->flags, vr->next);
		vr->addr = 0;
		vr->len = 0;
		vr->flags = 0;
		vr->next = 0;

		sc->sc_rq.vq_last++;

		vblk_vbb_free(sc, vbb);

		mutex_exit(&sc->sc_mutex);

		(*done)(dev, bp, status);
	}

#if 0
	int i;
	for (i = 0; i < 32; i++) {
		printf("%02x", sc->sc_cb[i]);
		if (i % 16 == 15)
			printf("\n");
		else
			printf(" ");
	}
	printf("\n");
#endif

	return 1;
}

static int
vblk_print(void *aux, const char *pnp)
{
	return UNCONF;
}

int
vblk_dobio(device_t dev, struct buf *bp, void (*done)(device_t, struct buf *, int))
{
	struct vblk_softc *sc;
	struct vblk_bio *vbb;
	struct vring_desc *vd;
	int l;
	int d, s;
	uint16_t flags;

	sc = device_private(device_parent(dev));

	mutex_enter(&sc->sc_mutex);

	if ((vbb = vblk_vbb_alloc(sc, true)) == NULL) {
		mutex_exit(&sc->sc_mutex);
		return EAGAIN;
	}


	atomic_inc_32(&sc->sc_outstanding);

	//printf("vblk_dobio vbb %p\n", vbb);

	bus_dmamap_load(sc->sc_vc.vc_bdt, vbb->dm_xfer,
	    bp->b_data, bp->b_bcount, NULL, BUS_DMA_NOWAIT |
	    ((bp->b_flags & B_READ) ? BUS_DMA_READ : BUS_DMA_WRITE));

	bus_dmamap_sync(sc->sc_vc.vc_bdt, vbb->dm_xfer, 0, vbb->dm_xfer->dm_mapsize,
	    (bp->b_flags & B_READ) ? BUS_DMASYNC_PREREAD : BUS_DMASYNC_PREWRITE);

	vbb->done = done;
	vbb->bp = bp;
	vbb->dev = dev;

	vbb->req.type =
	    (bp->b_flags & B_READ) ? VIRTIO_BLK_T_IN : VIRTIO_BLK_T_OUT;
	vbb->req.ioprio = 0x80000000;
	vbb->req.sector = bp->b_rawblkno;


	paddr_t addr;

	addr = sc->sc_vbbdm->dm_segs[0].ds_addr + VBLK_BIO_SIZE * vbb->idx;

	vd = vbb->desc;

	l = d = 0;
	vd[d].addr = addr + offsetof(struct vblk_bio, req) + 0;
	vd[d].len = 16;
	vd[d].flags = VRING_DESC_F_NEXT;
	l = d;
	vd[l].next = ++d;

	flags = (bp->b_flags & B_READ) ? VRING_DESC_F_WRITE : 0;
	flags |= VRING_DESC_F_NEXT;
	for (s = 0; s < vbb->dm_xfer->dm_nsegs; s++) {
		//printf("d %03x s %02x\n", d, s);
		vd[d].addr = vbb->dm_xfer->dm_segs[s].ds_addr;
		vd[d].len = vbb->dm_xfer->dm_segs[s].ds_len;
		vd[d].flags = flags;
		l = d;
		vd[l].next = ++d;
	}

	//printf("d %03x\n", d);

	vd[d].addr = addr + offsetof(struct vblk_bio, req) + offsetof(struct virtio_blk_req, status);
	vd[d].len = 1;
	vd[d].flags = VRING_DESC_F_WRITE;
	vd[d].next = 0;

	d = vblk_nextdesc(sc);
	//printf("< %03x <\n", d);
	sc->sc_rq.vq_vr.desc[d].addr = addr + offsetof(struct vblk_bio, desc);
	sc->sc_rq.vq_vr.desc[d].len = sizeof(struct vring_desc) * VBLK_NDESC;
	sc->sc_rq.vq_vr.desc[d].flags = VRING_DESC_F_INDIRECT;
	sc->sc_rq.vq_vr.desc[d].next = 0;
	sc->sc_rq.vq_vr.avail->ring[sc->sc_rq.vq_vr.avail->idx % sc->sc_rq.vq_vr.num] = d;


	bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_rq.vq_bdm, 0, sc->sc_rq.vq_bdm->dm_mapsize, BUS_DMASYNC_PREWRITE);

	mutex_exit(&sc->sc_mutex);

	sc->sc_rq.vq_vr.avail->idx++;

	bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_rq.vq_bdm, 0, sc->sc_rq.vq_bdm->dm_mapsize, BUS_DMASYNC_PREWRITE);


	if ((sc->sc_rq.vq_vr.used->flags & VRING_USED_F_NO_NOTIFY) == 0) {
		virtio_virtq_notify(&sc->sc_vc, &sc->sc_rq);
	}

	return 0;
}

static struct vblk_bio *
vblk_vbb_alloc(struct vblk_softc *sc, bool nosleep)
{
	struct vblk_bio *vbb;

	KASSERT(mutex_owned(&sc->sc_mutex));

	for(;;) {
		if ((vbb = SIMPLEQ_FIRST(&sc->sc_vbb_free)) != NULL) {
			SIMPLEQ_REMOVE_HEAD(&sc->sc_vbb_free, chain);
			break;
		}
		if (nosleep) {
			vbb = NULL;
			break;
		}
		cv_wait(&sc->sc_vbb_cv, &sc->sc_mutex);
	}

	return vbb;
}

static void
vblk_vbb_free(struct vblk_softc *sc, struct vblk_bio *vbb)
{
	KASSERT(mutex_owned(&sc->sc_mutex));

	/* hmm ordered right? */
	if (SIMPLEQ_EMPTY(&sc->sc_vbb_free))
		cv_signal(&sc->sc_vbb_cv);
	SIMPLEQ_INSERT_TAIL(&sc->sc_vbb_free, vbb, chain);
}

static int
vblk_nextdesc(struct vblk_softc *sc)
{
	struct vring_desc *vr;
	int n;

	for(n = 0; n < sc->sc_rq.vq_vr.num; n++) {
		vr = &sc->sc_rq.vq_vr.desc[n];
		if (vr->flags == 0)
			break;
	}

	KASSERT(n < sc->sc_rq.vq_vr.num);

	return n;
}
