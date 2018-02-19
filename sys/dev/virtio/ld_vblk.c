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
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/buf.h>
#include <sys/bufq.h>
#include <sys/endian.h>
#include <sys/dkio.h>
#include <sys/disk.h>
#include <sys/bus.h>

#include <dev/ldvar.h>

#include <dev/virtio/virtio_ring.h>
#include <dev/virtio/virtiovar.h>
#include <dev/virtio/vblkvar.h>

struct ld_vblk_softc {
	struct ld_softc sc_ld;
};

static int ld_vblk_match(device_t, cfdata_t, void *);
static void ld_vblk_attach(device_t, device_t, void *);
static int ld_vblk_start(struct ld_softc *, struct buf *);
static int ld_vblk_dump(struct ld_softc *, void *, int, int);
static void ld_vblk_done(device_t, struct buf *, int);

CFATTACH_DECL_NEW(ld_vblk, sizeof(struct ld_vblk_softc),
    ld_vblk_match, ld_vblk_attach, NULL, NULL);

static int
ld_vblk_match(device_t parent, cfdata_t match, void *aux)
{
	return 1;
}

static void
ld_vblk_attach(device_t parent, device_t self, void *aux)
{
	struct ld_vblk_softc *sc = device_private(self);
	struct ld_softc *ld = &sc->sc_ld;
	struct vblk_attach_args *vaa = aux;

	aprint_normal("\n");
	aprint_naive("\n");

	//printf("%p %p\n", ld, &ld->sc_flags);

	ld->sc_dv = self;
	ld->sc_flags = LDF_ENABLED;
	ld->sc_secperunit = vaa->blksperunit;
	ld->sc_secsize = vaa->blksize;
	ld->sc_maxxfer = vaa->maxxfer;
	ld->sc_ncylinders = vaa->cylinders;
	ld->sc_nheads  = vaa->heads;
	ld->sc_nsectors = vaa->sectors;
	ld->sc_maxqueuecnt = 32;
	ld->sc_dump = ld_vblk_dump;
	ld->sc_start = ld_vblk_start;

	ldattach(ld, BUFQ_DISK_DEFAULT_STRAT);

	return;
}

static int
ld_vblk_start(struct ld_softc *ld, struct buf *bp)
{
	vblk_dobio(ld->sc_dv, bp, ld_vblk_done);
	return 0;
}

static int
ld_vblk_dump(struct ld_softc *ld, void *data, int blkno, int blkcnt)
{
	return EIO;
}

static void
ld_vblk_done(device_t dev, struct buf *bp, int error)
{
	struct ld_vblk_softc *sc = device_private(dev);

	bp->b_error = error;

	if (error)
		bp->b_resid = bp->b_bcount;
	else
		bp->b_resid = 0;

	lddone(&sc->sc_ld, bp);
}
