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
#include <sys/conf.h>
#include <sys/tty.h>
#include <sys/proc.h>
#include <sys/kauth.h>
#include <sys/fcntl.h>
#include <sys/kmem.h>

#include <sys/bus.h>

#include <dev/virtio/virtio_config.h>
#include <dev/virtio/virtio_ring.h>
#include <dev/virtio/virtiovar.h>
#include <dev/virtio/virtio_console.h>

#include <dev/virtio/virtiovar.h>
#include <dev/virtio/vconvar.h>

#include <uvm/uvm_extern.h>

#define VCON_BUF_SIZE 4096

static int vcon_addrx(struct vcon_softc *);
static void vconstart(struct tty *);
static int vconparam(struct tty *, struct termios *);

extern struct cfdriver vcon_cd;

dev_type_open(vconopen);
dev_type_close(vconclose);
dev_type_read(vconread);
dev_type_write(vconwrite);
dev_type_ioctl(vconioctl);
dev_type_stop(vconstop);
dev_type_tty(vcontty);
dev_type_poll(vconpoll);

const struct cdevsw vcon_cdevsw = {
	.d_open = vconopen,
	.d_close = vconclose,
	.d_read = vconread,
	.d_write = vconwrite,
	.d_ioctl = vconioctl,
	.d_stop = vconstop,
	.d_tty = vcontty,
	.d_poll = vconpoll,
	.d_mmap = nommap,
	.d_kqfilter = ttykqfilter,
	.d_flag = D_TTY,
};

void
vcon_attach(struct vcon_softc *sc)
{
	struct tty *tp;
	int ret;

	/* reset */
	virtio_reset(&sc->sc_vc);
	virtio_status_write(&sc->sc_vc, VIRTIO_CONFIG_S_ACKNOWLEDGE);
	virtio_status_write(&sc->sc_vc, VIRTIO_CONFIG_S_DRIVER|VIRTIO_CONFIG_S_ACKNOWLEDGE);

	virtio_virtq_init(&sc->sc_vc, &sc->sc_rxvq, VIRTIO_CONSOLE_RECEIVEQ);
	virtio_virtq_init(&sc->sc_vc, &sc->sc_txvq, VIRTIO_CONSOLE_TRANSMITQ);

	/* we're in business */
	virtio_status_write(&sc->sc_vc, VIRTIO_CONFIG_S_DRIVER_OK|VIRTIO_CONFIG_S_DRIVER|VIRTIO_CONFIG_S_ACKNOWLEDGE);

	/* ---- */

	ret = bus_dmamap_create(sc->sc_vc.vc_bdt, MAXPHYS,
	    sc->sc_rxvq.vq_vr.num, 0xffffffff, 0,
	    BUS_DMA_NOWAIT | BUS_DMA_ALLOCNOW, &sc->sc_rxddm);
	if (ret)
		return;

	ret = bus_dmamap_create(sc->sc_vc.vc_bdt, MAXPHYS,
	    sc->sc_txvq.vq_vr.num, 0xffffffff, 0,
	    BUS_DMA_NOWAIT | BUS_DMA_ALLOCNOW, &sc->sc_txddm);
	if (ret)
		return;

	sc->sc_rbuf = kmem_alloc(VCON_BUF_SIZE, KM_SLEEP);  /* XXX bus_dmamem_alloc() instead */

	vcon_addrx(sc);

	tp = tty_alloc();

	tp->t_oproc = vconstart;
	tp->t_param = vconparam;

	sc->sc_tty = tp;

	tty_attach(tp);

	return;
}

int
vcon_intr(struct vcon_softc *sc)
{
	int id;
	uint32_t len;
	bool rxnew;

	rxnew = false;

	printf("rx used idx %x\n", sc->sc_rxvq.vq_vr.used->idx);
	printf("tx used idx %x\n", sc->sc_txvq.vq_vr.used->idx);

	printf("rx avail idx %x\n", sc->sc_rxvq.vq_vr.avail->idx);
	printf("tx avail idx %x\n", sc->sc_txvq.vq_vr.avail->idx);

	while (sc->sc_txvq.vq_last != sc->sc_txvq.vq_vr.used->idx) {
		id = sc->sc_txvq.vq_vr.used->ring[sc->sc_txvq.vq_last % sc->sc_txvq.vq_vr.num].id;
		len = sc->sc_txvq.vq_vr.used->ring[sc->sc_txvq.vq_last % sc->sc_txvq.vq_vr.num].len;
		bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_txddm, 0, sc->sc_txddm->dm_mapsize, BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->sc_vc.vc_bdt, sc->sc_txddm);
		printf("tx i %x l %x\n", id, len);
		CLR(sc->sc_tty->t_state, TS_BUSY);
		ndflush(&sc->sc_tty->t_outq, len);
		sc->sc_txvq.vq_last++;
	}

	while (sc->sc_rxvq.vq_last != sc->sc_rxvq.vq_vr.used->idx) {
		int i;
		rxnew = true;
		id = sc->sc_rxvq.vq_vr.used->ring[sc->sc_rxvq.vq_last % sc->sc_rxvq.vq_vr.num].id;
		len = sc->sc_rxvq.vq_vr.used->ring[sc->sc_rxvq.vq_last % sc->sc_rxvq.vq_vr.num].len;
		bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_rxddm,
		    0, sc->sc_rxddm->dm_mapsize, BUS_DMASYNC_POSTREAD);
		bus_dmamap_unload(sc->sc_vc.vc_bdt, sc->sc_rxddm);
		printf("rx i %x l %x\n", id, len);
		for (i = 0; i < len; i++) {
			(*sc->sc_tty->t_linesw->l_rint)(sc->sc_rbuf[i], sc->sc_tty);
		}

		sc->sc_rxvq.vq_last++;
	}

	if (rxnew)
		vcon_addrx(sc);

	return 1;
}

static int
vcon_addrx(struct vcon_softc *sc)
{
	int s, error;

	error = bus_dmamap_load(sc->sc_vc.vc_bdt, sc->sc_rxddm, sc->sc_rbuf,
	    VCON_BUF_SIZE, NULL, BUS_DMA_WAITOK|BUS_DMA_READ);
	if (error != 0) {
		printf("%s dmamap load failed\n", __func__);
		return error;
	}

	bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_rxddm,
	    0, sc->sc_rxddm->dm_mapsize, BUS_DMASYNC_PREREAD);

	for (s = 0; s < sc->sc_rxddm->dm_nsegs; s++) {
		sc->sc_rxvq.vq_vr.desc[s].addr = sc->sc_rxddm->dm_segs[s].ds_addr;
		sc->sc_rxvq.vq_vr.desc[s].len = sc->sc_rxddm->dm_segs[s].ds_len;
		sc->sc_rxvq.vq_vr.desc[s].flags = VRING_DESC_F_WRITE;
		if (s != (sc->sc_rxddm->dm_nsegs - 1)) {
			sc->sc_rxvq.vq_vr.desc[s].flags |= VRING_DESC_F_NEXT;
			sc->sc_rxvq.vq_vr.desc[s].next = s + 1;
		} else {
			sc->sc_rxvq.vq_vr.desc[s].next = 0;
		}
	}

	sc->sc_rxvq.vq_vr.avail->flags = 0;
	sc->sc_rxvq.vq_vr.avail->ring[sc->sc_rxvq.vq_vr.avail->idx % sc->sc_rxvq.vq_vr.num] = 0;

	bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_rxvq.vq_bdm,
	    0, sc->sc_rxvq.vq_bdm->dm_mapsize, BUS_DMASYNC_PREWRITE);

	sc->sc_rxvq.vq_vr.avail->idx++;

	bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_rxvq.vq_bdm,
	    0, sc->sc_rxvq.vq_bdm->dm_mapsize, BUS_DMASYNC_POSTWRITE);

	if ((sc->sc_rxvq.vq_vr.used->flags & VRING_USED_F_NO_NOTIFY) == 0) {
		virtio_virtq_notify(&sc->sc_vc, &sc->sc_rxvq);
	}

	return 0;
}

static void
vconstart(struct tty *tp)
{
	struct vcon_softc *sc =
		device_lookup_private(&vcon_cd, minor(tp->t_dev));
	int s;

	printf("start\n");

	s = spltty();

#if 1
	if (ISSET(tp->t_state, TS_BUSY | TS_TIMEOUT | TS_TTSTOP)) {
		goto out;
	}

	if (!ttypull(tp))
		goto out;
#endif

	sc->sc_tbc = ndqb(&tp->t_outq, 0);
	sc->sc_tbp = tp->t_outq.c_cf;

	printf("tbc %d\n", sc->sc_tbc);

	if (sc->sc_tbc == 0)
		goto out;

	SET(tp->t_state, TS_BUSY);

	bus_dmamap_load(sc->sc_vc.vc_bdt, sc->sc_txddm,
	    sc->sc_tbp, sc->sc_tbc, NULL, BUS_DMA_NOWAIT | BUS_DMA_WRITE);
	bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_txddm,
	    0, sc->sc_txddm->dm_mapsize, BUS_DMASYNC_PREWRITE);

	sc->sc_txvq.vq_vr.desc[0].addr = sc->sc_txddm->dm_segs[0].ds_addr;
	sc->sc_txvq.vq_vr.desc[0].len = sc->sc_txddm->dm_segs[0].ds_len;
	sc->sc_txvq.vq_vr.desc[0].flags = 0;
	sc->sc_txvq.vq_vr.desc[0].next = 0;

	sc->sc_txvq.vq_vr.avail->flags = 0;
	sc->sc_txvq.vq_vr.avail->ring[sc->sc_txvq.vq_vr.avail->idx % sc->sc_txvq.vq_vr.num] = 0;

	bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_txvq.vq_bdm,
	    0, sc->sc_txvq.vq_bdm->dm_mapsize, BUS_DMASYNC_PREWRITE);

	sc->sc_txvq.vq_vr.avail->idx++;

	bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_txvq.vq_bdm,
	    0, sc->sc_txvq.vq_bdm->dm_mapsize, BUS_DMASYNC_POSTWRITE);

	if ((sc->sc_txvq.vq_vr.used->flags & VRING_USED_F_NO_NOTIFY) == 0) {
		virtio_virtq_notify(&sc->sc_vc, &sc->sc_txvq);
	}

out:
	splx(s);
}

static int
vconparam(struct tty *tp, struct termios *t)
{

	SET(t->c_cflag, CLOCAL);
	CLR(t->c_cflag, HUPCL);

	tp->t_cflag = t->c_cflag;

	return 0;
}

int
vconopen(dev_t dev, int flag, int mode, struct lwp *l)
{
	struct vcon_softc *sc;
	struct tty *tp;
	int s;
	int error;

	sc = device_lookup_private(&vcon_cd, minor(dev));

	if (sc == NULL)
		return ENXIO;

	tp = sc->sc_tty;

#if 1
	if (kauth_authorize_device_tty(l->l_cred, KAUTH_DEVICE_TTY_OPEN, tp))
		return (EBUSY);
#endif

	s = spltty();

	if (!ISSET(tp->t_state, TS_ISOPEN) && tp->t_wopen == 0) {

		tp->t_dev = dev;

		tp->t_ospeed = 0;
		SET(tp->t_cflag, CLOCAL);
		CLR(tp->t_cflag, HUPCL);
		tp->t_iflag = TTYDEF_IFLAG;
		tp->t_oflag = TTYDEF_OFLAG;
		tp->t_lflag = TTYDEF_LFLAG;
		ttychars(tp);
		ttsetwater(tp);
	}

	splx(s);

	//error = ttyopen(tp, , ISSET(flag, O_NONBLOCK));
	error = (*tp->t_linesw->l_open)(dev, tp);

	return error;
}

int
vconclose(dev_t dev, int flag, int mode, struct lwp *l)
{
	struct vcon_softc *sc;
	struct tty *tp;

	sc = device_lookup_private(&vcon_cd, minor(dev));
	tp = sc->sc_tty;

	return (*tp->t_linesw->l_close)(tp, flag);
}

int
vconread(dev_t dev, struct uio *uio, int flag)
{
	struct vcon_softc *sc;
	struct tty *tp;

	sc = device_lookup_private(&vcon_cd, minor(dev));
	tp = sc->sc_tty;

	return (*tp->t_linesw->l_read)(tp, uio, flag);
}

int
vconwrite(dev_t dev, struct uio *uio, int flag)
{
	struct vcon_softc *sc;
	struct tty *tp;

	sc = device_lookup_private(&vcon_cd, minor(dev));
	tp = sc->sc_tty;

	return (*tp->t_linesw->l_write)(tp, uio, flag);
}

int
vconpoll(dev_t dev, int events, struct lwp *l)
{
	struct vcon_softc *sc;
	struct tty *tp;

	sc = device_lookup_private(&vcon_cd, minor(dev));
	tp = sc->sc_tty;

	return (*tp->t_linesw->l_poll)(tp, events, l);
}

struct tty *
vcontty(dev_t dev)
{
	struct vcon_softc *sc;
	struct tty *tp;

	sc = device_lookup_private(&vcon_cd, minor(dev));
	tp = sc->sc_tty;

	return tp;
}

int
vconioctl(dev_t dev, u_long cmd, void *data, int flag, struct lwp *l)
{
	struct vcon_softc *sc;
	struct tty *tp;
	int error;

	sc = device_lookup_private(&vcon_cd, minor(dev));
	tp = sc->sc_tty;

	error = (*tp->t_linesw->l_ioctl)(tp, cmd, data, flag, l);
	if (error != EPASSTHROUGH)
		return error;

	error = ttioctl(tp, cmd, data, flag, l);
	if (error != EPASSTHROUGH)
		return error;

	return EPASSTHROUGH;
}

void
vconstop(struct tty *tp, int flag)
{
	return;
}
