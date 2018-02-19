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

#include <sys/bus.h>

#include <net/if.h>
#include <net/if_ether.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>

#include <net/bpf.h>

#include <dev/virtio/virtio_config.h>
#include <dev/virtio/virtio_ring.h>
#include <dev/virtio/virtio_net.h>

#include <dev/virtio/virtiovar.h>

#include <dev/virtio/if_vnetvar.h>

static void vnet_rxintr(struct vnet_softc *);
static void vnet_txintr(struct vnet_softc *);
static void vnet_cxintr(struct vnet_softc *);

static int vnet_add_rxbuf(struct vnet_softc *, int);

static int vnet_ioctl(struct ifnet *, u_long, void *);
static void vnet_start(struct ifnet *);
static void vnet_watchdog(struct ifnet *);
static int vnet_init(struct ifnet *);
static void vnet_stop(struct ifnet *, int);

/*
 * Descriptor area:
 * tx vrds
 * rx vrds
 * tx vnhs
 * page align
 * rx vnhs
 * page align
 */

#define VNET_VRDSZ (sizeof(struct vring_desc))
#define VNET_VNHSZ ((sizeof(struct virtio_net_hdr) + 15) & ~15)

#define VNET_TXVRDOFF (0)
#define VNET_TXVRDPA(sc, x) ((sc)->sc_ddm->dm_segs[0].ds_addr + VNET_TXVRDOFF + VNET_VRDSZ * (x))
#define VNET_TXVRDVA(sc, x) ((struct vring_desc *)((char *)(sc)->sc_dva + VNET_TXVRDOFF + VNET_VRDSZ * (x)))
#define VNET_TXVRDSZ (VNET_VRDSZ * VNET_NTXSEGS)

#define VNET_RXVRDOFF (VNET_TXVRDOFF + VNET_TXVRDSZ)
#define VNET_RXVRDPA(sc, x) ((sc)->sc_ddm->dm_segs[0].ds_addr + VNET_RXVRDOFF + VNET_VRDSZ * (x))
#define VNET_RXVRDVA(sc, x) ((struct vring_desc *)((char *)(sc)->sc_dva + VNET_RXVRDOFF + VNET_VRDSZ * (x)))
#define VNET_RXDVRDSZ (VNET_VRDSZ * VNET_NRXDESC * 2)

#define VNET_TXVNHOFF (VNET_RXVRDOFF + VNET_RXDVRDSZ)
#define VNET_TXVNHPA(sc, x) ((sc)->sc_ddm->dm_segs[0].ds_addr + VNET_TXVNHOFF + VNET_VNHSZ * (x))
#define VNET_TXVNHVA(sc, x) ((struct virtio_net_hdr *)((char *)(sc)->sc_dva + VNET_TXVNHOFF * (x)))
#define VNET_TXVNHSZ (VNET_VNHSZ * VNET_NTXDESC)

//#define VNET_RXVNHOFF ((VNET_TXVNHOFF + VNET_TXVNHSZ + (PAGE_SIZE-1)) & ~(PAGE_SIZE-1))
#define VNET_RXVNHOFF round_page(VNET_TXVNHOFF + VNET_TXVNHSZ)
#define VNET_RXVNHPA(sc, x) ((sc)->sc_ddm->dm_segs[0].ds_addr + VNET_RXVNHOFF + VNET_VNHSZ * (x))
#define VNET_RXVNHVA(sc, x) ((struct virtio_net_hdr *)((char *)(sc)->sc_dva + VNET_RXVNHOFF * (x)))
#define VNET_RXVNHSZ (VNET_VNHSZ * VNET_NRXDESC)

//#define VNET_DSIZE ((VNET_RXVNHOFF + VNET_RXVNHSZ + (PAGE_SIZE-1)) & ~(PAGE_SIZE-1))
#define VNET_DSIZE round_page(VNET_RXVNHOFF + VNET_RXVNHSZ)

void
vnet_attach(struct vnet_softc *sc)
{
	struct ifnet *ifp;
	bus_dma_segment_t seg;
	int rseg;
	int error;
	int i;
	uint8_t eaddr[ETHER_ADDR_LEN];

	virtio_reset(&sc->sc_vc);
	virtio_status_write(&sc->sc_vc, VIRTIO_CONFIG_S_ACKNOWLEDGE);
	virtio_status_write(&sc->sc_vc, VIRTIO_CONFIG_S_DRIVER|VIRTIO_CONFIG_S_ACKNOWLEDGE);

	virtio_virtq_init(&sc->sc_vc, &sc->sc_rq, VIRTIO_NET_RECEIVEQ);
	virtio_virtq_init(&sc->sc_vc, &sc->sc_tq, VIRTIO_NET_TRANSMITQ);
	virtio_virtq_init(&sc->sc_vc, &sc->sc_cq, VIRTIO_NET_CONTROLQ);

	virtio_status_write(&sc->sc_vc, VIRTIO_CONFIG_S_DRIVER_OK|VIRTIO_CONFIG_S_DRIVER|VIRTIO_CONFIG_S_ACKNOWLEDGE);

	virtio_config_read(&sc->sc_vc, eaddr, VIRTIO_NET_CONFIG_MAC, ETHER_ADDR_LEN);

	size_t size;

	size = VNET_DSIZE;

	error = bus_dmamem_alloc(sc->sc_vc.vc_bdt, size, PAGE_SIZE, 0, &seg, 1, &rseg, BUS_DMA_NOWAIT);
	if (error)
		return;
	
	error = bus_dmamem_map(sc->sc_vc.vc_bdt, &seg, rseg, size,
		&sc->sc_dva, BUS_DMA_NOWAIT | BUS_DMA_COHERENT);
	if (error)
		return;

	error = bus_dmamap_create(sc->sc_vc.vc_bdt, size, 1, size, 0,
		BUS_DMA_NOWAIT, &sc->sc_ddm);
	if (error)
		return;

	error = bus_dmamap_load(sc->sc_vc.vc_bdt, sc->sc_ddm, sc->sc_dva,
		size, NULL, BUS_DMA_NOWAIT);
	if (error)
		return;

	printf("dva %p %016lx\n", sc->sc_dva, sc->sc_ddm->dm_segs[0].ds_addr);

	callout_init(&sc->sc_timeout, 0);

	for (i = 0; i < VNET_NRXDESC; i++) {
		if ((error = bus_dmamap_create(sc->sc_vc.vc_bdt, MCLBYTES, 1,
		    MCLBYTES, 0, 0, &sc->sc_rxs[i].rx_dm)) != 0) {
			aprint_error_dev(sc->sc_vc.vc_dev, "XXX\n");
		}
		sc->sc_rxs[i].rx_mb = NULL;
	}

	for (i = 0; i < VNET_NTXDESC; i++) {
		if ((error = bus_dmamap_create(sc->sc_vc.vc_bdt, VNET_MAXTXDMA,
			VNET_NTXSEGS, VNET_MAXTXDMA, 0, 0,
			&sc->sc_txs[i].tx_dm)) != 0) {
			aprint_error_dev(sc->sc_vc.vc_dev, "XXX\n");
		}
	}

	ifp = &sc->sc_ec.ec_if;

	ifp->if_softc = sc;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST | IFF_ALLMULTI;

	ifp->if_start = vnet_start;
	ifp->if_ioctl = vnet_ioctl;
        ifp->if_watchdog = vnet_watchdog;
        ifp->if_init = vnet_init;
        ifp->if_stop = vnet_stop;
	strncpy(ifp->if_xname, device_xname(sc->sc_vc.vc_dev), IFNAMSIZ);
	IFQ_SET_MAXLEN(&ifp->if_snd, max(VNET_IFQUEUELEN, IFQ_MAXLEN));
	IFQ_SET_READY(&ifp->if_snd);

	if_attach(ifp);
	ether_ifattach(ifp, eaddr);

	return;
}

int
vnet_intr(struct vnet_softc *sc)
{
	vnet_rxintr(sc);
	vnet_txintr(sc);
	vnet_cxintr(sc);

	return 1;
}

static void
vnet_rxintr(struct vnet_softc *sc)
{
	struct ifnet *ifp;
	struct vnet_rxsoft *rxs;
	struct mbuf *m;
	int i;
	int len;

	bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_rq.vq_bdm, 0, sc->sc_rq.vq_bdm->dm_mapsize, BUS_DMASYNC_POSTWRITE);

	ifp = &sc->sc_ec.ec_if;

	while (sc->sc_rq.vq_last != sc->sc_rq.vq_vr.used->idx) {
		i = sc->sc_rq.vq_vr.used->ring[sc->sc_rq.vq_last & (sc->sc_rq.vq_vr.num-1)].id;
		len = sc->sc_rq.vq_vr.used->ring[sc->sc_rq.vq_last & (sc->sc_rq.vq_vr.num-1)].len - sizeof(struct virtio_net_hdr);
		//printf("rxi %02x %x\n", i, len);

		i >>= 1;

		rxs = &sc->sc_rxs[i];

		bus_dmamap_sync(sc->sc_vc.vc_bdt, rxs->rx_dm, 0,
		    rxs->rx_dm->dm_mapsize, BUS_DMASYNC_POSTREAD);

		m = rxs->rx_mb;

		m->m_len = len;

		m_set_rcvif(m, ifp);
		m->m_pkthdr.len = len;

		if_percpuq_enqueue(ifp->if_percpuq, m);

		vnet_add_rxbuf(sc, i);
		sc->sc_rq.vq_last++;
	}
}

static void
vnet_txintr(struct vnet_softc *sc)
{
	//struct ifnet *ifp;
	struct vnet_txsoft *txs;
	int i;

	bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_tq.vq_bdm, 0, sc->sc_tq.vq_bdm->dm_mapsize, BUS_DMASYNC_POSTWRITE);

	//ifp = &sc->sc_ec.ec_if;

	i = sc->sc_txsdirty;
 
	while (sc->sc_tq.vq_last != sc->sc_tq.vq_vr.used->idx) {
#if 0
		int id;
		int len;
		id = sc->sc_tq.vq_vr.used->ring[sc->sc_tq.vq_last & (sc->sc_tq.vq_vr.num-1)].id;
		len = sc->sc_tq.vq_vr.used->ring[sc->sc_tq.vq_last & (sc->sc_tq.vq_vr.num-1)].len;
		//printf("txi t %02x h %02x l %x\n", i, id, len);
#endif

		txs = &sc->sc_txs[i];

		bus_dmamap_sync(sc->sc_vc.vc_bdt, txs->tx_dm, 0, txs->tx_dm->dm_mapsize, BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->sc_vc.vc_bdt, txs->tx_dm);
		m_freem(txs->tx_mb);
		txs->tx_mb = NULL;
		txs->tx_first = -1;
		txs->tx_ndesc = 0;

		sc->sc_tq.vq_last++;
		sc->sc_txsfree++;
		i = VNET_NEXTTX(i);
	}

	sc->sc_txsdirty = i;
}

static void
vnet_cxintr(struct vnet_softc *sc)
{
	while (sc->sc_cq.vq_last != sc->sc_cq.vq_vr.used->idx) {
		int id;
		uint32_t len;
		id = sc->sc_tq.vq_vr.used->ring[sc->sc_tq.vq_last & (sc->sc_tq.vq_vr.num-1)].id;
		len = sc->sc_tq.vq_vr.used->ring[sc->sc_tq.vq_last & (sc->sc_tq.vq_vr.num-1)].len;
		printf("cxi %02x %x\n", id, len);
		sc->sc_cq.vq_last++;
	}
}

static int
vnet_add_rxbuf(struct vnet_softc *sc, int x)
{
	struct vnet_rxsoft *rxs;
	struct vring_desc *vrd;
	struct mbuf *m;
	int error;

	MGETHDR(m, M_DONTWAIT, MT_DATA);
	if (m == NULL)
		return ENOBUFS;

	MCLAIM(m, &sc->sc_ec.ec_rx_mowner);

	MCLGET(m, M_DONTWAIT);

	if ((m->m_flags & M_EXT) == 0) {
		m_freem(m);
		return ENOBUFS;
	}

	rxs = &sc->sc_rxs[x];

	if (rxs->rx_mb != NULL)
		bus_dmamap_unload(sc->sc_vc.vc_bdt, rxs->rx_dm);

	rxs->rx_mb = m;

	error = bus_dmamap_load(sc->sc_vc.vc_bdt, rxs->rx_dm,
	    m->m_ext.ext_buf, m->m_ext.ext_size, NULL,
	    BUS_DMA_READ|BUS_DMA_NOWAIT);
	if (error) {
		panic("%s: can't load rx DMA map %d error %d",
		    device_xname(sc->sc_vc.vc_dev), x, error);
	}

	bus_dmamap_sync(sc->sc_vc.vc_bdt, rxs->rx_dm, 0,
	    rxs->rx_dm->dm_mapsize, BUS_DMASYNC_PREREAD);

	vrd = &sc->sc_rq.vq_vr.desc[(x<<1)|0];
	vrd->addr = VNET_RXVNHPA(sc, x);
	vrd->len = sizeof(struct virtio_net_hdr);
	vrd->flags = VRING_DESC_F_NEXT|VRING_DESC_F_WRITE;
	vrd->next = (x<<1)|1;
	//printf("rvnh %02x %016lx %08x %04x %04x\n", (x<<1), vrd->addr, vrd->len, vrd->flags, vrd->next);

	vrd = &sc->sc_rq.vq_vr.desc[(x<<1)|1];
	vrd->addr = rxs->rx_dm->dm_segs[0].ds_addr;
	vrd->len = rxs->rx_dm->dm_segs[0].ds_len;
	vrd->flags = VRING_DESC_F_WRITE;
	vrd->next = 0;
	//printf("rbuf %02x %016lx %08x %04x %04x\n", (x<<1)|1, vrd->addr, vrd->len, vrd->flags, vrd->next);

	sc->sc_rq.vq_vr.avail->ring[sc->sc_rq.vq_vr.avail->idx & (sc->sc_rq.vq_vr.num-1)] = x << 1;

	bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_rq.vq_bdm, 0, sc->sc_rq.vq_bdm->dm_mapsize, BUS_DMASYNC_PREWRITE);

	sc->sc_rq.vq_vr.avail->idx++;

	bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_rq.vq_bdm, 0, sc->sc_rq.vq_bdm->dm_mapsize, BUS_DMASYNC_PREWRITE);

	if ((sc->sc_rq.vq_vr.used->flags & VRING_USED_F_NO_NOTIFY) == 0) {
		virtio_virtq_notify(&sc->sc_vc, &sc->sc_rq);
	}

	return 0;
}

static int
vnet_init(struct ifnet *ifp)
{
	struct vnet_softc *sc;
	struct vnet_rxsoft *rxs;
	int error;
	int i;

	sc = ifp->if_softc;

	vnet_stop(ifp, 0);

	sc->sc_rxfree = sc->sc_rq.vq_vr.num;
	sc->sc_rxnext = 0;
	sc->sc_rxsfree = VNET_NRXDESC;
	sc->sc_rxsnext = 0;

	sc->sc_txfree = sc->sc_tq.vq_vr.num;
	sc->sc_txnext = 0;
	sc->sc_txsfree = VNET_NTXDESC;
	sc->sc_txsnext = 0;
	sc->sc_txsdirty = 0;

	for(i = 0; i < VNET_NRXDESC; i++) {
		rxs = &sc->sc_rxs[i];

		if (rxs->rx_mb == NULL) {
			if ((error = vnet_add_rxbuf(sc, i)) != 0) {
				return error;
			}
		}
	}

	ifp->if_flags |= IFF_RUNNING;
	ifp->if_flags &= ~IFF_OACTIVE;

	return 0;
}

static int
vnet_ioctl(struct ifnet *ifp, u_long command, void *data)
{
	int error;
	int s;

	s = splnet();
	error = ether_ioctl(ifp, command, data);
	if (error == ENETRESET) {
		error = 0;
	}

	splx(s);

	return error;
}

static void
vnet_start(struct ifnet *ifp)
{
	struct vnet_softc *sc;
	struct vnet_txsoft *txs;
	struct vring_desc *vrd;
	struct mbuf *m0;
	//int ofree;
	int error;
	int s;
	int added = 0;

	sc = ifp->if_softc;

	//ofree = sc->sc_txfree;

	if ((ifp->if_flags & (IFF_RUNNING | IFF_OACTIVE)) != IFF_RUNNING)
		return;

	for (;;) {
		IFQ_POLL(&ifp->if_snd, m0);
		if (m0 == NULL)
			break;

		txs = &sc->sc_txs[sc->sc_txsnext];
		
		error = bus_dmamap_load_mbuf(sc->sc_vc.vc_bdt, txs->tx_dm, m0,
			BUS_DMA_WRITE|BUS_DMA_NOWAIT);
		if (error)
			panic("XXX");

		if (txs->tx_dm->dm_nsegs + 1 >= sc->sc_txfree) {
			ifp->if_flags |= IFF_OACTIVE;
			printf("tx needs %d have %d\n", txs->tx_dm->dm_nsegs + 1, sc->sc_txfree);
			bus_dmamap_unload(sc->sc_vc.vc_bdt, txs->tx_dm);
			break;
		}

		IFQ_DEQUEUE(&ifp->if_snd, m0);

		/*
		 * WE ARE NOW COMMITTED TO TRANSMITTING THE PACKET.
		 */

		//printf("txs t %02x\n", sc->sc_txsnext);

		txs->tx_mb = m0;
		txs->tx_first = sc->sc_txnext;
		txs->tx_ndesc = txs->tx_dm->dm_nsegs + 1;

		bus_dmamap_sync(sc->sc_vc.vc_bdt, txs->tx_dm,
			0, txs->tx_dm->dm_mapsize, BUS_DMASYNC_PREWRITE);

#if 0
		(VNET_TXVNHVA(sc, sc->sc_txsnext))->hdr_len =
			sizeof(struct virtio_net_hdr);
#endif

		vrd = &sc->sc_tq.vq_vr.desc[sc->sc_txnext];

		vrd->addr = VNET_TXVNHPA(sc, sc->sc_txnext);
		vrd->len = sizeof(struct virtio_net_hdr);
		vrd->flags = VRING_DESC_F_NEXT;
		vrd->next = (sc->sc_txnext + 1) & (sc->sc_tq.vq_vr.num-1);
		//printf("tvnh %016lx %08x %04x %04x\n", vrd->addr, vrd->len, vrd->flags, vrd->next);

		sc->sc_txnext = (sc->sc_txnext + 1) & (sc->sc_tq.vq_vr.num-1);

		for (s = 0; s < txs->tx_dm->dm_nsegs; s++) {
			vrd = &sc->sc_tq.vq_vr.desc[sc->sc_txnext];
			vrd->addr = txs->tx_dm->dm_segs[s].ds_addr;
			vrd->len = txs->tx_dm->dm_segs[s].ds_len;
			if (s < txs->tx_dm->dm_nsegs - 1) {
				vrd->flags = VRING_DESC_F_NEXT;
				vrd->next = (sc->sc_txnext + 1) & (sc->sc_tq.vq_vr.num-1);
			} else { 
				vrd->flags = 0;
				vrd->next = 0;
			}
			//printf("tbuf %02x %016lx %08x %04x %04x\n", s, vrd->addr, vrd->len, vrd->flags, vrd->next);
			sc->sc_txnext = (sc->sc_txnext + 1) & (sc->sc_tq.vq_vr.num-1);
		}

		/* Pass the packet to any BPF listeners. */
		bpf_mtap(ifp, m0);

		sc->sc_tq.vq_vr.avail->ring[sc->sc_tq.vq_vr.avail->idx & (sc->sc_tq.vq_vr.num-1)] = txs->tx_first;
		added++;

		sc->sc_txsfree--;
		sc->sc_txsnext = VNET_NEXTTX(sc->sc_txsnext);
	}

	//printf("added %d\n", added);

	bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_tq.vq_bdm, 0, sc->sc_tq.vq_bdm->dm_mapsize, BUS_DMASYNC_PREWRITE);

	sc->sc_tq.vq_vr.avail->idx += added;

	bus_dmamap_sync(sc->sc_vc.vc_bdt, sc->sc_tq.vq_bdm, 0, sc->sc_tq.vq_bdm->dm_mapsize, BUS_DMASYNC_PREWRITE);

	if ((sc->sc_tq.vq_vr.used->flags & VRING_USED_F_NO_NOTIFY) == 0) {
		virtio_virtq_notify(&sc->sc_vc, &sc->sc_tq);
	}


	return;
}

static void
vnet_watchdog(struct ifnet *ifp)
{
	//struct vnet_softc * const sc = ifp->if_softc;

	ifp->if_oerrors++;

	vnet_init(ifp);

	vnet_start(ifp);

	return;
}

static void
vnet_stop(struct ifnet *ifp, int disable)
{
	struct vnet_softc *sc;

	sc = ifp->if_softc;

	callout_stop(&sc->sc_timeout);

	ifp->if_flags &= ~(IFF_RUNNING | IFF_OACTIVE);
	ifp->if_timer = 0;
}
