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

#ifndef _VIRTIO_IF_VNETVAR_H_
#define _VIRTIO_IF_VNETVAR_H_

#define VNET_IFQUEUELEN	256

#define VNET_NRXDESC	64
#define VNET_NRXDESC_MASK (VNET_NRXDESC - 1)
#define VNET_NEXTRX(x)	(((x) + 1) & VNET_NRXDESC_MASK)

#define VNET_NTXDESC	64
#define VNET_NTXDESC_MASK (VNET_NTXDESC - 1)
#define VNET_NEXTTX(x)  (((x) + 1) & VNET_NTXDESC_MASK)

#define VNET_NTXSEGS	32

#define VNET_MAXTXDMA	round_page(IP_MAXPACKET)

struct vnet_txsoft {
	struct mbuf *tx_mb;
	bus_dmamap_t tx_dm;
	struct virtio_net_hdr *tx_nh;
	int tx_first;
	int tx_ndesc;
};      
 
struct vnet_rxsoft {
	struct mbuf *rx_mb;
	bus_dmamap_t rx_dm;
	struct virtio_net_hdr *rx_nh;
	int rx_first;
	int rx_ndesc;
};

struct vnet_softc {
	struct virtio_common	sc_vc;
	struct ethercom		sc_ec;

	struct virtio_virtq	sc_rq;
	struct virtio_virtq	sc_tq;
	struct virtio_virtq	sc_cq;

	struct vnet_rxsoft	sc_rxs[VNET_NRXDESC];
	struct vnet_txsoft	sc_txs[VNET_NTXDESC];

	bus_dmamap_t		sc_ddm;
	void *			sc_dva;

	callout_t		sc_timeout;

	int			sc_rxfree;
	int			sc_rxnext;
	int			sc_rxsfree;
	int			sc_rxsnext;

	int			sc_txfree;
	int			sc_txnext;

	int			sc_txsfree;
	int			sc_txsnext;
	int			sc_txsdirty;
};

void vnet_attach(struct vnet_softc *);
int vnet_intr(struct vnet_softc *);

#endif /* !_VIRTIO_IF_VNETVAR_H_ */
