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

#ifndef _DEV_VIRTIO_VIRTIO_NET_H_
#define _DEV_VIRTIO_VIRTIO_NET_H_

#define VIRTIO_ID_NET 1

#define VIRTIO_NET_F_CSUM	__BIT(0)
#define VIRTIO_NET_F_GUEST_CSUM	__BIT(1)
#define VIRTIO_NET_F_MAC	__BIT(5)
#define VIRTIO_NET_F_GSO	__BIT(6)
#define VIRTIO_NET_F_GUEST_TSO4	__BIT(7)
#define VIRTIO_NET_F_GUEST_TSO6	__BIT(8)
#define VIRTIO_NET_F_GUEST_ECN	__BIT(9)
#define VIRTIO_NET_F_GUEST_UFO	__BIT(10)
#define VIRTIO_NET_F_HOST_TSO4	__BIT(11)
#define VIRTIO_NET_F_HOST_TSO6	__BIT(12)
#define VIRTIO_NET_F_HOST_ECN	__BIT(13)
#define VIRTIO_NET_F_HOST_UFO	__BIT(14)
#define VIRTIO_NET_F_MRG_RXBUF	__BIT(15)
#define VIRTIO_NET_F_STATUS	__BIT(16)
#define VIRTIO_NET_F_CTRL_VQ	__BIT(17)
#define VIRTIO_NET_F_CTRL_RX	__BIT(18)
#define VIRTIO_NET_F_CTRL_VLAN	__BIT(19)

#define VIRTIO_NET_S_LINK_UP	__BIT(0)

#define VIRTIO_NET_HDR_F_NEEDS_CSUM	__BIT(0)

#define VIRTIO_NET_HDR_GSO_NONE		0
#define VIRTIO_NET_HDR_GSO_TCPV4	1
#define VIRTIO_NET_HDR_GSO_UDP		3
#define VIRTIO_NET_HDR_GSO_TCPV6	4
#define VIRTIO_NET_HDR_GSO_ECN		0x80

struct virtio_net_config {
	uint8_t mac[ETHER_ADDR_LEN];
	uint16_t status;
} __packed;

#define VIRTIO_NET_CONFIG_MAC \
	(offsetof(struct virtio_net_config, mac))
#define VIRTIO_NET_CONFIG_STATUS \
	(offsetof(struct virtio_net_config, status))

struct virtio_net_hdr {
	uint8_t		flags;
	uint8_t		gso_type;
	uint16_t	hdr_len;
	uint16_t	gso_size;
	uint16_t	csum_start;
	uint16_t	csum_offset;
} __packed;

#define VIRTIO_NET_RECEIVEQ	0
#define VIRTIO_NET_TRANSMITQ	1
#define VIRTIO_NET_CONTROLQ	2

#endif /* !_DEV_VIRTIO_VIRTIO_NET_H_ */
