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

#ifndef _DEV_VIRTIO_VIRTIO_CONSOLE_H_
#define _DEV_VIRTIO_VIRTIO_CONSOLE_H_

#define VIRTIO_ID_CONSOLE 3

#define VIRTIO_CONSOLE_F_SIZE __BIT(0)
#define VIRTIO_CONSOLE_F_MULTIPORT __BIT(1)

struct virtio_console_config {
	uint16_t cols;
	uint16_t rows;
	uint32_t max_nr_ports;
	uint32_t nr_ports;
} __packed;

struct virtio_console_control {
	uint32_t id;
	uint16_t event;
	uint16_t value;
} __packed;

#define VIRTIO_CONSOLE_RECEIVEQ 0
#define VIRTIO_CONSOLE_TRANSMITQ 1

#endif /* !_DEV_VIRTIO_VIRTIO_CONSOLE_H_ */
