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

#include <dev/pci/pcivar.h>
#include <dev/pci/virtio/virtio_pci.h>
#include <dev/virtio/virtio_config.h>
#include <dev/virtio/virtio_ring.h>
#include <dev/virtio/virtio_console.h>
#include <dev/virtio/virtiovar.h>
#include <dev/virtio/vconvar.h>

#include <uvm/uvm_extern.h>

#define VCON_BUF_SIZE 4096

#define PCI_VENDOR_QUMRANET 0x1AF4
#define PCI_PRODUCT_QUMRANET_VIRTIO_MIN 0x1000
#define PCI_PRODUCT_QUMRANET_VIRTIO_MAX 0x103f

static int vcon_pci_match(device_t, cfdata_t, void *);
static void vcon_pci_attach(device_t, device_t, void *);
static int vcon_pci_intr(void *);

struct vcon_pci_softc {
	struct vcon_softc	sc_vcon;
	pci_chipset_tag_t	sc_pc;
	pcitag_t		sc_pcitag;
	void			*sc_ih;
};

CFATTACH_DECL_NEW(vcon_pci, sizeof(struct vcon_pci_softc),
    vcon_pci_match, vcon_pci_attach, NULL, NULL);

static int
vcon_pci_match(device_t parent, cfdata_t match, void *aux)
{
	struct pci_attach_args *pa = aux;
	pcireg_t subid;

	if (PCI_VENDOR(pa->pa_id) != PCI_VENDOR_QUMRANET)
		return 0;

	if (PCI_PRODUCT(pa->pa_id) > PCI_PRODUCT_QUMRANET_VIRTIO_MAX)
		return 0;

	if (PCI_PRODUCT(pa->pa_id) < PCI_PRODUCT_QUMRANET_VIRTIO_MIN)
		return 0;

	subid = pci_conf_read(pa->pa_pc, pa->pa_tag, PCI_SUBSYS_ID_REG);

	if (PCI_PRODUCT(subid) == VIRTIO_ID_CONSOLE)
		return 1;

	return 0;
}

static void
vcon_pci_attach(device_t parent, device_t self, void *aux)
{
	struct pci_attach_args *pa = aux;
	struct vcon_pci_softc *psc = device_private(self);
	struct vcon_softc *sc = &psc->sc_vcon;
	const char *intrstr;
	char intrbuf[PCI_INTRSTR_LEN];
	pci_intr_handle_t intrhandle;
	int ret;

	sc->sc_vc.vc_dev = self;
	psc->sc_pc = pa->pa_pc;
	psc->sc_pcitag = pa->pa_tag;

	aprint_naive("\n");
	aprint_normal("\n");

	ret = pci_mapreg_map(pa, PCI_MAPREG_START, PCI_MAPREG_TYPE_IO, 0,
			&sc->sc_vc.vc_iot, &sc->sc_vc.vc_ioh, NULL, &sc->sc_vc.vc_ios);

	if (ret) {
		aprint_error_dev(self, "couldn't map\n");
		return;
	}

	if (pci_dma64_available(pa))
		sc->sc_vc.vc_bdt = pa->pa_dmat64;
	else
		sc->sc_vc.vc_bdt = pa->pa_dmat;

	if (pci_intr_map(pa, &intrhandle) != 0) {
		/* XXX unmap space */
		return;
	}
	intrstr = pci_intr_string(pa->pa_pc, intrhandle, intrbuf, sizeof(intrbuf));
	psc->sc_ih = pci_intr_establish(pa->pa_pc, intrhandle,
			IPL_VM, vcon_pci_intr, sc);
	if (psc->sc_ih == NULL) {
		/* XXX unmap stuff */
		return;
	}

	aprint_normal_dev(self, "interrupting at %s\n",
			intrstr ? intrstr : "unknown interrupt");

	vcon_attach(sc);

	return;
}

static int
vcon_pci_intr(void *v)
{
	struct vcon_softc *sc = v;
	uint8_t isr;

	isr = bus_space_read_1(sc->sc_vc.vc_iot, sc->sc_vc.vc_ioh, VIRTIO_PCI_ISR);

	if (__predict_false((isr & VIRTIO_PCI_ISR_CONFIG) != 0))
		printf("%s: unimplemented config change interrupt\n", device_xname(sc->sc_vc.vc_dev));

	if ((isr & VIRTIO_PCI_ISR_IRQ) == 0)
		return 0;

	return vcon_intr(sc);
}
