/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Emulation of a CXL Switch Mailbox CCI PCIe function.
 *
 * Copyright (c) 2023 Huawei Technologies.
 *
 * From www.computeexpresslink.org
 * Compute Express Link (CXL) Specification revision 3.0 Version 1.0
 */
#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci-bridge/cxl_upstream_port.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/cxl/cxl.h"
#include "hw/cxl/cxl_vcs_switch.h"

#define CXL_SWCCI_MSIX_MBOX 3
#define PCI_VENDOR_ID_ZETTAI 0x7a74
#define PCI_DEVICE_ID_ZETTAI_SWITCH_MAILBOX_CCI 0xa123

static void cswmbcci_reset(DeviceState *dev)
{
    CSWMBCCIDev *cswmb = CXL_SWITCH_MAILBOX_CCI(dev);
    cxl_device_register_init_swcci(cswmb, CXL_SWCCI_MSIX_MBOX);
}

static void cswbcci_realize(PCIDevice *pci_dev, Error **errp)
{
    CSWMBCCIDev *cswmb = CXL_SWITCH_MAILBOX_CCI(pci_dev);
    CXLComponentState *cxl_cstate = &cswmb->cxl_cstate;
    CXLDeviceState *cxl_dstate = &cswmb->cxl_dstate;
    CXLDVSECRegisterLocator *regloc_dvsec;
    CXLUpstreamPort *usp = NULL;
    CXLVCSSwitch *vcs = NULL;

    if (!cswmb->target) {
        error_setg(errp, "Target not set");
        return;
    }
    if (object_dynamic_cast(cswmb->target, TYPE_CXL_VCS_SWITCH)) {
        vcs = CXL_VCS_SWITCH(cswmb->target);
        if (!vcs->usp_ppbs[0]) {
            error_setg(errp, "VCS target requires a registered usppb 0");
            return;
        }
        usp = vcs->usp_ppbs[0]->usp;
    } else if (object_dynamic_cast(cswmb->target, TYPE_CXL_USP)) {
        usp = CXL_USP(cswmb->target);
    } else {
        error_setg(errp, "Target must be a cxl-upstream or Zettai switch");
        return;
    }

    pcie_endpoint_cap_init(pci_dev, 0x80);
    cxl_cstate->dvsec_offset = 0x100;
    cxl_cstate->pdev = pci_dev;
    cswmb->cci = vcs ? &vcs->swcci : &usp->swcci;
    cxl_device_register_block_init(OBJECT(pci_dev), cxl_dstate, cswmb->cci);
    pci_register_bar(pci_dev, 0,
                     PCI_BASE_ADDRESS_SPACE_MEMORY |
                         PCI_BASE_ADDRESS_MEM_TYPE_64,
                     &cxl_dstate->device_registers);
    regloc_dvsec = &(CXLDVSECRegisterLocator) {
        .rsvd         = 0,
        .reg0_base_lo = RBI_CXL_DEVICE_REG | 0,
        .reg0_base_hi = 0,
    };
    cxl_component_create_dvsec(cxl_cstate, CXL3_SWITCH_MAILBOX_CCI,
                               REG_LOC_DVSEC_LENGTH, REG_LOC_DVSEC,
                               REG_LOC_DVSEC_REVID, (uint8_t *)regloc_dvsec);

    if (vcs) {
        cxl_initialize_vcs_swcci(cswmb->cci, vcs, DEVICE(pci_dev),
                                 DEVICE(usp), CXL_MAILBOX_MAX_PAYLOAD_SIZE);
    } else {
        cxl_initialize_mailbox_swcci(cswmb->cci, DEVICE(pci_dev),
                                     DEVICE(usp),
                                     CXL_MAILBOX_MAX_PAYLOAD_SIZE);
    }
}

static void cswmbcci_exit(PCIDevice *pci_dev)
{
    /* Nothing to do here yet */
}

static const Property cxl_switch_cci_props[] = {
    DEFINE_PROP_LINK("target", CSWMBCCIDev,
                     target, TYPE_OBJECT, Object *),
};

static void cswmbcci_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(oc);

    pc->realize = cswbcci_realize;
    pc->exit = cswmbcci_exit;
    /* Serial bus, CXL Switch CCI */
    pc->class_id = 0x0c0b;
    /*
     * Zettai emulated CXL Switch Mailbox CCI. The vendor id is reserved for
     * this simulator tree and is not a PCI-SIG assigned production id.
     */
    pc->vendor_id = PCI_VENDOR_ID_ZETTAI;
    pc->device_id = PCI_DEVICE_ID_ZETTAI_SWITCH_MAILBOX_CCI;
    pc->revision = 0;
    dc->desc = "CXL Switch Mailbox CCI";
    device_class_set_legacy_reset(dc, cswmbcci_reset);
    device_class_set_props(dc, cxl_switch_cci_props);
}

static const TypeInfo cswmbcci_info = {
    .name = TYPE_CXL_SWITCH_MAILBOX_CCI,
    .parent = TYPE_PCI_DEVICE,
    .class_init = cswmbcci_class_init,
    .instance_size = sizeof(CSWMBCCIDev),
    .interfaces = (const InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void cxl_switch_mailbox_cci_register(void)
{
    type_register_static(&cswmbcci_info);
}
type_init(cxl_switch_mailbox_cci_register);
