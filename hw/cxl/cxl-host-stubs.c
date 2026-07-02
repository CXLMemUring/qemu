/*
 * CXL host parameter parsing routine stubs
 *
 * Copyright (c) 2022 Huawei
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/qapi-commands-cxl.h"
#include "hw/cxl/cxl.h"
#include "hw/cxl/cxl_host.h"

void cxl_fmws_link_targets(Error **errp) {};
void cxl_machine_init(Object *obj, CXLState *state) {};
void cxl_hook_up_pxb_registers(PCIBus *bus, CXLState *state, Error **errp) {};
hwaddr cxl_fmws_set_memmap(hwaddr base, hwaddr max_addr)
{
    return base;
};
void cxl_fmws_update_mmio(void) {};

const MemoryRegionOps cfmws_ops;

void qmp_zettai_bind_vppb(const char *path, uint8_t vcs_id, uint8_t vppb_id,
                          uint8_t dsp_ppb_id, bool has_ld_id, uint16_t ld_id,
                          Error **errp)
{
    error_setg(errp, "Zettai VCS switch support is not enabled");
}

void qmp_zettai_unbind_vppb(const char *path, uint8_t vcs_id, uint8_t vppb_id,
                            bool has_option, uint16_t option, Error **errp)
{
    error_setg(errp, "Zettai VCS switch support is not enabled");
}
