/*
 * Zettai CXL VCS-capable switch object
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef CXL_VCS_SWITCH_H
#define CXL_VCS_SWITCH_H

#include "hw/cxl/cxl.h"
#include "hw/cxl/cxl_device.h"
#include "hw/pci-bridge/cxl_upstream_port.h"
#include "hw/qdev-core.h"
#include "qobject/qdict.h"
#include "qom/object.h"
#include "qom/object_interfaces.h"

#define TYPE_CXL_VCS_SWITCH "zettai"
OBJECT_DECLARE_TYPE(CXLVCSSwitch, CXLVCSSwitchClass, CXL_VCS_SWITCH)

#define CXL_VPPB_BINDING_STATUS_UNBOUND      0x00
#define CXL_VPPB_BINDING_STATUS_IN_PROGRESS  0x01
#define CXL_VPPB_BINDING_STATUS_BOUND_PORT   0x02
#define CXL_VPPB_BINDING_STATUS_BOUND_LD     0x03
#define CXL_VPPB_BINDING_STATUS_BOUND_PID    0x04

#define CXL_VPPB_UNBIND_WAIT_FOR_LINK_DOWN   0x0
#define CXL_VPPB_UNBIND_MANAGED_HOT_REMOVE   0x1
#define CXL_VPPB_UNBIND_SURPRISE_HOT_REMOVE  0x2

#define CXL_UNSUPPORTED_LD_ID                 0xffff
#define CXL_INVALID_BOUND_LD_ID               0xff

#define CXL_VCS_STATE_DISABLED                0x0
#define CXL_VCS_STATE_ENABLED                 0x1

#define CXL_MAX_VCS_PORTS                     8
#define CXL_MAX_VPPB_PER_VCS                  8
#define CXL_MAX_USP_PPBS                      CXL_MAX_VCS_PORTS
#define CXL_MAX_DSP_PPBS                      CXL_MAX_VPPB_PER_VCS
#define CXL_VPPB_LIST_LIMIT                   8

typedef struct CXLVPPBInfo {
    PCIDevice *dsp;
    uint8_t binding_status;
    uint8_t bound_port_id;
    uint8_t bound_ld_id;
} CXLVPPBInfo;

typedef struct CXLVCSInfoBlock {
    uint8_t vcs_id;
    uint8_t vcs_state;
    uint8_t usp_id;
    uint8_t num_vppbs;
    CXLVPPBInfo *vppbs[CXL_MAX_VPPB_PER_VCS];
} CXLVCSInfoBlock;

typedef struct CXLUpstreamPPB {
    CXLUpstreamPort *usp;
    CXLVCSInfoBlock *info;
} CXLUpstreamPPB;

typedef struct CXLDownstreamPPB {
    DeviceState *dev;
    QDict *opts;
    bool from_json;
    bool is_bound;
    uint8_t bound_vcs_id;
    uint8_t bound_vppb_id;
} CXLDownstreamPPB;

struct CXLVCSSwitch {
    Object parent_obj;

    uint8_t num_usp_ppbs;
    uint8_t num_dsp_ppbs;
    bool local_fm;

    CXLCCI swcci;
    DeviceListener listener;
    CXLUpstreamPPB *usp_ppbs[CXL_MAX_USP_PPBS];
    CXLDownstreamPPB *dsp_ppbs[CXL_MAX_DSP_PPBS];
};

struct CXLVCSSwitchClass {
    ObjectClass parent_class;

    CXLRetCode (*bind_vppb)(CXLVCSSwitch *sw, uint8_t vcs_id,
                            uint8_t vppb_id, uint8_t dsp_ppb_id,
                            uint16_t ld_id);
    CXLRetCode (*unbind_vppb)(CXLVCSSwitch *sw, uint8_t vcs_id,
                              uint8_t vppb_id, uint16_t option);
};

void cxl_vcs_register_usp(CXLVCSSwitch *sw, CXLUpstreamPort *usp,
                          Error **errp);
void cxl_vcs_register_vppb(CXLVCSSwitch *sw, CXLUpstreamPort *usp,
                           CXLDownstreamPort *dsp, Error **errp);
bool cxl_vcs_hide_device_listener(DeviceListener *listener,
                                  const QDict *device_opts, bool from_json,
                                  Error **errp);
void qmp_zettai_bind_vppb(const char *path, uint8_t vcs_id, uint8_t vppb_id,
                          uint8_t dsp_ppb_id, bool has_ld_id, uint16_t ld_id,
                          Error **errp);
void qmp_zettai_unbind_vppb(const char *path, uint8_t vcs_id, uint8_t vppb_id,
                            bool has_option, uint16_t option, Error **errp);

CXLRetCode cxl_vcs_bind_vppb(CXLVCSSwitch *sw, uint8_t vcs_id,
                              uint8_t vppb_id, uint8_t dsp_ppb_id,
                              uint16_t ld_id);
CXLRetCode cxl_vcs_unbind_vppb(CXLVCSSwitch *sw, uint8_t vcs_id,
                                uint8_t vppb_id, uint16_t option);

#endif
