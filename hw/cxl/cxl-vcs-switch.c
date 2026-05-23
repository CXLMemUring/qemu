/*
 * Zettai CXL VCS-capable switch object
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/cxl/cxl_vcs_switch.h"
#include "monitor/qdev.h"
#include "qapi/error.h"
#include "qapi/qapi-commands-cxl.h"
#include "qobject/qdict.h"
#include "qom/object_interfaces.h"

static int cxl_vcs_qdict_get_int(const QDict *opts, const char *key,
                                 int default_value)
{
    const char *str = qdict_get_try_str(opts, key);

    if (str) {
        return atoi(str);
    }

    return qdict_get_try_int(opts, key, default_value);
}

static bool cxl_vcs_get_local_fm(Object *obj, Error **errp)
{
    return CXL_VCS_SWITCH(obj)->local_fm;
}

static void cxl_vcs_set_local_fm(Object *obj, bool value, Error **errp)
{
    CXL_VCS_SWITCH(obj)->local_fm = value;
}

static CXLVCSSwitch *cxl_vcs_resolve(const char *path, Error **errp)
{
    Object *obj = object_resolve_path_type(path, TYPE_CXL_VCS_SWITCH, NULL);

    if (!obj) {
        obj = object_resolve_path_component(object_get_objects_root(), path);
    }
    if (!obj || !object_dynamic_cast(obj, TYPE_CXL_VCS_SWITCH)) {
        error_setg(errp, "'%s' is not a Zettai switch object", path);
        return NULL;
    }

    return CXL_VCS_SWITCH(obj);
}

static void cxl_vcs_qmp_check_status(const char *op, CXLRetCode ret,
                                     Error **errp)
{
    if (ret != CXL_MBOX_SUCCESS) {
        error_setg(errp, "Zettai %s failed with mailbox status 0x%x", op, ret);
    }
}

void qmp_zettai_bind_vppb(const char *path, uint8_t vcs_id, uint8_t vppb_id,
                          uint8_t dsp_ppb_id, bool has_ld_id, uint16_t ld_id,
                          Error **errp)
{
    CXLVCSSwitch *sw = cxl_vcs_resolve(path, errp);
    CXLRetCode ret;

    if (!sw) {
        return;
    }

    ret = cxl_vcs_bind_vppb(sw, vcs_id, vppb_id, dsp_ppb_id,
                            has_ld_id ? ld_id : CXL_UNSUPPORTED_LD_ID);
    cxl_vcs_qmp_check_status("bind-vppb", ret, errp);
}

void qmp_zettai_unbind_vppb(const char *path, uint8_t vcs_id, uint8_t vppb_id,
                            bool has_option, uint16_t option, Error **errp)
{
    CXLVCSSwitch *sw = cxl_vcs_resolve(path, errp);
    CXLRetCode ret;

    if (!sw) {
        return;
    }

    ret = cxl_vcs_unbind_vppb(sw, vcs_id, vppb_id,
                              has_option ? option :
                              CXL_VPPB_UNBIND_SURPRISE_HOT_REMOVE);
    cxl_vcs_qmp_check_status("unbind-vppb", ret, errp);
}

static CXLRetCode cxl_vcs_bind_realized_vppb(CXLVCSSwitch *sw, uint8_t vcs_id,
                                             uint8_t vppb_id,
                                             uint8_t dsp_ppb_id,
                                             uint16_t ld_id)
{
    return CXL_MBOX_UNSUPPORTED;
}

static CXLRetCode cxl_vcs_bind_qdict_vppb(CXLVCSSwitch *sw, uint8_t vcs_id,
                                          uint8_t vppb_id,
                                          uint8_t dsp_ppb_id,
                                          uint16_t ld_id)
{
    Error *local_err = NULL;
    CXLVPPBInfo *vppb;
    CXLDownstreamPPB *dsppb;
    QDict *bind_opts;
    DeviceState *dev;
    PCIDevice *vppb_dev;
    uint8_t prev_binding_status;

    if (vcs_id >= sw->num_usp_ppbs || vppb_id >= CXL_MAX_VPPB_PER_VCS ||
        dsp_ppb_id >= sw->num_dsp_ppbs || !sw->usp_ppbs[vcs_id]) {
        return CXL_MBOX_INVALID_INPUT;
    }

    vppb = sw->usp_ppbs[vcs_id]->info->vppbs[vppb_id];
    dsppb = sw->dsp_ppbs[dsp_ppb_id];
    if (!vppb || !dsppb || dsppb->is_bound || vppb->binding_status !=
        CXL_VPPB_BINDING_STATUS_UNBOUND) {
        return CXL_MBOX_INVALID_INPUT;
    }

    vppb_dev = vppb->dsp;
    bind_opts = qdict_clone_shallow(dsppb->opts);
    qdict_put_str(bind_opts, "bus", vppb_dev->qdev.id);
    qdict_del(bind_opts, "vcs");
    qdict_del(bind_opts, "dsppb");

    prev_binding_status = vppb->binding_status;
    vppb->binding_status = CXL_VPPB_BINDING_STATUS_IN_PROGRESS;
    dev = qdev_device_add_from_qdict(bind_opts, dsppb->from_json, &local_err);
    qobject_unref(bind_opts);

    if (!dev || local_err) {
        vppb->binding_status = prev_binding_status;
        if (local_err) {
            error_report_err(local_err);
        }
        return CXL_MBOX_INTERNAL_ERROR;
    }

    vppb->bound_port_id = dsp_ppb_id;
    vppb->bound_ld_id = CXL_INVALID_BOUND_LD_ID;
    vppb->binding_status = CXL_VPPB_BINDING_STATUS_BOUND_PORT;

    dsppb->dev = dev;
    dsppb->is_bound = true;
    dsppb->bound_vcs_id = vcs_id;
    dsppb->bound_vppb_id = vppb_id;

    return CXL_MBOX_SUCCESS;
}

CXLRetCode cxl_vcs_bind_vppb(CXLVCSSwitch *sw, uint8_t vcs_id,
                              uint8_t vppb_id, uint8_t dsp_ppb_id,
                              uint16_t ld_id)
{
    if (!sw->local_fm) {
        return CXL_MBOX_UNSUPPORTED;
    }

    if (ld_id == CXL_UNSUPPORTED_LD_ID) {
        return cxl_vcs_bind_qdict_vppb(sw, vcs_id, vppb_id, dsp_ppb_id,
                                       ld_id);
    }

    return cxl_vcs_bind_realized_vppb(sw, vcs_id, vppb_id, dsp_ppb_id,
                                      ld_id);
}

static CXLRetCode cxl_vcs_unbind_qdict_vppb(CXLVCSSwitch *sw, uint8_t vcs_id,
                                            uint8_t vppb_id, uint16_t option)
{
    Error *local_err = NULL;
    CXLVPPBInfo *vppb;
    CXLDownstreamPPB *dsppb = NULL;

    if (vcs_id >= sw->num_usp_ppbs || vppb_id >= CXL_MAX_VPPB_PER_VCS ||
        !sw->usp_ppbs[vcs_id]) {
        return CXL_MBOX_INVALID_INPUT;
    }

    vppb = sw->usp_ppbs[vcs_id]->info->vppbs[vppb_id];
    if (!vppb || vppb->binding_status == CXL_VPPB_BINDING_STATUS_UNBOUND ||
        vppb->binding_status == CXL_VPPB_BINDING_STATUS_IN_PROGRESS) {
        return CXL_MBOX_INVALID_INPUT;
    }

    for (int i = 0; i < sw->num_dsp_ppbs; i++) {
        if (sw->dsp_ppbs[i] && sw->dsp_ppbs[i]->is_bound &&
            sw->dsp_ppbs[i]->bound_vcs_id == vcs_id &&
            sw->dsp_ppbs[i]->bound_vppb_id == vppb_id) {
            dsppb = sw->dsp_ppbs[i];
            break;
        }
    }

    if (!dsppb || !dsppb->dev) {
        return CXL_MBOX_INVALID_INPUT;
    }

    option &= 0xf;
    if (option == CXL_VPPB_UNBIND_WAIT_FOR_LINK_DOWN) {
        return CXL_MBOX_UNSUPPORTED;
    }

    qdev_unplug(dsppb->dev, &local_err);
    if (local_err) {
        error_report_err(local_err);
        return CXL_MBOX_INTERNAL_ERROR;
    }

    vppb->binding_status = CXL_VPPB_BINDING_STATUS_IN_PROGRESS;

    if (option == CXL_VPPB_UNBIND_SURPRISE_HOT_REMOVE) {
        object_unparent(OBJECT(dsppb->dev));
        dsppb->dev = NULL;
        dsppb->is_bound = false;
        dsppb->bound_vcs_id = 0;
        dsppb->bound_vppb_id = 0;
        vppb->binding_status = CXL_VPPB_BINDING_STATUS_UNBOUND;
        vppb->bound_port_id = 0;
        vppb->bound_ld_id = CXL_INVALID_BOUND_LD_ID;
    }

    return CXL_MBOX_SUCCESS;
}

CXLRetCode cxl_vcs_unbind_vppb(CXLVCSSwitch *sw, uint8_t vcs_id,
                                uint8_t vppb_id, uint16_t option)
{
    return cxl_vcs_unbind_qdict_vppb(sw, vcs_id, vppb_id, option);
}

void cxl_vcs_register_usp(CXLVCSSwitch *sw, CXLUpstreamPort *usp,
                          Error **errp)
{
    uint8_t ppb = usp->ppb;

    if (g_strcmp0(object_get_canonical_path_component(OBJECT(sw)),
                  usp->vcs_name)) {
        error_setg(errp, "VCS id for USP and switch do not match");
        return;
    }
    if (ppb >= sw->num_usp_ppbs || ppb >= CXL_MAX_USP_PPBS) {
        error_setg(errp, "vcs '%s': usppb %u is out of range",
                   object_get_canonical_path_component(OBJECT(sw)), ppb);
        return;
    }
    if (sw->usp_ppbs[ppb]) {
        error_setg(errp, "vcs '%s': usppb %u is already registered",
                   object_get_canonical_path_component(OBJECT(sw)), ppb);
        return;
    }

    sw->usp_ppbs[ppb] = g_new0(CXLUpstreamPPB, 1);
    sw->usp_ppbs[ppb]->usp = usp;
    sw->usp_ppbs[ppb]->info = g_new0(CXLVCSInfoBlock, 1);
    sw->usp_ppbs[ppb]->info->vcs_id = ppb;
    sw->usp_ppbs[ppb]->info->vcs_state = CXL_VCS_STATE_ENABLED;
    sw->usp_ppbs[ppb]->info->usp_id = ppb;
    usp->swcci.vcs = sw;
}

void cxl_vcs_register_vppb(CXLVCSSwitch *sw, CXLUpstreamPort *usp,
                           CXLDownstreamPort *dsp, Error **errp)
{
    CXLUpstreamPPB *vcs = NULL;

    for (int i = 0; i < sw->num_usp_ppbs; i++) {
        if (sw->usp_ppbs[i] && sw->usp_ppbs[i]->usp == usp) {
            vcs = sw->usp_ppbs[i];
            break;
        }
    }
    if (!vcs) {
        error_setg(errp, "USP was not found in the VCS switch");
        return;
    }

    for (int i = 0; i < CXL_MAX_VPPB_PER_VCS; i++) {
        if (!vcs->info->vppbs[i]) {
            CXLVPPBInfo *vppb = g_new0(CXLVPPBInfo, 1);

            vppb->dsp = PCI_DEVICE(dsp);
            vppb->binding_status = CXL_VPPB_BINDING_STATUS_UNBOUND;
            vppb->bound_ld_id = CXL_INVALID_BOUND_LD_ID;
            vcs->info->vppbs[i] = vppb;
            vcs->info->num_vppbs++;
            return;
        }
    }

    error_setg(errp, "No free VPPB slots in the VCS");
}

static void cxl_vcs_register_qdict_dsppb(CXLVCSSwitch *sw, const QDict *opts,
                                         bool from_json, Error **errp)
{
    int ppb = cxl_vcs_qdict_get_int(opts, "dsppb", -1);
    QDict *dev_opts;

    if (ppb < 0) {
        error_setg(errp, "No dsppb id given in CLI");
        return;
    }
    if (ppb >= sw->num_dsp_ppbs || ppb >= CXL_MAX_DSP_PPBS) {
        error_setg(errp, "vcs '%s': dsppb %u is out of range",
                   object_get_canonical_path_component(OBJECT(sw)), ppb);
        return;
    }
    if (sw->dsp_ppbs[ppb]) {
        error_setg(errp, "vcs '%s': dsppb %u is already occupied",
                   object_get_canonical_path_component(OBJECT(sw)), ppb);
        return;
    }

    dev_opts = qdict_clone_shallow(opts);
    qdict_del(dev_opts, "vcs");
    qdict_del(dev_opts, "dsppb");
    qdict_del(dev_opts, "bus");

    sw->dsp_ppbs[ppb] = g_new0(CXLDownstreamPPB, 1);
    sw->dsp_ppbs[ppb]->opts = dev_opts;
    sw->dsp_ppbs[ppb]->from_json = from_json;
}

bool cxl_vcs_hide_device_listener(DeviceListener *listener,
                                  const QDict *opts, bool from_json,
                                  Error **errp)
{
    CXLVCSSwitch *vcs = container_of(listener, CXLVCSSwitch, listener);
    const char *vcs_id = qdict_get_try_str(opts, "vcs");
    int ppb;

    if (!vcs_id || !qdict_haskey(opts, "dsppb")) {
        return false;
    }
    if (g_strcmp0(vcs_id, object_get_canonical_path_component(OBJECT(vcs)))) {
        return false;
    }

    ppb = cxl_vcs_qdict_get_int(opts, "dsppb", -1);
    if (ppb < 0 || ppb >= CXL_MAX_DSP_PPBS) {
        error_setg(errp, "Invalid dsppb id");
        return false;
    }
    if (vcs->dsp_ppbs[ppb]) {
        error_setg(errp, "dsppb %d is already populated", ppb);
        return false;
    }

    cxl_vcs_register_qdict_dsppb(vcs, opts, from_json, errp);
    return true;
}

static void cxl_vcs_ppb_unrealize_listener(DeviceListener *listener,
                                           DeviceState *dev)
{
    CXLVCSSwitch *sw = container_of(listener, CXLVCSSwitch, listener);
    CXLDownstreamPPB *dsppb = NULL;
    CXLVPPBInfo *vppb;

    for (int i = 0; i < sw->num_dsp_ppbs; i++) {
        if (sw->dsp_ppbs[i] && sw->dsp_ppbs[i]->dev == dev) {
            dsppb = sw->dsp_ppbs[i];
            break;
        }
    }
    if (!dsppb) {
        return;
    }

    vppb = sw->usp_ppbs[dsppb->bound_vcs_id]->info
        ->vppbs[dsppb->bound_vppb_id];
    dsppb->dev = NULL;
    dsppb->is_bound = false;
    dsppb->bound_vcs_id = 0;
    dsppb->bound_vppb_id = 0;
    vppb->binding_status = CXL_VPPB_BINDING_STATUS_UNBOUND;
    vppb->bound_port_id = 0;
    vppb->bound_ld_id = CXL_INVALID_BOUND_LD_ID;
}

static void cxl_vcs_switch_complete(UserCreatable *uc, Error **errp)
{
    CXLVCSSwitch *sw = CXL_VCS_SWITCH(uc);

    sw->listener.hide_device = cxl_vcs_hide_device_listener;
    sw->listener.unrealize = cxl_vcs_ppb_unrealize_listener;
    device_listener_register(&sw->listener);
}

static bool cxl_vcs_switch_can_be_deleted(UserCreatable *uc)
{
    return false;
}

static void vcs_get_usp_ppbs(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    uint8_t value = CXL_VCS_SWITCH(obj)->num_usp_ppbs;

    visit_type_uint8(v, name, &value, errp);
}

static void vcs_set_usp_ppbs(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    uint8_t value;

    if (!visit_type_uint8(v, name, &value, errp)) {
        return;
    }
    CXL_VCS_SWITCH(obj)->num_usp_ppbs = value;
}

static void vcs_get_dsp_ppbs(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    uint8_t value = CXL_VCS_SWITCH(obj)->num_dsp_ppbs;

    visit_type_uint8(v, name, &value, errp);
}

static void vcs_set_dsp_ppbs(Object *obj, Visitor *v, const char *name,
                             void *opaque, Error **errp)
{
    uint8_t value;

    if (!visit_type_uint8(v, name, &value, errp)) {
        return;
    }
    CXL_VCS_SWITCH(obj)->num_dsp_ppbs = value;
}

static void cxl_vcs_class_init(ObjectClass *oc, const void *data)
{
    CXLVCSSwitchClass *cc = CXL_VCS_SWITCH_CLASS(oc);
    UserCreatableClass *ucc = USER_CREATABLE_CLASS(oc);

    ucc->complete = cxl_vcs_switch_complete;
    ucc->can_be_deleted = cxl_vcs_switch_can_be_deleted;
    cc->bind_vppb = cxl_vcs_bind_vppb;
    cc->unbind_vppb = cxl_vcs_unbind_vppb;

    object_class_property_add_bool(oc, "local-fm",
                                   cxl_vcs_get_local_fm,
                                   cxl_vcs_set_local_fm);
    object_class_property_set_description(oc, "local-fm",
        "true when this QEMU instance owns the Fabric Manager CCI");
    object_class_property_add(oc, "usp-ppbs", "uint8",
                              vcs_get_usp_ppbs, vcs_set_usp_ppbs, NULL,
                              NULL);
    object_class_property_set_description(oc, "usp-ppbs",
        "Number of upstream physical PPBs in the switch");
    object_class_property_add(oc, "dsp-ppbs", "uint8",
                              vcs_get_dsp_ppbs, vcs_set_dsp_ppbs, NULL,
                              NULL);
    object_class_property_set_description(oc, "dsp-ppbs",
        "Number of downstream physical PPBs in the switch");
}

static void cxl_vcs_instance_init(Object *obj)
{
    CXLVCSSwitch *sw = CXL_VCS_SWITCH(obj);

    sw->local_fm = true;
}

static void cxl_vcs_instance_finalize(Object *obj)
{
    CXLVCSSwitch *sw = CXL_VCS_SWITCH(obj);

    device_listener_unregister(&sw->listener);
    for (int i = 0; i < CXL_MAX_USP_PPBS; i++) {
        if (sw->usp_ppbs[i]) {
            for (int j = 0; j < CXL_MAX_VPPB_PER_VCS; j++) {
                g_free(sw->usp_ppbs[i]->info->vppbs[j]);
            }
            g_free(sw->usp_ppbs[i]->info);
            g_free(sw->usp_ppbs[i]);
        }
    }
    for (int i = 0; i < CXL_MAX_DSP_PPBS; i++) {
        if (sw->dsp_ppbs[i]) {
            qobject_unref(sw->dsp_ppbs[i]->opts);
            g_free(sw->dsp_ppbs[i]);
        }
    }
}

static const InterfaceInfo cxl_vcs_interfaces[] = {
    { TYPE_USER_CREATABLE },
    { }
};

static const TypeInfo cxl_vcs_info = {
    .name = TYPE_CXL_VCS_SWITCH,
    .parent = TYPE_OBJECT,
    .instance_size = sizeof(CXLVCSSwitch),
    .class_size = sizeof(CXLVCSSwitchClass),
    .class_init = cxl_vcs_class_init,
    .instance_init = cxl_vcs_instance_init,
    .instance_finalize = cxl_vcs_instance_finalize,
    .interfaces = cxl_vcs_interfaces,
};

static void cxl_vcs_register(void)
{
    type_register_static(&cxl_vcs_info);
}

type_init(cxl_vcs_register)
