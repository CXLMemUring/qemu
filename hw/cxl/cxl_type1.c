/*
 * CXL Type 1 Device (Accelerator with Cache)
 * Forwards virtio accelerator requests to CXLMemSim server
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/range.h"
#include "qemu/rcu.h"
#include "qemu/sockets.h"
#include "qapi/error.h"
#include "hw/cxl/cxl.h"
#include "hw/cxl/cxl_device.h"
#include "hw/cxl/cxl_component.h"
#include "hw/cxl/cxl_cdat.h"
#include "hw/cxl/cxl_pci.h"
#include "hw/cxl/cxl_type1.h"
#include "hw/pci/pci.h"
#include "hw/pci/pcie.h"
#include "hw/pci/pcie_sriov.h"
#include "hw/pci/msix.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/resettable.h"
#include "hw/virtio/virtio.h"
#include "hw/virtio/virtio-crypto.h"
#include "io/channel-socket.h"
#include "qemu/thread.h"
#include "qemu/units.h"
#include "system/memory.h"

#define TYPE_CXL_TYPE1_DEV "cxl-type1-accel"
#define CXL_T1_VENDOR_ID 0x8086
#define CXL_T1_DEVICE_ID 0x0d90

/* Extended CXL Type 1 State with implementation details */
typedef struct CXLType1StateImpl {
    PCIDevice parent_obj;
    
    CXLComponentState cxl_cstate;
    CXLDeviceState cxl_dstate;
    
    struct {
        uint32_t cache_mem;
    } reg_state;
    
    MemoryRegion device_registers;
    MemoryRegion component_registers;
    MemoryRegion bar0;
    MemoryRegion cache_mem;
    
    struct {
        QIOChannelSocket *socket;
        char *server_addr;
        uint16_t server_port;
        QemuThread recv_thread;
        bool connected;
        QemuMutex lock;
    } cxlmemsim;
    
    struct {
        VirtQueue *dataq;
        VirtQueue *ctrlq;
        uint32_t max_queues;
        bool enabled;
    } virtio_accel;
    
    uint64_t device_size;
    uint64_t cache_size;
    uint64_t sn;
    
    struct {
        char *host;
        uint16_t port;
        void *socket_unused;
        bool connected_unused;
    } memsim_server;
    
    bool latency_enabled;
    uint32_t read_latency_ns;
    uint32_t write_latency_ns;
} CXLType1StateImpl;

#define CXL_TYPE1_DEV(obj) \
    OBJECT_CHECK(CXLType1StateImpl, (obj), TYPE_CXL_TYPE1_DEV)

typedef struct CXLMemSimMessage {
    uint32_t type;
    uint32_t size;
    uint64_t addr;
    uint8_t data[];
} CXLMemSimMessage;

enum CXLMemSimMsgType {
    CXLMEMSIM_MSG_READ = 1,
    CXLMEMSIM_MSG_WRITE,
    CXLMEMSIM_MSG_CRYPTO_OP,
    CXLMEMSIM_MSG_CACHE_FLUSH,
    CXLMEMSIM_MSG_RESPONSE,
};


static void build_dvsecs(CXLType1StateImpl *ct1d)
{
    CXLComponentState *cxl_cstate = &ct1d->cxl_cstate;
    uint8_t *dvsec;
    
    dvsec = (uint8_t *)&(CXLDVSECDevice){
        .cap = 0x1f,  /* Bit 0: Cache+, Bit 1: IO+, Bit 2: Mem+, Bit 3: Mem HWInit+, Bit 4: HDMCount=1 */
        .ctrl = 0x7,  /* Cache+ IO+ Mem+ enabled */
        .status = 0,
        .ctrl2 = 0,
        .status2 = 0x2,
        .lock = 0,
        .cap2 = (ct1d->cache_size >> 20) & 0xFFFF,  /* Cache size in MB for cap2 field */
        .range1_size_hi = ct1d->cache_size >> 32,
        .range1_size_lo = (ct1d->cache_size & 0xFFFFFFF0) | 0x3,  /* Valid, Active, Type=Cache */
        .range1_base_hi = 0,
        .range1_base_lo = 0,
        .range2_size_hi = ct1d->device_size >> 32,
        .range2_size_lo = (ct1d->device_size & 0xFFFFFFF0) | 0x1,  /* Valid, Type=Volatile */
        .range2_base_hi = 0,
        .range2_base_lo = 0,
    };
    
    cxl_component_create_dvsec(cxl_cstate, CXL2_TYPE3_DEVICE,
                              PCIE_CXL_DEVICE_DVSEC_LENGTH,
                              PCIE_CXL_DEVICE_DVSEC,
                              PCIE_CXL31_DEVICE_DVSEC_REVID,
                              dvsec);
    
    dvsec = (uint8_t *)&(CXLDVSECRegisterLocator){
        .rsvd = 0,
        .reg0_base_lo = RBI_COMPONENT_REG | CXL_COMPONENT_REG_BAR_IDX,
        .reg0_base_hi = 0,
        .reg1_base_lo = RBI_CXL_DEVICE_REG | CXL_DEVICE_REG_BAR_IDX,
        .reg1_base_hi = 0,
    };
    
    cxl_component_create_dvsec(cxl_cstate, CXL2_TYPE3_DEVICE,
                              REG_LOC_DVSEC_LENGTH, REG_LOC_DVSEC,
                              REG_LOC_DVSEC_REVID, dvsec);
    
    dvsec = (uint8_t *)&(CXLDVSECPortFlexBus){
        .cap = 0x26,
        .ctrl = 0x02,
        .status = 0x26,
        .rcvd_mod_ts_data_phase1 = 0xef,
    };
    
    cxl_component_create_dvsec(cxl_cstate, CXL2_TYPE3_DEVICE,
                              PCIE_CXL3_FLEXBUS_PORT_DVSEC_LENGTH, 
                              PCIE_FLEXBUS_PORT_DVSEC,
                              PCIE_CXL3_FLEXBUS_PORT_DVSEC_REVID, dvsec);
}

static void cxl_type1_reset(Object *obj, ResetType type)
{
    CXLType1StateImpl *ct1d = CXL_TYPE1_DEV(obj);
    uint32_t *reg_state = ct1d->cxl_cstate.crb.cache_mem_registers;
    uint32_t *write_msk = ct1d->cxl_cstate.crb.cache_mem_regs_write_mask;
    
    cxl_component_register_init_common(reg_state, write_msk, CXL2_TYPE3_DEVICE);
}

static void cxlmemsim_connect(CXLType1State *ct1d)
{
    Error *err = NULL;
    SocketAddress addr;
    
    if (ct1d->cxlmemsim.connected) {
        return;
    }
    
    addr.type = SOCKET_ADDRESS_TYPE_INET;
    addr.u.inet.host = ct1d->cxlmemsim.server_addr;
    addr.u.inet.port = g_strdup_printf("%u", ct1d->cxlmemsim.server_port);
    
    ct1d->cxlmemsim.socket = qio_channel_socket_new();
    if (qio_channel_socket_connect_sync(ct1d->cxlmemsim.socket, &addr, &err) < 0) {
        qemu_log("Warning: Failed to connect to CXLMemSim server at %s:%s: %s\n", 
                addr.u.inet.host, addr.u.inet.port, error_get_pretty(err));
        error_free(err);
        g_free(addr.u.inet.port);
        object_unref(OBJECT(ct1d->cxlmemsim.socket));
        ct1d->cxlmemsim.socket = NULL;
        return;
    }
    
    ct1d->cxlmemsim.connected = true;
    g_free(addr.u.inet.port);
    qemu_log("Connected to CXLMemSim server at %s:%u\n", 
            ct1d->cxlmemsim.server_addr, ct1d->cxlmemsim.server_port);
}

static void cxlmemsim_disconnect(CXLType1State *ct1d)
{
    if (!ct1d->cxlmemsim.connected) {
        return;
    }
    
    ct1d->cxlmemsim.connected = false;
    if (ct1d->cxlmemsim.socket) {
        qio_channel_close(QIO_CHANNEL(ct1d->cxlmemsim.socket), NULL);
        object_unref(OBJECT(ct1d->cxlmemsim.socket));
        ct1d->cxlmemsim.socket = NULL;
    }
}


static void *cxlmemsim_recv_thread(void *opaque)
{
    CXLType1State *ct1d = opaque;
    CXLMemSimMessage header;
    Error *err = NULL;
    
    while (ct1d->cxlmemsim.connected) {
        if (qio_channel_read_all(QIO_CHANNEL(ct1d->cxlmemsim.socket),
                                 (char *)&header, sizeof(header), &err) < 0) {
            if (ct1d->cxlmemsim.connected) {
                error_report("Failed to receive from CXLMemSim: %s",
                           error_get_pretty(err));
                error_free(err);
            }
            break;
        }
        
        if (header.type == CXLMEMSIM_MSG_RESPONSE && header.size > 0) {
            uint8_t *data = g_malloc(header.size);
            if (qio_channel_read_all(QIO_CHANNEL(ct1d->cxlmemsim.socket),
                                    (char *)data, header.size, &err) < 0) {
                error_report("Failed to receive data from CXLMemSim: %s",
                           error_get_pretty(err));
                error_free(err);
                g_free(data);
                break;
            }
            
            g_free(data);
        }
    }
    
    return NULL;
}



static void ct1_reg_write(void *opaque, hwaddr offset, uint64_t value, 
                         unsigned size)
{
    CXLComponentState *cxl_cstate = opaque;
    
    if (offset >= CXL2_COMPONENT_CM_REGION_SIZE) {
        qemu_log_mask(LOG_UNIMP,
                     "CXL Type1: Unimplemented register write at 0x%lx\n",
                     offset);
        return;
    }
    
    stl_le_p((uint8_t *)cxl_cstate->crb.cache_mem_registers + offset, value);
}

static uint64_t ct1_reg_read(void *opaque, hwaddr offset, unsigned size)
{
    CXLComponentState *cxl_cstate = opaque;
    
    if (offset >= CXL2_COMPONENT_CM_REGION_SIZE) {
        qemu_log_mask(LOG_UNIMP,
                     "CXL Type1: Unimplemented register read at 0x%lx\n",
                     offset);
        return 0;
    }
    
    return ldl_le_p((uint8_t *)cxl_cstate->crb.cache_mem_registers + offset);
}

static const MemoryRegionOps ct1_reg_ops = {
    .read = ct1_reg_read,
    .write = ct1_reg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static void cxl_type1_realize(PCIDevice *pci_dev, Error **errp)
{
    CXLType1StateImpl *ct1d = CXL_TYPE1_DEV(pci_dev);
    CXLComponentState *cxl_cstate = &ct1d->cxl_cstate;
    
    pci_config_set_prog_interface(pci_dev->config, 0x10);
    
    /* Set default values */
    if (!ct1d->cxlmemsim.server_addr) {
        ct1d->cxlmemsim.server_addr = g_strdup("127.0.0.1");
    }
    if (ct1d->cxlmemsim.server_port == 0) {
        ct1d->cxlmemsim.server_port = 9999;
    }
    if (ct1d->device_size == 0) {
        ct1d->device_size = 256 * MiB;
    }
    if (ct1d->cache_size == 0) {
        ct1d->cache_size = 64 * MiB;
    }
    
    qemu_mutex_init(&ct1d->cxlmemsim.lock);
    
    pcie_endpoint_cap_init(pci_dev, 0x80);
    if (ct1d->sn != 0) {
        pcie_dev_ser_num_init(pci_dev, 0x100, ct1d->sn);
        cxl_cstate->dvsec_offset = 0x100 + 0x0c;
    } else {
        cxl_cstate->dvsec_offset = 0x100;
    }
    
    ct1d->cxl_cstate.pdev = pci_dev;
    build_dvsecs(ct1d);
    
    cxl_component_register_block_init(OBJECT(pci_dev), cxl_cstate,
                                      TYPE_CXL_TYPE1_DEV);
    
    memory_region_init(&ct1d->bar0, OBJECT(ct1d), "cxl-type1-bar0",
                      CXL2_COMPONENT_BLOCK_SIZE);
    
    memory_region_init_io(&ct1d->component_registers, OBJECT(ct1d),
                         &ct1_reg_ops, cxl_cstate, "cxl-type1-component",
                         CXL2_COMPONENT_CM_REGION_SIZE);
    memory_region_add_subregion(&ct1d->bar0, 0, &ct1d->component_registers);
    
    pci_register_bar(pci_dev, 0,
                    PCI_BASE_ADDRESS_SPACE_MEMORY |
                    PCI_BASE_ADDRESS_MEM_TYPE_64,
                    &ct1d->bar0);
    
    if (msix_init_exclusive_bar(pci_dev, 10, 4, NULL)) {
        error_setg(errp, "Failed to initialize MSI-X");
        return;
    }
    
    
    cxlmemsim_connect(ct1d);
    if (ct1d->cxlmemsim.connected) {
        qemu_thread_create(&ct1d->cxlmemsim.recv_thread, "cxlmemsim-recv",
                          cxlmemsim_recv_thread, ct1d, QEMU_THREAD_JOINABLE);
    }
}

static void cxl_type1_exit(PCIDevice *pci_dev)
{
    CXLType1StateImpl *ct1d = CXL_TYPE1_DEV(pci_dev);
    
    cxlmemsim_disconnect(ct1d);
    if (ct1d->cxlmemsim.recv_thread.thread) {
        qemu_thread_join(&ct1d->cxlmemsim.recv_thread);
    }
    qemu_mutex_destroy(&ct1d->cxlmemsim.lock);
}

static const Property cxl_type1_props[] = {
    DEFINE_PROP_SIZE("size", CXLType1State, device_size, 256 * MiB),
    DEFINE_PROP_SIZE("cache-size", CXLType1State, cache_size, 64 * MiB),
    DEFINE_PROP_STRING("cxlmemsim-addr", CXLType1State, cxlmemsim.server_addr),
    DEFINE_PROP_UINT16("cxlmemsim-port", CXLType1State, cxlmemsim.server_port, 9999),
};

static void cxl_type1_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);
    
    pc->realize = cxl_type1_realize;
    pc->exit = cxl_type1_exit;
    pc->vendor_id = CXL_T1_VENDOR_ID;
    pc->device_id = CXL_T1_DEVICE_ID;
    pc->revision = 1;
    pc->class_id = PCI_CLASS_MEMORY_CXL;
    
    dc->desc = "CXL Type 1 Accelerator Device";
    rc->phases.hold = cxl_type1_reset;
    device_class_set_props(dc, cxl_type1_props);
}

static const TypeInfo cxl_type1_info = {
    .name = TYPE_CXL_TYPE1_DEV,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(CXLType1State),
    .class_init = cxl_type1_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { INTERFACE_CXL_DEVICE },
        { }
    },
};

static void cxl_type1_register_types(void)
{
    type_register_static(&cxl_type1_info);
}

type_init(cxl_type1_register_types)

/* Memory access functions for CXL Type 1 cache device */
MemTxResult cxl_type1_read(PCIDevice *d, hwaddr host_addr, uint64_t *data,
                           unsigned size, MemTxAttrs attrs)
{
    CXLType1StateImpl *ct1d = CXL_TYPE1_DEV(d);
    
    if (host_addr < ct1d->cache_size) {
        /* Reading from cache region */
        qemu_log_mask(LOG_UNIMP, "CXL Type1: Cache read at 0x%lx, size %u\n", 
                     host_addr, size);
        *data = 0;  /* Return dummy data for now */
        return MEMTX_OK;
    } else if (host_addr < ct1d->cache_size + ct1d->device_size) {
        /* Reading from device memory region */
        hwaddr offset = host_addr - ct1d->cache_size;
        qemu_log_mask(LOG_UNIMP, "CXL Type1: Device memory read at 0x%lx, size %u\n", 
                     offset, size);
        *data = 0;  /* Return dummy data for now */
        return MEMTX_OK;
    }
    
    return MEMTX_ERROR;
}

MemTxResult cxl_type1_write(PCIDevice *d, hwaddr host_addr, uint64_t data,
                            unsigned size, MemTxAttrs attrs)
{
    CXLType1StateImpl *ct1d = CXL_TYPE1_DEV(d);
    
    if (host_addr < ct1d->cache_size) {
        /* Writing to cache region */
        qemu_log_mask(LOG_UNIMP, "CXL Type1: Cache write at 0x%lx, data 0x%lx, size %u\n", 
                     host_addr, data, size);
        return MEMTX_OK;
    } else if (host_addr < ct1d->cache_size + ct1d->device_size) {
        /* Writing to device memory region */
        hwaddr offset = host_addr - ct1d->cache_size;
        qemu_log_mask(LOG_UNIMP, "CXL Type1: Device memory write at 0x%lx, data 0x%lx, size %u\n", 
                     offset, data, size);
        return MEMTX_OK;
    }
    
    return MEMTX_ERROR;
};