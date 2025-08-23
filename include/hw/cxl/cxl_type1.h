/*
 * CXL Type 1 Device (Accelerator) Header
 * 
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef CXL_TYPE1_H
#define CXL_TYPE1_H

#include "hw/pci/pci.h"
#include "hw/cxl/cxl_device.h"
#include "hw/cxl/cxl_component.h"
#include "exec/memory.h"
#include "exec/memattrs.h"

#define TYPE_CXL_TYPE1 "cxl-type1"

typedef struct CXLType1State {
    PCIDevice parent;
    
    CXLComponentState cxl_cstate;
    CXLDeviceState cxl_dstate;
    
    MemoryRegion cache_mem;
    
    uint64_t cache_size;
    uint64_t sn;
    
    struct {
        char *host;
        uint16_t port;
        void *socket;
        bool connected;
    } memsim_server;
} CXLType1State;

#define CXL_TYPE1(obj) OBJECT_CHECK(CXLType1State, (obj), TYPE_CXL_TYPE1)

void cxl_type1_process_virtio_request(CXLType1State *ct1s, void *req, size_t size);
int cxl_type1_forward_to_memsim(CXLType1State *ct1s, uint32_t op_type, 
                                void *data, size_t size);

/* Memory access functions */
MemTxResult cxl_type1_read(PCIDevice *d, hwaddr host_addr, uint64_t *data,
                           unsigned size, MemTxAttrs attrs);
MemTxResult cxl_type1_write(PCIDevice *d, hwaddr host_addr, uint64_t data,
                            unsigned size, MemTxAttrs attrs);

#endif