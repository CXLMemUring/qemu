/*
 * CXL Type 2 Device (Accelerator with Coherent Memory)
 * GPU Passthrough Forwarder with CPU-GPU Coherency
 *
 * This implements a CXL Type 2 device that combines:
 * - Type 1 cache coherency (CXL.cache protocol)
 * - Type 3 device-attached memory (CXL.mem protocol)
 * - GPU passthrough via VFIO
 * - Full coherency protocol between CPU and GPU memory
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "hw/cxl/cxl_type2.h"
#include "crypto/hash.h"
#include "hw/cxl/cxl.h"
#include "hw/cxl/cxl_cdat.h"
#include "hw/cxl/cxl_component.h"
#include "hw/cxl/cxl_device.h"
#include "hw/cxl/cxl_hetgpu.h"
#include "hw/cxl/cxl_pci.h"
#include "hw/cxl/cxl_type2_coherency.h"
#include "hw/cxl/cxl_type2_gpu_cmd.h"
#include "hw/cxl/slugarch_type2_protocol.h"
#include "hw/pci/msix.h"
#include "hw/pci/pci.h"
#include "hw/pci/pcie.h"
#include "hw/qdev-properties-system.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/range.h"
#include "qemu/rcu.h"
#include "qemu/sockets.h"
#include "qemu/units.h"
#include "system/memory.h"

#define CXL_TYPE2_DEVICE_REG_OFFSET CXL2_COMPONENT_BLOCK_SIZE
#define CXL_TYPE2_BAR0_SIZE (2 * CXL2_COMPONENT_BLOCK_SIZE)
#include "io/channel-socket.h"
#include <fcntl.h>
#include <linux/vfio.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef DEFAULT_HETGPU_LIB_PATH
#define DEFAULT_HETGPU_LIB_PATH NULL
#endif

#define CXL_OP_READ 0
#define CXL_OP_WRITE 1
#define CXL_OP_FENCE 5
#define CXL_OP_BI_ENABLE 14
#define CXL_OP_BI_DISABLE 15
#define CXL_OP_BI_INVALIDATE 16
#define CXL_OP_BI_WRITEBACK 17
#define CXL_OP_BI_QUERY 18

typedef struct QEMU_PACKED CXLMemSimRequest {
    uint8_t op_type;
    uint64_t addr;
    uint64_t size;
    uint64_t timestamp;
    uint64_t value;
    uint64_t expected;
    uint8_t data[64];
} CXLMemSimRequest;

typedef struct QEMU_PACKED CXLMemSimResponse {
    uint8_t status;
    uint64_t latency_ns;
    uint64_t old_value;
    uint8_t data[64];
} CXLMemSimResponse;

/* Forward declarations for hetGPU coherency integration */
static void cxl_type2_hetgpu_coherency_callback(void *opaque, uint64_t addr, uint64_t size, bool invalidate);
static bool cxl_type2_memsim_request_ext(CXLType2State *ct2d, uint8_t op_type, uint64_t addr, uint64_t size,
                                         const uint8_t *data, uint64_t value, uint64_t expected,
                                         CXLMemSimResponse *resp);
static bool cxl_type2_memsim_request(CXLType2State *ct2d, uint8_t op_type, uint64_t addr, uint64_t size,
                                     const uint8_t *data, CXLMemSimResponse *resp);

/* ========================================================================
 * Coherency Protocol Implementation
 * ======================================================================== */

void cxl_type2_coherency_init(CXLType2State *ct2d) {
    qemu_mutex_init(&ct2d->coherency.lock);
    ct2d->coherency.cache_lines = g_hash_table_new_full(g_int64_hash, g_int64_equal, g_free, g_free);
    ct2d->coherency.cache_hits = 0;
    ct2d->coherency.cache_misses = 0;
    ct2d->coherency.coherency_ops = 0;
    ct2d->coherency.snoops = 0;

    qemu_log("CXL Type2: Coherency protocol initialized\n");
}

void cxl_type2_coherency_cleanup(CXLType2State *ct2d) {
    if (ct2d->coherency.cache_lines) {
        g_hash_table_destroy(ct2d->coherency.cache_lines);
        ct2d->coherency.cache_lines = NULL;
    }
    qemu_mutex_destroy(&ct2d->coherency.lock);

    qemu_log("CXL Type2: Coherency stats - Hits: %lu, Misses: %lu, Ops: %lu, Snoops: %lu\n", ct2d->coherency.cache_hits,
             ct2d->coherency.cache_misses, ct2d->coherency.coherency_ops, ct2d->coherency.snoops);
}

CXLCacheLine *cxl_type2_cache_lookup(CXLType2State *ct2d, uint64_t addr) {
    /* Align to cache line boundary */
    uint64_t cache_line_addr = addr & ~0x3F;
    CXLCacheLine *line;

    qemu_mutex_lock(&ct2d->coherency.lock);
    line = g_hash_table_lookup(ct2d->coherency.cache_lines, &cache_line_addr);

    if (line) {
        ct2d->coherency.cache_hits++;
    } else {
        ct2d->coherency.cache_misses++;
    }
    qemu_mutex_unlock(&ct2d->coherency.lock);

    return line;
}

void cxl_type2_cache_insert(CXLType2State *ct2d, uint64_t addr, const uint8_t *data, CXLCoherencyState state) {
    uint64_t cache_line_addr = addr & ~0x3F;
    CXLCacheLine *line = g_new0(CXLCacheLine, 1);
    uint64_t *key = g_new(uint64_t, 1);

    *key = cache_line_addr;
    line->tag = cache_line_addr;
    line->state = state;
    line->dirty = false;
    line->timestamp = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (data) {
        memcpy(line->data, data, 64);
    }

    qemu_mutex_lock(&ct2d->coherency.lock);
    g_hash_table_insert(ct2d->coherency.cache_lines, key, line);
    ct2d->coherency.coherency_ops++;
    qemu_mutex_unlock(&ct2d->coherency.lock);

    qemu_log_mask(LOG_TRACE, "CXL Type2: Cache insert at 0x%lx, state=%d\n", cache_line_addr, state);
}

void cxl_type2_cache_invalidate(CXLType2State *ct2d, uint64_t addr) {
    uint64_t cache_line_addr = addr & ~0x3F;

    qemu_mutex_lock(&ct2d->coherency.lock);
    if (g_hash_table_remove(ct2d->coherency.cache_lines, &cache_line_addr)) {
        ct2d->coherency.coherency_ops++;
        qemu_log_mask(LOG_TRACE, "CXL Type2: Cache invalidate at 0x%lx\n", cache_line_addr);
    }
    qemu_mutex_unlock(&ct2d->coherency.lock);

    if (ct2d->bi_enabled) {
        cxl_type2_memsim_request(ct2d, CXL_OP_BI_INVALIDATE, cache_line_addr, 64, NULL, NULL);
    }
}

void cxl_type2_cache_writeback(CXLType2State *ct2d, uint64_t addr) {
    uint64_t cache_line_addr = addr & ~0x3F;
    CXLCacheLine *line;

    qemu_mutex_lock(&ct2d->coherency.lock);
    line = g_hash_table_lookup(ct2d->coherency.cache_lines, &cache_line_addr);

    if (line && line->dirty) {
        /* Write back to device memory */
        if (cache_line_addr < ct2d->device_mem_size) {
            uint8_t *mem_ptr = memory_region_get_ram_ptr(&ct2d->device_mem);
            if (mem_ptr) {
                memcpy(mem_ptr + cache_line_addr, line->data, 64);
            }
        }

        if (ct2d->bi_enabled) {
            cxl_type2_memsim_request(ct2d, CXL_OP_BI_WRITEBACK, cache_line_addr, 64, line->data, NULL);
        } else {
            cxl_type2_memsim_request(ct2d, CXL_OP_WRITE, cache_line_addr, 64, line->data, NULL);
        }

        line->dirty = false;
        ct2d->coherency.coherency_ops++;

        qemu_log_mask(LOG_TRACE, "CXL Type2: Cache writeback at 0x%lx\n", cache_line_addr);
    }
    qemu_mutex_unlock(&ct2d->coherency.lock);
}

static bool cxl_type2_cache_fill_line(CXLType2State *ct2d, uint64_t cache_line_addr, uint8_t cache_data[64]) {
    HetGPUState *hetgpu = &ct2d->gpu_info.hetgpu_state;
    uint8_t *mem_ptr;

    memset(cache_data, 0, 64);

    if (hetgpu->initialized) {
        HetGPUError err;

        err = hetgpu_memcpy_dtoh(hetgpu, cache_data, cache_line_addr, 64);
        if (err == HETGPU_SUCCESS) {
            return true;
        }
    }

    mem_ptr = memory_region_get_ram_ptr(&ct2d->device_mem);
    if (mem_ptr && cache_line_addr < ct2d->device_mem_size) {
        size_t copy_size = MIN(64, ct2d->device_mem_size - cache_line_addr);

        memcpy(cache_data, mem_ptr + cache_line_addr, copy_size);
        return true;
    }

    return false;
}

static void cxl_type2_cache_prefetch(CXLType2State *ct2d, uint64_t addr, uint64_t size, bool write_intent) {
    uint64_t start;
    uint64_t last;

    if (!ct2d->coherency.coherency_enabled || size == 0) {
        return;
    }

    start = addr & ~0x3FULL;
    if (addr + size - 1 < addr) {
        last = UINT64_MAX & ~0x3FULL;
    } else {
        last = (addr + size - 1) & ~0x3FULL;
    }

    for (uint64_t line_addr = start;; line_addr += 64) {
        CXLCacheLine *line = cxl_type2_cache_lookup(ct2d, line_addr);

        if (line && line->state != CXL_COHERENCY_INVALID) {
            if (write_intent) {
                qemu_mutex_lock(&ct2d->coherency.lock);
                line->state = CXL_COHERENCY_EXCLUSIVE;
                line->timestamp = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                qemu_mutex_unlock(&ct2d->coherency.lock);
            }
        } else {
            uint8_t cache_data[64];
            CXLCoherencyState state = write_intent ? CXL_COHERENCY_EXCLUSIVE : CXL_COHERENCY_SHARED;

            if (cxl_type2_cache_fill_line(ct2d, line_addr, cache_data)) {
                cxl_type2_cache_insert(ct2d, line_addr, cache_data, state);
                cxl_type2_memsim_request(ct2d, CXL_OP_READ, line_addr, 64, NULL, NULL);
                ct2d->stats.read_ops++;
            }
        }

        if (line_addr == last || line_addr > UINT64_MAX - 64) {
            break;
        }
    }

    ct2d->coherency.coherency_ops++;

    qemu_log_mask(LOG_TRACE, "CXL Type2: Cache prefetch addr=0x%" PRIx64 " size=0x%" PRIx64 " write_intent=%d\n", addr,
                  size, write_intent);
}

bool cxl_type2_snoop_request(CXLType2State *ct2d, uint64_t addr, bool invalidate) {
    uint64_t cache_line_addr = addr & ~0x3F;
    CXLCacheLine *line;
    bool hit = false;

    qemu_mutex_lock(&ct2d->coherency.lock);
    ct2d->coherency.snoops++;

    line = g_hash_table_lookup(ct2d->coherency.cache_lines, &cache_line_addr);
    if (line) {
        hit = true;

        /* If dirty, write back before invalidation */
        if (line->dirty) {
            qemu_mutex_unlock(&ct2d->coherency.lock);
            cxl_type2_cache_writeback(ct2d, addr);
            qemu_mutex_lock(&ct2d->coherency.lock);
            line = g_hash_table_lookup(ct2d->coherency.cache_lines, &cache_line_addr);
        }

        if (invalidate && line) {
            g_hash_table_remove(ct2d->coherency.cache_lines, &cache_line_addr);
        } else if (line) {
            /* Downgrade to shared */
            line->state = CXL_COHERENCY_SHARED;
        }
    }

    qemu_mutex_unlock(&ct2d->coherency.lock);

    qemu_log_mask(LOG_TRACE, "CXL Type2: Snoop request at 0x%lx, hit=%d, invalidate=%d\n", cache_line_addr, hit,
                  invalidate);

    return hit;
}

/* ========================================================================
 * GPU Passthrough Implementation - VFIO Helpers
 * ======================================================================== */

/* Open and setup VFIO container */
static int cxl_type2_vfio_container_init(CXLType2State *ct2d, Error **errp) {
    int container_fd, version;

    /* Open VFIO container */
    container_fd = open("/dev/vfio/vfio", O_RDWR);
    if (container_fd < 0) {
        error_setg(errp, "Failed to open /dev/vfio/vfio: %s", strerror(errno));
        return -1;
    }

    /* Check VFIO API version */
    version = ioctl(container_fd, VFIO_GET_API_VERSION);
    if (version != VFIO_API_VERSION) {
        error_setg(errp, "VFIO API version mismatch: expected %d, got %d", VFIO_API_VERSION, version);
        close(container_fd);
        return -1;
    }

    /* Check VFIO extension support */
    if (!ioctl(container_fd, VFIO_CHECK_EXTENSION, VFIO_TYPE1_IOMMU) &&
        !ioctl(container_fd, VFIO_CHECK_EXTENSION, VFIO_TYPE1v2_IOMMU)) {
        error_setg(errp, "VFIO does not support TYPE1 or TYPE1v2 IOMMU");
        close(container_fd);
        return -1;
    }

    ct2d->gpu_info.vfio_container = GINT_TO_POINTER(container_fd);
    qemu_log("CXL Type2: VFIO container initialized (fd=%d)\n", container_fd);

    return container_fd;
}

/* Setup VFIO group for GPU device */
static int cxl_type2_vfio_group_init(CXLType2State *ct2d, const char *pci_addr, Error **errp) {
    char group_path[256];
    char iommu_group_path[512];
    char *group_name;
    int group_fd, container_fd;
    ssize_t len;
    struct vfio_group_status group_status = {.argsz = sizeof(group_status)};

    /* Get IOMMU group for the device */
    snprintf(iommu_group_path, sizeof(iommu_group_path), "/sys/bus/pci/devices/%s/iommu_group", pci_addr);

    len = readlink(iommu_group_path, group_path, sizeof(group_path) - 1);
    if (len < 0) {
        error_setg(errp, "Failed to read IOMMU group for %s: %s", pci_addr, strerror(errno));
        return -1;
    }
    group_path[len] = '\0';

    /* Extract group number from path */
    group_name = basename(group_path);

    /* Open VFIO group */
    snprintf(group_path, sizeof(group_path), "/dev/vfio/%s", group_name);
    group_fd = open(group_path, O_RDWR);
    if (group_fd < 0) {
        error_setg(errp, "Failed to open VFIO group %s: %s", group_path, strerror(errno));
        return -1;
    }

    /* Check group is viable */
    if (ioctl(group_fd, VFIO_GROUP_GET_STATUS, &group_status) < 0) {
        error_setg(errp, "Failed to get VFIO group status: %s", strerror(errno));
        close(group_fd);
        return -1;
    }

    if (!(group_status.flags & VFIO_GROUP_FLAGS_VIABLE)) {
        error_setg(errp, "VFIO group is not viable (not all devices bound to VFIO)");
        close(group_fd);
        return -1;
    }

    /* Add group to container */
    container_fd = GPOINTER_TO_INT(ct2d->gpu_info.vfio_container);
    if (ioctl(group_fd, VFIO_GROUP_SET_CONTAINER, &container_fd) < 0) {
        error_setg(errp, "Failed to add group to container: %s", strerror(errno));
        close(group_fd);
        return -1;
    }

    /* Enable IOMMU for the container */
    if (ioctl(container_fd, VFIO_SET_IOMMU, VFIO_TYPE1v2_IOMMU) < 0) {
        if (ioctl(container_fd, VFIO_SET_IOMMU, VFIO_TYPE1_IOMMU) < 0) {
            error_setg(errp, "Failed to set IOMMU type: %s", strerror(errno));
            close(group_fd);
            return -1;
        }
    }

    ct2d->gpu_info.vfio_group = GINT_TO_POINTER(group_fd);
    qemu_log("CXL Type2: VFIO group %s initialized (fd=%d)\n", group_name, group_fd);

    return group_fd;
}

/* Open and setup VFIO device */
static int cxl_type2_vfio_device_init(CXLType2State *ct2d, const char *pci_addr, Error **errp) {
    int group_fd, device_fd;
    struct vfio_device_info device_info = {.argsz = sizeof(device_info)};
    struct vfio_region_info region_info = {.argsz = sizeof(region_info)};

    group_fd = GPOINTER_TO_INT(ct2d->gpu_info.vfio_group);

    /* Get device FD */
    device_fd = ioctl(group_fd, VFIO_GROUP_GET_DEVICE_FD, pci_addr);
    if (device_fd < 0) {
        error_setg(errp, "Failed to get device FD for %s: %s", pci_addr, strerror(errno));
        return -1;
    }

    /* Get device info */
    if (ioctl(device_fd, VFIO_DEVICE_GET_INFO, &device_info) < 0) {
        error_setg(errp, "Failed to get device info: %s", strerror(errno));
        close(device_fd);
        return -1;
    }

    qemu_log("CXL Type2: GPU device %s has %d regions, %d IRQs\n", pci_addr, device_info.num_regions,
             device_info.num_irqs);

    /* Get BAR0 region info (GPU memory typically in BAR0 or BAR1) */
    region_info.index = VFIO_PCI_BAR0_REGION_INDEX;
    if (ioctl(device_fd, VFIO_DEVICE_GET_REGION_INFO, &region_info) < 0) {
        error_setg(errp, "Failed to get BAR0 region info: %s", strerror(errno));
        close(device_fd);
        return -1;
    }

    /* Map GPU memory region */
    if (region_info.size > 0) {
        void *bar_mem = mmap(NULL, region_info.size, PROT_READ | PROT_WRITE, MAP_SHARED, device_fd, region_info.offset);

        if (bar_mem == MAP_FAILED) {
            error_setg(errp, "Failed to mmap GPU BAR0: %s", strerror(errno));
            close(device_fd);
            return -1;
        }

        ct2d->gpu_info.gpu_mem_base = region_info.offset;
        ct2d->gpu_info.gpu_mem_size = region_info.size;

        qemu_log("CXL Type2: Mapped GPU BAR0 at offset 0x%llx, size %llu MB\n", region_info.offset,
                 region_info.size / (1024 * 1024));
    }

    ct2d->gpu_info.vfio_device_fd = device_fd;

    return device_fd;
}

/* Setup DMA mapping for GPU coherent access */
static int cxl_type2_vfio_dma_map(CXLType2State *ct2d, Error **errp) {
    int container_fd;
    struct vfio_iommu_type1_dma_map dma_map = {
        .argsz = sizeof(dma_map),
        .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        .vaddr = 0, /* Will be set to device memory */
        .iova = 0, /* IO virtual address */
        .size = 0, /* Will be set to device memory size */
    };

    container_fd = GPOINTER_TO_INT(ct2d->gpu_info.vfio_container);

    /* Get device memory pointer */
    uint8_t *mem_ptr = memory_region_get_ram_ptr(&ct2d->device_mem);
    if (!mem_ptr) {
        error_setg(errp, "Failed to get device memory pointer");
        return -1;
    }

    dma_map.vaddr = (uint64_t)mem_ptr;
    dma_map.iova = 0; /* Map at IOVA 0 */
    dma_map.size = ct2d->device_mem_size;

    /* Map the device memory for DMA */
    if (ioctl(container_fd, VFIO_IOMMU_MAP_DMA, &dma_map) < 0) {
        error_setg(errp, "Failed to map DMA region: %s", strerror(errno));
        return -1;
    }

    qemu_log("CXL Type2: DMA mapping configured - IOVA: 0x%llx, Size: %llu MB\n", dma_map.iova,
             dma_map.size / (1024 * 1024));

    return 0;
}

/* Interrupt handling thread for GPU events */
static void *cxl_type2_irq_thread(void *opaque) {
    CXLType2State *ct2d = opaque;
    PCIDevice *pci_dev = PCI_DEVICE(ct2d);
    int device_fd = ct2d->gpu_info.vfio_device_fd;
    struct vfio_irq_info irq_info = {.argsz = sizeof(irq_info)};
    int *event_fds = NULL;
    int num_irqs = 0;
    int ret, i;

    /* Get IRQ info for MSI-X */
    irq_info.index = VFIO_PCI_MSIX_IRQ_INDEX;
    if (ioctl(device_fd, VFIO_DEVICE_GET_IRQ_INFO, &irq_info) < 0) {
        qemu_log("CXL Type2: Failed to get MSI-X IRQ info: %s\n", strerror(errno));
        return NULL;
    }

    num_irqs = irq_info.count;
    if (num_irqs == 0) {
        qemu_log("CXL Type2: No MSI-X interrupts available\n");
        return NULL;
    }

    qemu_log("CXL Type2: Setting up %d MSI-X interrupts\n", num_irqs);

    /* Create event FDs for each interrupt */
    event_fds = g_new0(int, num_irqs);
    for (i = 0; i < num_irqs; i++) {
        event_fds[i] = eventfd(0, EFD_NONBLOCK);
        if (event_fds[i] < 0) {
            qemu_log("CXL Type2: Failed to create event FD %d: %s\n", i, strerror(errno));
            goto cleanup;
        }
    }

    /* Set up VFIO IRQ forwarding */
    struct vfio_irq_set *irq_set;
    size_t irq_set_size = sizeof(*irq_set) + num_irqs * sizeof(int);
    irq_set = g_malloc0(irq_set_size);
    irq_set->argsz = irq_set_size;
    irq_set->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
    irq_set->index = VFIO_PCI_MSIX_IRQ_INDEX;
    irq_set->start = 0;
    irq_set->count = num_irqs;
    memcpy(irq_set->data, event_fds, num_irqs * sizeof(int));

    if (ioctl(device_fd, VFIO_DEVICE_SET_IRQS, irq_set) < 0) {
        qemu_log("CXL Type2: Failed to set IRQs: %s\n", strerror(errno));
        g_free(irq_set);
        goto cleanup;
    }

    g_free(irq_set);
    qemu_log("CXL Type2: MSI-X interrupt forwarding configured\n");

    /* Monitor interrupts and forward to guest */
    fd_set rfds;
    int max_fd = 0;
    for (i = 0; i < num_irqs; i++) {
        if (event_fds[i] > max_fd) {
            max_fd = event_fds[i];
        }
    }

    while (ct2d->gpu_info.passthrough_enabled) {
        FD_ZERO(&rfds);
        for (i = 0; i < num_irqs; i++) {
            FD_SET(event_fds[i], &rfds);
        }

        struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
        ret = select(max_fd + 1, &rfds, NULL, NULL, &tv);

        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            qemu_log("CXL Type2: IRQ select failed: %s\n", strerror(errno));
            break;
        }

        if (ret == 0) {
            /* Timeout, check if still enabled */
            continue;
        }

        /* Check which interrupt fired */
        for (i = 0; i < num_irqs; i++) {
            if (FD_ISSET(event_fds[i], &rfds)) {
                uint64_t count;
                ssize_t s = read(event_fds[i], &count, sizeof(count));
                if (s == sizeof(count)) {
                    qemu_log_mask(LOG_TRACE, "CXL Type2: GPU interrupt %d triggered\n", i);

                    /* Notify guest via MSI-X */
                    if (msix_enabled(pci_dev)) {
                        msix_notify(pci_dev, i);
                    }

                    /* Update statistics */
                    ct2d->stats.gpu_accesses++;
                }
            }
        }
    }

cleanup:
    /* Clean up event FDs */
    for (i = 0; i < num_irqs; i++) {
        if (event_fds[i] >= 0) {
            close(event_fds[i]);
        }
    }
    g_free(event_fds);

    qemu_log("CXL Type2: IRQ thread exiting\n");
    return NULL;
}

/* ========================================================================
 * hetGPU Backend Implementation
 * ======================================================================== */

/* Coherency callback for hetGPU operations */
static void cxl_type2_hetgpu_coherency_callback(void *opaque, uint64_t addr, uint64_t size, bool invalidate) {
    CXLType2State *ct2d = opaque;

    if (!ct2d || !ct2d->coherency.coherency_enabled) {
        return;
    }

    /* Notify enhanced BAR coherency layer of GPU access */
    if (ct2d->bar_coherency.enabled) {
        cxl_bar_notify_gpu_access(&ct2d->bar_coherency, addr, size, invalidate);
    }

    if (invalidate) {
        /* GPU is writing - invalidate CPU cache lines */
        if (addr && size) {
            uint64_t aligned_addr = addr & ~0x3F;
            uint64_t end_addr = (addr + size + 63) & ~0x3F;
            for (uint64_t a = aligned_addr; a < end_addr; a += 64) {
                cxl_type2_cache_invalidate(ct2d, a);
            }
        }
    } else {
        /* GPU is reading - write back CPU cache lines */
        if (addr && size) {
            uint64_t aligned_addr = addr & ~0x3F;
            uint64_t end_addr = (addr + size + 63) & ~0x3F;
            for (uint64_t a = aligned_addr; a < end_addr; a += 64) {
                cxl_type2_cache_writeback(ct2d, a);
            }
        }
    }

    ct2d->coherency.coherency_ops++;
}

int cxl_type2_hetgpu_init(CXLType2State *ct2d, Error **errp) {
    HetGPUState *hetgpu = &ct2d->gpu_info.hetgpu_state;
    HetGPUError err;
    const char *lib_path;

    fprintf(stderr, "cxl_type2_hetgpu_init: ENTERED\n");
    fflush(stderr);

    /* Determine library path - try multiple locations */
    lib_path = ct2d->gpu_info.hetgpu_lib_path;
    fprintf(stderr, "cxl_type2_hetgpu_init: hetgpu_lib_path = '%s'\n", lib_path ? lib_path : "(null)");
    fflush(stderr);
    if (!lib_path || lib_path[0] == '\0') {
        lib_path = getenv("HETGPU_LIB_PATH");
        fprintf(stderr, "cxl_type2_hetgpu_init: env HETGPU_LIB_PATH = '%s'\n", lib_path ? lib_path : "(null)");
        fflush(stderr);
    }
    if (!lib_path || lib_path[0] == '\0') {
        /* Try system CUDA library first for real GPU passthrough */
        lib_path = DEFAULT_HETGPU_LIB_PATH;
    }
    if (!lib_path || lib_path[0] == '\0') {
        lib_path = "/usr/lib/x86_64-linux-gnu/libcuda.so";
    }

    fprintf(stderr, "cxl_type2_hetgpu_init: Using library: %s\n", lib_path);
    fprintf(stderr, "cxl_type2_hetgpu_init: backend=%d, device_index=%d\n", ct2d->gpu_info.hetgpu_backend,
            ct2d->gpu_info.hetgpu_device_index);
    fflush(stderr);

    /* Initialize hetGPU */
    err = hetgpu_init(hetgpu, ct2d->gpu_info.hetgpu_backend, ct2d->gpu_info.hetgpu_device_index, lib_path);
    if (err != HETGPU_SUCCESS) {
        error_setg(errp, "hetGPU initialization failed: %s", hetgpu_get_error_string(err));
        return -1;
    }

    /* Create context */
    err = hetgpu_create_context(hetgpu);
    if (err != HETGPU_SUCCESS) {
        error_setg(errp, "hetGPU context creation failed: %s", hetgpu_get_error_string(err));
        hetgpu_cleanup(hetgpu);
        return -1;
    }

    /* Set up coherency callback */
    hetgpu_set_coherency_callback(hetgpu, cxl_type2_hetgpu_coherency_callback, ct2d);

    ct2d->gpu_info.passthrough_enabled = true;
    ct2d->gpu_info.gpu_mem_size = hetgpu->props.total_memory;

    qemu_log("CXL Type2: hetGPU initialized - Backend: %s, Device: %s\n", hetgpu_get_backend_name(hetgpu->backend),
             hetgpu->props.name);
    qemu_log("CXL Type2: GPU Memory: %lu MB, Compute: %d.%d\n", hetgpu->props.total_memory / (1024 * 1024),
             hetgpu->props.compute_capability_major, hetgpu->props.compute_capability_minor);

    return 0;
}

void cxl_type2_hetgpu_cleanup(CXLType2State *ct2d) {
    HetGPUState *hetgpu = &ct2d->gpu_info.hetgpu_state;
    HetGPUError err;

    if (!hetgpu->initialized) {
        return;
    }

    err = hetgpu_unregister_coherent_region(
        hetgpu, &ct2d->coherent_pool_gpu_region);
    if (err != HETGPU_SUCCESS) {
        qemu_log("CXL Type2: Coherent pool unregister failed; "
                 "synchronizing and retrying: %s\n",
                 hetgpu_get_error_string(err));
        hetgpu_synchronize(hetgpu);
        err = hetgpu_unregister_coherent_region(
            hetgpu, &ct2d->coherent_pool_gpu_region);
        if (err != HETGPU_SUCCESS) {
            error_report("CXL Type2: refusing to release CUDA-registered "
                         "coherent pool after unregister failure: %s",
                         hetgpu_get_error_string(err));
            abort();
        }
    }
    qemu_log("CXL Type2: Cleaning up hetGPU backend\n");
    hetgpu_cleanup(hetgpu);
}

int cxl_type2_hetgpu_load_ptx(CXLType2State *ct2d, const char *ptx_source, void **module) {
    HetGPUState *hetgpu = &ct2d->gpu_info.hetgpu_state;
    HetGPUError err;

    if (!hetgpu->initialized) {
        return -1;
    }

    err = hetgpu_load_ptx(hetgpu, ptx_source, (HetGPUModule *)module);
    if (err != HETGPU_SUCCESS) {
        qemu_log("CXL Type2: Failed to load PTX: %s\n", hetgpu_get_error_string(err));
        return -1;
    }

    return 0;
}

int cxl_type2_hetgpu_launch_kernel(CXLType2State *ct2d, void *function, uint32_t grid_x, uint32_t grid_y,
                                   uint32_t grid_z, uint32_t block_x, uint32_t block_y, uint32_t block_z,
                                   uint32_t shared_mem, void **args, size_t num_args) {
    HetGPUState *hetgpu = &ct2d->gpu_info.hetgpu_state;
    HetGPULaunchConfig config;
    HetGPUError err;

    if (!hetgpu->initialized) {
        return -1;
    }

    config.grid_dim[0] = grid_x;
    config.grid_dim[1] = grid_y;
    config.grid_dim[2] = grid_z;
    config.block_dim[0] = block_x;
    config.block_dim[1] = block_y;
    config.block_dim[2] = block_z;
    config.shared_mem_bytes = shared_mem;
    config.stream = NULL;

    err = hetgpu_launch_kernel(hetgpu, function, &config, args, num_args);
    if (err != HETGPU_SUCCESS) {
        qemu_log("CXL Type2: Kernel launch failed: %s\n", hetgpu_get_error_string(err));
        return -1;
    }

    ct2d->stats.gpu_accesses++;
    return 0;
}

int cxl_type2_hetgpu_malloc(CXLType2State *ct2d, size_t size, uint64_t *dev_ptr) {
    HetGPUState *hetgpu = &ct2d->gpu_info.hetgpu_state;
    HetGPUError err;

    if (!hetgpu->initialized || !dev_ptr) {
        return -1;
    }

    err = hetgpu_malloc(hetgpu, size, HETGPU_MEM_HOST_MAPPED, (HetGPUDevicePtr *)dev_ptr);
    if (err != HETGPU_SUCCESS) {
        qemu_log("CXL Type2: Memory allocation failed: %s\n", hetgpu_get_error_string(err));
        return -1;
    }

    return 0;
}

int cxl_type2_hetgpu_free(CXLType2State *ct2d, uint64_t dev_ptr) {
    HetGPUState *hetgpu = &ct2d->gpu_info.hetgpu_state;
    HetGPUError err;

    if (!hetgpu->initialized) {
        return -1;
    }

    err = hetgpu_free(hetgpu, dev_ptr);
    if (err != HETGPU_SUCCESS) {
        qemu_log("CXL Type2: Memory free failed: %s\n", hetgpu_get_error_string(err));
        return -1;
    }

    return 0;
}

int cxl_type2_hetgpu_memcpy_htod(CXLType2State *ct2d, uint64_t dst, const void *src, size_t size) {
    HetGPUState *hetgpu = &ct2d->gpu_info.hetgpu_state;
    HetGPUError err;

    if (!hetgpu->initialized) {
        return -1;
    }

    /* Invalidate cache before GPU write */
    cxl_type2_cache_invalidate(ct2d, dst);

    err = hetgpu_memcpy_htod(hetgpu, dst, src, size);
    if (err != HETGPU_SUCCESS) {
        qemu_log("CXL Type2: HtoD memcpy failed: %s\n", hetgpu_get_error_string(err));
        return -1;
    }

    ct2d->stats.gpu_accesses++;
    return 0;
}

int cxl_type2_hetgpu_memcpy_dtoh(CXLType2State *ct2d, void *dst, uint64_t src, size_t size) {
    HetGPUState *hetgpu = &ct2d->gpu_info.hetgpu_state;
    HetGPUError err;

    if (!hetgpu->initialized) {
        return -1;
    }

    /* Writeback cache before GPU read */
    cxl_type2_cache_writeback(ct2d, src);

    err = hetgpu_memcpy_dtoh(hetgpu, dst, src, size);
    if (err != HETGPU_SUCCESS) {
        qemu_log("CXL Type2: DtoH memcpy failed: %s\n", hetgpu_get_error_string(err));
        return -1;
    }

    ct2d->stats.gpu_accesses++;
    return 0;
}

int cxl_type2_hetgpu_sync(CXLType2State *ct2d) {
    HetGPUState *hetgpu = &ct2d->gpu_info.hetgpu_state;
    HetGPUError err;

    if (!hetgpu->initialized) {
        return -1;
    }

    err = hetgpu_synchronize(hetgpu);
    if (err != HETGPU_SUCCESS) {
        qemu_log("CXL Type2: Sync failed: %s\n", hetgpu_get_error_string(err));
        return -1;
    }

    return 0;
}

/* ========================================================================
 * GPU Passthrough Implementation - Main Functions
 * ======================================================================== */

int cxl_type2_gpu_init(CXLType2State *ct2d, Error **errp) {
    Error *local_err = NULL;
    bool auto_mode = ct2d->gpu_info.mode == CXL_TYPE2_GPU_MODE_AUTO;
    int ret;

    /* Determine GPU mode if auto */
    if (ct2d->gpu_info.mode == CXL_TYPE2_GPU_MODE_AUTO) {
        if (ct2d->gpu_info.vfio_device && ct2d->gpu_info.vfio_device[0]) {
            ct2d->gpu_info.mode = CXL_TYPE2_GPU_MODE_VFIO;
        } else {
            /* Default to hetGPU mode - will use system libcuda.so for real GPU */
            ct2d->gpu_info.mode = CXL_TYPE2_GPU_MODE_HETGPU;
        }
    }

    /* Initialize based on mode */
    fprintf(stderr, "CXL Type2: GPU mode = %d (NONE=0, VFIO=1, HETGPU=2, AUTO=3)\n", ct2d->gpu_info.mode);
    fflush(stderr);
    switch (ct2d->gpu_info.mode) {
    case CXL_TYPE2_GPU_MODE_HETGPU:
        fprintf(stderr, "CXL Type2: Initializing hetGPU backend...\n");
        fflush(stderr);
        ret = cxl_type2_hetgpu_init(ct2d, &local_err);
        fprintf(stderr, "CXL Type2: cxl_type2_hetgpu_init returned %d\n", ret);
        fflush(stderr);
        if (ret == 0) {
            fprintf(stderr, "CXL Type2: hetGPU backend initialized successfully\n");
            fflush(stderr);
            return 0;
        }
        if (!auto_mode) {
            error_propagate(errp, local_err);
            return -1;
        }
        error_free(local_err);
        local_err = NULL;
        /* Fall through to VFIO or simulation if hetGPU fails */
        fprintf(stderr, "CXL Type2: hetGPU init failed, trying fallback\n");
        fflush(stderr);
        /* fall through */

    case CXL_TYPE2_GPU_MODE_VFIO:
        if (!ct2d->gpu_info.vfio_device || !ct2d->gpu_info.vfio_device[0]) {
            qemu_log("CXL Type2: No VFIO device specified\n");
            ct2d->gpu_info.mode = CXL_TYPE2_GPU_MODE_NONE;
            /* fall through */
        } else {
            qemu_log("CXL Type2: Initializing GPU passthrough for device %s\n", ct2d->gpu_info.vfio_device);
            break; /* Continue with VFIO init below */
        }
        /* fall through */

    case CXL_TYPE2_GPU_MODE_NONE:
    default:
        qemu_log("CXL Type2: GPU passthrough not configured (simulation mode)\n");
        ct2d->gpu_info.gpu_mem_base = 0;
        ct2d->gpu_info.gpu_mem_size = ct2d->device_mem_size;
        ct2d->gpu_info.passthrough_enabled = false;
        return 0;
    }

    /* VFIO passthrough initialization continues here */
    qemu_log("CXL Type2: Initializing VFIO passthrough for device %s\n", ct2d->gpu_info.vfio_device);

    /* Step 1: Open VFIO container */
    ret = cxl_type2_vfio_container_init(ct2d, &local_err);
    if (ret < 0) {
        error_propagate(errp, local_err);
        return -1;
    }

    /* Step 2: Setup VFIO group */
    ret = cxl_type2_vfio_group_init(ct2d, ct2d->gpu_info.vfio_device, &local_err);
    if (ret < 0) {
        error_propagate(errp, local_err);
        goto err_close_container;
    }

    /* Step 3: Get device FD and map BARs */
    ret = cxl_type2_vfio_device_init(ct2d, ct2d->gpu_info.vfio_device, &local_err);
    if (ret < 0) {
        error_propagate(errp, local_err);
        goto err_close_group;
    }

    /* Step 4: Setup DMA mapping for coherent access */
    ret = cxl_type2_vfio_dma_map(ct2d, &local_err);
    if (ret < 0) {
        error_propagate(errp, local_err);
        goto err_close_device;
    }

    ct2d->gpu_info.passthrough_enabled = true;

    /* Step 5: Start IRQ forwarding thread */
    ct2d->gpu_info.irq_thread_running = true;
    qemu_thread_create(&ct2d->gpu_info.irq_thread, "cxl-type2-irq", cxl_type2_irq_thread, ct2d, QEMU_THREAD_JOINABLE);

    qemu_log("CXL Type2: GPU passthrough successfully initialized\n");
    qemu_log("CXL Type2: GPU memory: base=0x%lx, size=%lu MB\n", ct2d->gpu_info.gpu_mem_base,
             ct2d->gpu_info.gpu_mem_size / (1024 * 1024));

    return 0;

err_close_device:
    if (ct2d->gpu_info.vfio_device_fd >= 0) {
        close(ct2d->gpu_info.vfio_device_fd);
        ct2d->gpu_info.vfio_device_fd = -1;
    }

err_close_group:
    if (ct2d->gpu_info.vfio_group) {
        close(GPOINTER_TO_INT(ct2d->gpu_info.vfio_group));
        ct2d->gpu_info.vfio_group = NULL;
    }

err_close_container:
    if (ct2d->gpu_info.vfio_container) {
        close(GPOINTER_TO_INT(ct2d->gpu_info.vfio_container));
        ct2d->gpu_info.vfio_container = NULL;
    }

    ct2d->gpu_info.passthrough_enabled = false;
    return -1;
}

void cxl_type2_gpu_cleanup(CXLType2State *ct2d) {
    if (!ct2d->gpu_info.passthrough_enabled) {
        return;
    }

    /* Handle hetGPU cleanup */
    if (ct2d->gpu_info.mode == CXL_TYPE2_GPU_MODE_HETGPU) {
        cxl_type2_hetgpu_cleanup(ct2d);
        ct2d->gpu_info.passthrough_enabled = false;
        return;
    }

    /* VFIO cleanup */
    /* Stop IRQ forwarding thread */
    if (ct2d->gpu_info.irq_thread_running) {
        ct2d->gpu_info.irq_thread_running = false;
        ct2d->gpu_info.passthrough_enabled = false; /* Signal thread to exit */
        qemu_thread_join(&ct2d->gpu_info.irq_thread);
        qemu_log("CXL Type2: IRQ thread stopped\n");
    }

    /* Unmap DMA regions */
    if (ct2d->gpu_info.vfio_container) {
        int container_fd = GPOINTER_TO_INT(ct2d->gpu_info.vfio_container);
        struct vfio_iommu_type1_dma_unmap dma_unmap = {
            .argsz = sizeof(dma_unmap),
            .flags = 0,
            .iova = 0,
            .size = ct2d->device_mem_size,
        };

        if (ioctl(container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap) < 0) {
            qemu_log("CXL Type2: Warning - Failed to unmap DMA: %s\n", strerror(errno));
        }
    }

    /* Disable VFIO interrupts */
    if (ct2d->gpu_info.vfio_device_fd >= 0) {
        struct vfio_irq_set irq_set = {
            .argsz = sizeof(irq_set),
            .flags = VFIO_IRQ_SET_DATA_NONE | VFIO_IRQ_SET_ACTION_TRIGGER,
            .index = VFIO_PCI_MSIX_IRQ_INDEX,
            .start = 0,
            .count = 0,
        };
        ioctl(ct2d->gpu_info.vfio_device_fd, VFIO_DEVICE_SET_IRQS, &irq_set);
    }

    /* Close VFIO device */
    if (ct2d->gpu_info.vfio_device_fd >= 0) {
        close(ct2d->gpu_info.vfio_device_fd);
        ct2d->gpu_info.vfio_device_fd = -1;
    }

    /* Close VFIO group */
    if (ct2d->gpu_info.vfio_group) {
        int group_fd = GPOINTER_TO_INT(ct2d->gpu_info.vfio_group);
        close(group_fd);
        ct2d->gpu_info.vfio_group = NULL;
    }

    /* Close VFIO container */
    if (ct2d->gpu_info.vfio_container) {
        int container_fd = GPOINTER_TO_INT(ct2d->gpu_info.vfio_container);
        close(container_fd);
        ct2d->gpu_info.vfio_container = NULL;
    }

    qemu_log("CXL Type2: GPU passthrough cleanup complete\n");
}

int cxl_type2_gpu_read(CXLType2State *ct2d, uint64_t offset, void *buf, size_t size) {
    ssize_t ret;

    if (!ct2d->gpu_info.passthrough_enabled) {
        return -1;
    }

    /* If VFIO device is configured, read from actual GPU BAR */
    if (ct2d->gpu_info.vfio_device_fd >= 0) {
        uint64_t bar_offset = ct2d->gpu_info.gpu_mem_base + offset;

        ret = pread(ct2d->gpu_info.vfio_device_fd, buf, size, bar_offset);
        if (ret < 0) {
            qemu_log("CXL Type2: GPU read failed at offset 0x%lx: %s\n", offset, strerror(errno));
            return -1;
        }

        if (ret != size) {
            qemu_log("CXL Type2: GPU read incomplete at offset 0x%lx: %zd/%zu bytes\n", offset, ret, size);
            return -1;
        }

        ct2d->stats.gpu_accesses++;
        return 0;
    }

    /* Fallback to device memory for simulation mode */
    if (offset + size <= ct2d->device_mem_size) {
        uint8_t *mem_ptr = memory_region_get_ram_ptr(&ct2d->device_mem);
        if (mem_ptr) {
            memcpy(buf, mem_ptr + offset, size);
            ct2d->stats.gpu_accesses++;
            return 0;
        }
    }

    return -1;
}

int cxl_type2_gpu_write(CXLType2State *ct2d, uint64_t offset, const void *buf, size_t size) {
    ssize_t ret;

    if (!ct2d->gpu_info.passthrough_enabled) {
        return -1;
    }

    /* If VFIO device is configured, write to actual GPU BAR */
    if (ct2d->gpu_info.vfio_device_fd >= 0) {
        uint64_t bar_offset = ct2d->gpu_info.gpu_mem_base + offset;

        ret = pwrite(ct2d->gpu_info.vfio_device_fd, buf, size, bar_offset);
        if (ret < 0) {
            qemu_log("CXL Type2: GPU write failed at offset 0x%lx: %s\n", offset, strerror(errno));
            return -1;
        }

        if (ret != size) {
            qemu_log("CXL Type2: GPU write incomplete at offset 0x%lx: %zd/%zu bytes\n", offset, ret, size);
            return -1;
        }

        /* Invalidate cache line if present for coherency */
        cxl_type2_cache_invalidate(ct2d, offset);

        cxl_type2_memsim_request(ct2d, CXL_OP_WRITE, offset, size, buf, NULL);

        ct2d->stats.gpu_accesses++;
        return 0;
    }

    /* Fallback to device memory for simulation mode */
    if (offset + size <= ct2d->device_mem_size) {
        uint8_t *mem_ptr = memory_region_get_ram_ptr(&ct2d->device_mem);
        if (mem_ptr) {
            memcpy(mem_ptr + offset, buf, size);

            /* Invalidate cache line if present */
            cxl_type2_cache_invalidate(ct2d, offset);

            ct2d->stats.gpu_accesses++;
            return 0;
        }
    }

    return -1;
}

/* ========================================================================
 * CXLMemSim Communication
 * ======================================================================== */

static void cxl_type2_close_memsim_socket(CXLType2State *ct2d) {
    ct2d->memsim.connected = false;
    if (ct2d->memsim.socket) {
        qio_channel_close(QIO_CHANNEL(ct2d->memsim.socket), NULL);
        object_unref(OBJECT(ct2d->memsim.socket));
        ct2d->memsim.socket = NULL;
    }
}

static void cxlmemsim_drop_locked(CXLType2State *ct2d) { cxl_type2_close_memsim_socket(ct2d); }

static void cxl_type2_slugarch_close_event_log(CXLType2State *ct2d) {
    if (ct2d->slugarch.event_log) {
        fclose(ct2d->slugarch.event_log);
        ct2d->slugarch.event_log = NULL;
    }
}

static bool cxl_type2_slugarch_write_line(CXLType2State *ct2d, const char *line, Error **errp) {
    if (!ct2d->slugarch.event_log) {
        error_setg(errp, "SlugArch Type-2 event log is not open");
        return false;
    }
    if (fputs(line, ct2d->slugarch.event_log) == EOF || fflush(ct2d->slugarch.event_log) != 0) {
        error_setg_errno(errp, errno, "SlugArch Type-2 event log write failed");
        return false;
    }
    return true;
}

static bool cxl_type2_slugarch_open_event_log(CXLType2State *ct2d, Error **errp) {
    int fd;

    fd = open(ct2d->slugarch.event_log_path, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (fd < 0) {
        error_setg_errno(errp, errno, "cannot create SlugArch Type-2 event log '%s'", ct2d->slugarch.event_log_path);
        return false;
    }
    ct2d->slugarch.event_log = fdopen(fd, "w");
    if (!ct2d->slugarch.event_log) {
        error_setg_errno(errp, errno, "cannot open SlugArch Type-2 event stream '%s'", ct2d->slugarch.event_log_path);
        close(fd);
        return false;
    }
    return true;
}

static char *cxl_type2_slugarch_uuid_hex(const uint8_t uuid[16]) {
    char *hex = g_malloc(33);
    size_t i;

    for (i = 0; i < 16; i++) {
        snprintf(hex + i * 2, 3, "%02x", uuid[i]);
    }
    hex[32] = '\0';
    return hex;
}

static bool cxl_type2_slugarch_emit_handshake(CXLType2State *ct2d, Error **errp) {
    g_autofree char *uuid = cxl_type2_slugarch_uuid_hex(ct2d->slugarch.server_uuid);
    g_autofree char *line = g_strdup_printf("{\"event\":\"handshake\",\"client_id\":%" PRIu64
                                            ",\"server_instance_id\":\"%s\",\"capacity_bytes\":%" PRIu64
                                            ",\"configured_latency_ns\":%" PRIu64 ",\"protocol_version\":%u}\n",
                                            ct2d->slugarch.client_id, uuid, ct2d->slugarch.server_capacity,
                                            ct2d->slugarch.configured_base_latency, ct2d->slugarch.wire_version);

    return cxl_type2_slugarch_write_line(ct2d, line, errp);
}

static bool cxl_type2_slugarch_handshake(CXLType2State *ct2d, Error **errp) {
    const char *transport_mode = getenv("CXL_TRANSPORT_MODE");
    SocketAddress address = {
        .type = SOCKET_ADDRESS_TYPE_INET,
    };
    SlugArchT2Hello hello = {
        .request_id = 1,
        .role = SLUGARCH_T2_ROLE_QEMU,
    };
    SlugArchT2Frame request = {0};
    SlugArchT2Frame response = {0};
    SlugArchT2Ack ack = {0};
    Error *local_err = NULL;
    char *port = NULL;
    uint64_t deadline_ns;
    size_t i;

    if (!transport_mode || !transport_mode[0]) {
        transport_mode = getenv("CXL_MEMSIM_TRANSPORT");
    }
    if (transport_mode &&
        (!strcmp(transport_mode, "shm") || !strcmp(transport_mode, "pgas") || !strcmp(transport_mode, "pgas-shm"))) {
        error_setg(&local_err, "synchronous wire mode requires TCP, not %s", transport_mode);
        goto fail;
    }
    if (ct2d->slugarch.wire_version != SLUGARCH_T2_VERSION) {
        error_setg(&local_err, "wire version %u is not supported", ct2d->slugarch.wire_version);
        goto fail;
    }
    if (!ct2d->slugarch.event_log_path || !g_path_is_absolute(ct2d->slugarch.event_log_path)) {
        error_setg(&local_err, "slugarch-event-log must be a nonempty absolute path");
        goto fail;
    }
    if (ct2d->device_mem_size != 256 * MiB) {
        error_setg(&local_err, "mem-size must be exactly 268435456 bytes");
        goto fail;
    }
    if (ct2d->coherency.coherency_enabled) {
        error_setg(&local_err, "coherency-enabled must be false in synchronous wire mode");
        goto fail;
    }
    if (!ct2d->memsim.server_addr || !ct2d->memsim.server_addr[0] || !ct2d->memsim.server_port) {
        error_setg(&local_err, "CXLMemSim TCP address and port must be nonempty");
        goto fail;
    }
    if (!cxl_type2_slugarch_open_event_log(ct2d, &local_err)) {
        goto fail;
    }

    address.u.inet.host = ct2d->memsim.server_addr;
    port = g_strdup_printf("%u", ct2d->memsim.server_port);
    address.u.inet.port = port;
    ct2d->memsim.socket = qio_channel_socket_new();
    if (qio_channel_socket_connect_sync(ct2d->memsim.socket, &address, &local_err) < 0) {
        goto fail;
    }

    for (i = 0; i < sizeof(hello.client_nonce); i++) {
        hello.client_nonce[i] = i;
    }
    if (!slugarch_t2_encode_hello(&hello, &request)) {
        error_setg(&local_err, "cannot encode HELLO");
        goto fail;
    }
    deadline_ns = slugarch_t2_monotonic_ns() + SLUGARCH_T2_REQUEST_TIMEOUT_NS;
    if (slugarch_t2_exchange_until(ct2d->memsim.socket, &request, deadline_ns, &response, &local_err) !=
        SLUGARCH_T2_IO_OK) {
        goto fail;
    }
    if (!slugarch_t2_decode_ack(&response, &ack, &local_err)) {
        goto fail;
    }
    if (ack.request_id != 1) {
        error_setg(&local_err, "ACK request ID %" PRIu64 " is not 1", ack.request_id);
        goto fail;
    }
    if (!ack.client_id) {
        error_setg(&local_err, "ACK client ID is zero");
        goto fail;
    }
    if (ack.status != SLUGARCH_T2_STATUS_SUCCESS) {
        error_setg(&local_err, "ACK status %u is not success", ack.status);
        goto fail;
    }
    if (ack.maximum_frame != SLUGARCH_T2_MAX_FRAME) {
        error_setg(&local_err, "ACK maximum frame %u is not 128", ack.maximum_frame);
        goto fail;
    }
    if (ack.capacity != 256 * MiB) {
        error_setg(&local_err, "ACK capacity %" PRIu64 " is not 268435456", ack.capacity);
        goto fail;
    }
    if (ack.configured_base_latency > 1000000ULL) {
        error_setg(&local_err, "ACK latency %" PRIu64 " exceeds 1000000 ns", ack.configured_base_latency);
        goto fail;
    }

    ct2d->slugarch.client_id = ack.client_id;
    memcpy(ct2d->slugarch.server_uuid, ack.server_uuid, sizeof(ct2d->slugarch.server_uuid));
    ct2d->slugarch.server_capacity = ack.capacity;
    ct2d->slugarch.configured_base_latency = ack.configured_base_latency;
    ct2d->slugarch.next_request_id = 2;
    ct2d->slugarch.connection_failed = false;
    ct2d->memsim.connected = true;
    if (!cxl_type2_slugarch_emit_handshake(ct2d, &local_err)) {
        goto fail;
    }

    g_free(port);
    return true;

fail:
    g_free(port);
    cxl_type2_close_memsim_socket(ct2d);
    cxl_type2_slugarch_close_event_log(ct2d);
    if (!local_err) {
        error_setg(&local_err, "unknown handshake failure");
    }
    error_prepend(&local_err, "SlugArch Type-2 handshake failed: ");
    error_propagate(errp, local_err);
    return false;
}

static void cxlmemsim_connect(CXLType2State *ct2d) {
    Error *err = NULL;
    SocketAddress addr;

    if (ct2d->memsim.connected) {
        return;
    }

    /* Check if using shared memory mode */
    const char *transport_mode = getenv("CXL_TRANSPORT_MODE");
    if (!transport_mode || !transport_mode[0]) {
        transport_mode = getenv("CXL_MEMSIM_TRANSPORT");
    }

    qemu_log("CXL Type2: Transport mode = %s\n", transport_mode ? transport_mode : "(not set)");

    if (transport_mode && (strcmp(transport_mode, "shm") == 0 || strcmp(transport_mode, "pgas") == 0)) {
        ct2d->memsim.use_shm = true;
        qemu_log("CXL Type2: Using shared memory transport - skipping TCP connection\n");
        /* Type3 device handles SHM connection */
        return;
    }

    qemu_log("CXL Type2: Using TCP transport mode\n");

    /* TCP connection to CXLMemSim */
    addr.type = SOCKET_ADDRESS_TYPE_INET;
    addr.u.inet.host = ct2d->memsim.server_addr;
    addr.u.inet.port = g_strdup_printf("%u", ct2d->memsim.server_port);

    ct2d->memsim.socket = qio_channel_socket_new();
    if (qio_channel_socket_connect_sync(ct2d->memsim.socket, &addr, &err) < 0) {
        qemu_log("Warning: Failed to connect to CXLMemSim at %s:%s: %s\n", addr.u.inet.host, addr.u.inet.port,
                 error_get_pretty(err));
        error_free(err);
        g_free(addr.u.inet.port);
        object_unref(OBJECT(ct2d->memsim.socket));
        ct2d->memsim.socket = NULL;
        return;
    }

    ct2d->memsim.connected = true;
    g_free(addr.u.inet.port);

    qemu_log("CXL Type2: Connected to CXLMemSim at %s:%u\n", ct2d->memsim.server_addr, ct2d->memsim.server_port);
}

static void cxlmemsim_disconnect(CXLType2State *ct2d) {
    qemu_mutex_lock(&ct2d->memsim.lock);

    cxlmemsim_drop_locked(ct2d);

    if (ct2d->memsim.use_shm && ct2d->memsim.shm_base) {
        munmap(ct2d->memsim.shm_base, ct2d->memsim.shm_size);
        ct2d->memsim.shm_base = NULL;
    }

    qemu_mutex_unlock(&ct2d->memsim.lock);
}

static void cxl_type2_memsim_v2_disconnect(CXLType2State *ct2d) {
    cxl_memsim_v2_client_free(ct2d->memsim_v2.endpoints.device);
    cxl_memsim_v2_client_free(ct2d->memsim_v2.endpoints.host);
    ct2d->memsim_v2.endpoints.device = NULL;
    ct2d->memsim_v2.endpoints.host = NULL;
}

static bool cxl_type2_memsim_v2_connect(CXLType2State *ct2d, Error **errp) {
    CXLType2MemSimV2State *v2 = &ct2d->memsim_v2;
    Error *local_err = NULL;

    v2->endpoints.host = cxl_memsim_v2_client_new(v2->host_endpoint, NULL, NULL);
    v2->endpoints.device = cxl_memsim_v2_client_new(v2->device_endpoint, NULL, NULL);
    if (!v2->endpoints.host || !v2->endpoints.device) {
        error_setg(&local_err, "cannot allocate CXLMemSim v2 endpoints");
        goto fail;
    }
    if (!cxl_memsim_v2_client_set_write_policy(
            v2->endpoints.host, v2->write_through ? CXL_MEMSIM_V2_WRITE_THROUGH : CXL_MEMSIM_V2_WRITE_BACK,
            &local_err) ||
        !cxl_memsim_v2_client_set_write_policy(
            v2->endpoints.device, v2->write_through ? CXL_MEMSIM_V2_WRITE_THROUGH : CXL_MEMSIM_V2_WRITE_BACK,
            &local_err)) {
        goto fail;
    }
    if (!cxl_memsim_v2_client_connect(v2->endpoints.host, ct2d->memsim.server_addr, ct2d->memsim.server_port,
                                      v2->cache_capacity, v2->cache_ways, v2->timeout_ms, &local_err)) {
        error_prepend(&local_err, "host endpoint %u registration failed: ", v2->host_endpoint);
        goto fail;
    }
    if (!cxl_memsim_v2_client_connect(v2->endpoints.device, ct2d->memsim.server_addr, ct2d->memsim.server_port,
                                      v2->cache_capacity, v2->cache_ways, v2->timeout_ms, &local_err)) {
        error_prepend(&local_err, "device endpoint %u registration failed: ", v2->device_endpoint);
        goto fail;
    }
    qemu_log("CXL Type2: protocol-v2 host endpoint %u session 0x%" PRIx64 ", device endpoint %u session 0x%" PRIx64
             ", policy=%s\n",
             v2->host_endpoint, cxl_memsim_v2_client_session(v2->endpoints.host), v2->device_endpoint,
             cxl_memsim_v2_client_session(v2->endpoints.device), v2->write_through ? "write-through" : "write-back");
    return true;

fail:
    cxl_type2_memsim_v2_disconnect(ct2d);
    error_propagate(errp, local_err);
    return false;
}

static bool cxl_type2_memsim_request_ext(CXLType2State *ct2d, uint8_t op_type, uint64_t addr, uint64_t size,
                                         const uint8_t *data, uint64_t value, uint64_t expected,
                                         CXLMemSimResponse *resp) {
    CXLMemSimRequest req;
    CXLMemSimResponse local_resp;
    Error *err = NULL;
    bool ok = false;

    if (ct2d->memsim_v2.enabled || ct2d->slugarch.enabled || !ct2d->memsim.connected || ct2d->memsim.use_shm) {
        return false;
    }

    memset(&req, 0, sizeof(req));
    memset(&local_resp, 0, sizeof(local_resp));
    req.op_type = op_type;
    req.addr = addr;
    req.size = size;
    req.timestamp = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    req.value = value;
    req.expected = expected;
    if (data) {
        memcpy(req.data, data, MIN(size, sizeof(req.data)));
    }

    qemu_mutex_lock(&ct2d->memsim.lock);
    if (!ct2d->memsim.connected || !ct2d->memsim.socket) {
        goto out;
    }

    if (qio_channel_write_all(QIO_CHANNEL(ct2d->memsim.socket), (const char *)&req, sizeof(req), &err) < 0) {
        error_report("CXL Type2: Failed to send request to CXLMemSim: %s", error_get_pretty(err));
        error_free(err);
        cxlmemsim_drop_locked(ct2d);
        goto out;
    }

    if (qio_channel_read_all(QIO_CHANNEL(ct2d->memsim.socket), (char *)&local_resp, sizeof(local_resp), &err) < 0) {
        error_report("CXL Type2: Failed to receive response from CXLMemSim: %s", error_get_pretty(err));
        error_free(err);
        cxlmemsim_drop_locked(ct2d);
        goto out;
    }

    if (resp) {
        *resp = local_resp;
    }
    ok = local_resp.status == 0;

out:
    qemu_mutex_unlock(&ct2d->memsim.lock);
    return ok;
}

static bool cxl_type2_memsim_request(CXLType2State *ct2d, uint8_t op_type, uint64_t addr, uint64_t size,
                                     const uint8_t *data, CXLMemSimResponse *resp) {
    return cxl_type2_memsim_request_ext(ct2d, op_type, addr, size, data, 0, 0, resp);
}

static char *cxl_type2_slugarch_sha256_hex(const uint8_t *bytes, size_t length, Error **errp) {
    uint8_t *digest = NULL;
    size_t digest_length = 0;
    char *hex;
    size_t i;

    if (qcrypto_hash_bytes(QCRYPTO_HASH_ALGO_SHA256, (const char *)bytes, length, &digest, &digest_length, errp) < 0) {
        return NULL;
    }
    if (digest_length != 32) {
        error_setg(errp, "SlugArch Type-2 SHA-256 length %zu is not 32", digest_length);
        g_free(digest);
        return NULL;
    }

    hex = g_malloc(65);
    for (i = 0; i < digest_length; i++) {
        snprintf(hex + i * 2, 3, "%02x", digest[i]);
    }
    hex[64] = '\0';
    g_free(digest);
    return hex;
}

static bool cxl_type2_slugarch_emit_path_counters_locked(CXLType2State *ct2d, Error **errp) {
    g_autofree char *line = g_strdup_printf(
        "{\"event\":\"path_counters\",\"phase_id\":\"%s\""
        ",\"direct_cfmws\":%" PRIu64 ",\"bar4_overlay\":%" PRIu64 ",\"local_shadow\":%" PRIu64
        ",\"local_cache\":%" PRIu64 ",\"bulk_overlay\":%" PRIu64 ",\"coherent_pool\":%" PRIu64 "}\n",
        ct2d->slugarch.phase_id, ct2d->slugarch.direct_cfmws_completions, ct2d->slugarch.bar4_overlay_completions,
        ct2d->slugarch.local_shadow_completions, ct2d->slugarch.local_cache_completions,
        ct2d->slugarch.bulk_overlay_completions, ct2d->slugarch.coherent_pool_completions);

    return cxl_type2_slugarch_write_line(ct2d, line, errp);
}

static bool cxl_type2_slugarch_emit_completion_locked(CXLType2State *ct2d, bool is_write, hwaddr dpa, unsigned size,
                                                      uint64_t request_id, uint64_t server_sequence,
                                                      uint64_t modeled_latency, const uint8_t *payload,
                                                      const SlugArchT2DelayResult *delay, Error **errp) {
    g_autofree char *digest = cxl_type2_slugarch_sha256_hex(payload, size, errp);
    g_autofree char *line = NULL;

    if (!digest) {
        return false;
    }
    line = g_strdup_printf(
        "{\"event\":\"completion\",\"client_id\":%" PRIu64 ",\"request_id\":%" PRIu64 ",\"server_sequence\":%" PRIu64
        ",\"operation\":\"%s\",\"dpa\":%" HWADDR_PRIu ",\"length\":%u,\"payload_sha256\":\"%s\",\"status\":0"
        ",\"returned_modeled_latency_ns\":%" PRIu64 ",\"requested_delay_ns\":%" PRIu64 ",\"applied_delay_ns\":%" PRIu64
        ",\"delay_overshoot_ns\":%" PRIu64 ",\"delay_undershot\":%s,\"path\":\"direct_cfmws\""
        ",\"phase_id\":\"%s\"}\n",
        ct2d->slugarch.client_id, request_id, server_sequence, is_write ? "write" : "read", dpa, size, digest,
        modeled_latency, delay->requested_ns, delay->actual_ns, delay->overshoot_ns,
        delay->undershot ? "true" : "false", ct2d->slugarch.phase_id);
    return cxl_type2_slugarch_write_line(ct2d, line, errp);
}

static uint64_t cxl_type2_jit_phase_number(const char *phase) {
    const uint8_t *byte = (const uint8_t *)(phase ?: "");
    uint64_t hash = UINT64_C(14695981039346656037);

    while (*byte) {
        hash ^= *byte++;
        hash *= UINT64_C(1099511628211);
    }
    return hash ?: 1;
}

static bool cxl_type2_jit_observe_cfmws(CXLType2State *ct2d, bool is_write, bool is_completion, hwaddr dpa,
                                        unsigned size, uint64_t request_id, uint64_t phase_id, const uint8_t *payload,
                                        uint32_t status, uint64_t *event_id, Error **errp) {
    CXLType2JitEvent event = {
        .client_id = ct2d->slugarch.client_id,
        .direction = is_completion ? SLUG_JIT_DIRECTION_DEVICE_TO_HOST : SLUG_JIT_DIRECTION_HOST_TO_DEVICE,
        .event_class = is_completion ? (is_write ? SLUG_JIT_EVENT_COMPLETION : SLUG_JIT_EVENT_CXL_MEM_DATA)
                                     : (is_write ? SLUG_JIT_EVENT_CXL_MEM_WRITE : SLUG_JIT_EVENT_CXL_MEM_READ),
        .opcode = is_completion ? SLUGARCH_T2_FRAME_MEMORY_RESPONSE
                                : (is_write ? SLUGARCH_T2_FRAME_WRITE : SLUGARCH_T2_FRAME_READ),
        .payload_len = (is_write == is_completion) ? 0 : size,
        .address = dpa,
        .tag = request_id,
        .phase_id = phase_id,
        .monotonic_ns = slugarch_t2_monotonic_ns(),
        .status = status,
    };
    SlugJitDecision decision;
    bool accepted;

    *event_id = 0;
    if (event.payload_len) {
        memcpy(event.payload, payload, event.payload_len);
    }
    accepted = cxl_type2_jit_observe_event(&ct2d->jit.runtime, &event, &decision, errp);
    *event_id = ct2d->jit.runtime.last_event.event_id;
    return accepted;
}

static MemTxResult cxl_type2_slugarch_fail_locked(CXLType2State *ct2d, SlugArchT2IOResult io_result,
                                                  bool mismatched_response, Error *error) {
    ct2d->slugarch.failed_requests++;
    if (io_result == SLUGARCH_T2_IO_TIMEOUT) {
        ct2d->slugarch.timed_out_requests++;
    } else if (io_result == SLUGARCH_T2_IO_EOF || io_result == SLUGARCH_T2_IO_SYSTEM_ERROR) {
        ct2d->slugarch.partial_io_failures++;
    }
    if (mismatched_response || io_result == SLUGARCH_T2_IO_PROTOCOL_ERROR) {
        ct2d->slugarch.mismatched_responses++;
    }
    ct2d->slugarch.connection_failed = true;
    cxl_type2_close_memsim_socket(ct2d);
    if (error) {
        error_report_err(error);
    }
    return MEMTX_ERROR;
}

static MemTxResult cxl_type2_slugarch_access(CXLType2State *ct2d, bool is_write, hwaddr dpa, uint64_t *value,
                                             unsigned size, MemTxAttrs attrs) {
    SlugArchT2MemoryRequest request = {
        .type = is_write ? SLUGARCH_T2_FRAME_WRITE : SLUGARCH_T2_FRAME_READ,
    };
    SlugArchT2MemoryResponse response = {0};
    SlugArchT2Frame request_frame = {0};
    SlugArchT2Frame response_frame = {0};
    SlugArchT2DelayResult delay = {0};
    SlugArchT2IOResult io_result;
    Error *local_err = NULL;
    uint64_t request_id;
    uint64_t deadline_ns;
    uint64_t jit_phase_id = 0;
    uint64_t request_event_id = 0;
    uint64_t completion_event_id = 0;
    uint8_t *shadow;
    const uint8_t *payload;
    MemTxResult result = MEMTX_ERROR;

    (void)attrs;
    if (!value || !size || size > sizeof(*value)) {
        return MEMTX_ERROR;
    }

    qemu_mutex_lock(&ct2d->memsim.lock);
    if (!ct2d->slugarch.enabled || ct2d->slugarch.connection_failed || !ct2d->memsim.connected ||
        !ct2d->memsim.socket) {
        error_setg(&local_err, "SlugArch Type-2 connection is unavailable");
        result = cxl_type2_slugarch_fail_locked(ct2d, SLUGARCH_T2_IO_SYSTEM_ERROR, false, local_err);
        goto out;
    }
    if (dpa > ct2d->device_mem_size || size > ct2d->device_mem_size - dpa) {
        error_setg(&local_err, "SlugArch Type-2 DPA range 0x%" HWADDR_PRIx "+%u exceeds device memory", dpa, size);
        result = cxl_type2_slugarch_fail_locked(ct2d, SLUGARCH_T2_IO_PROTOCOL_ERROR, true, local_err);
        goto out;
    }
    if (!ct2d->slugarch.next_request_id || ct2d->slugarch.next_request_id == UINT64_MAX) {
        error_setg(&local_err, "SlugArch Type-2 request ID space is exhausted");
        result = cxl_type2_slugarch_fail_locked(ct2d, SLUGARCH_T2_IO_PROTOCOL_ERROR, true, local_err);
        goto out;
    }

    request_id = ct2d->slugarch.next_request_id++;
    request.request_id = request_id;
    request.client_id = ct2d->slugarch.client_id;
    request.length = size;
    request.dpa = dpa;
    request.client_monotonic_ns = slugarch_t2_monotonic_ns();
    if (is_write) {
        stn_le_p(request.data, size, *value);
    }
    if (!slugarch_t2_encode_memory_request(&request, &request_frame)) {
        error_setg(&local_err, "SlugArch Type-2 memory request encoding failed");
        result = cxl_type2_slugarch_fail_locked(ct2d, SLUGARCH_T2_IO_PROTOCOL_ERROR, true, local_err);
        goto out;
    }

    if (ct2d->jit.advertised) {
        Error *join_error = NULL;
        uint32_t failure;

        jit_phase_id = cxl_type2_jit_phase_number(ct2d->slugarch.phase_id);
        if (!cxl_type2_jit_observe_cfmws(ct2d, is_write, false, dpa, size, request_id, jit_phase_id, request.data, 0,
                                         &request_event_id, &local_err)) {
            failure = ct2d->jit.runtime.last_error ? ct2d->jit.runtime.last_error : SLUG_JIT_ERR_BACKEND;
            if (!cxl_type2_jit_log_cfmws_join(&ct2d->jit.runtime, request_event_id, 0, request_id, 0, false, failure,
                                              &join_error)) {
                error_report_err(join_error);
            }
            error_prepend(&local_err, "SlugArch CFMWS request record failed: ");
            result = cxl_type2_slugarch_fail_locked(ct2d, SLUGARCH_T2_IO_OK, false, local_err);
            goto out;
        }
    }

    deadline_ns = slugarch_t2_monotonic_ns() + SLUGARCH_T2_REQUEST_TIMEOUT_NS;
    io_result =
        slugarch_t2_exchange_until(ct2d->memsim.socket, &request_frame, deadline_ns, &response_frame, &local_err);
    if (io_result != SLUGARCH_T2_IO_OK) {
        result = cxl_type2_slugarch_fail_locked(ct2d, io_result, false, local_err);
        goto out;
    }
    if (!slugarch_t2_decode_memory_response(&response_frame, &response, &local_err)) {
        result = cxl_type2_slugarch_fail_locked(ct2d, SLUGARCH_T2_IO_PROTOCOL_ERROR, true, local_err);
        goto out;
    }
    if (response.request_id != request_id || response.client_id != ct2d->slugarch.client_id ||
        response.status != SLUGARCH_T2_STATUS_SUCCESS || response.returned_length != (is_write ? 0 : size) ||
        !response.server_sequence || response.modeled_latency > 1000000ULL) {
        error_setg(&local_err, "SlugArch Type-2 memory response does not match request");
        result = cxl_type2_slugarch_fail_locked(ct2d, SLUGARCH_T2_IO_PROTOCOL_ERROR, true, local_err);
        goto out;
    }
    if (!slugarch_t2_apply_delay(response.modeled_latency, &delay, &local_err)) {
        result = cxl_type2_slugarch_fail_locked(ct2d, SLUGARCH_T2_IO_PROTOCOL_ERROR, true, local_err);
        goto out;
    }

    if (!is_write) {
        *value = ldn_le_p(response.data, size);
    }
    payload = is_write ? request.data : response.data;
    if (ct2d->jit.advertised) {
        Error *join_error = NULL;
        uint32_t failure;

        if (!cxl_type2_jit_observe_cfmws(ct2d, is_write, true, dpa, size, request_id, jit_phase_id, payload,
                                         response.status, &completion_event_id, &local_err)) {
            failure = ct2d->jit.runtime.last_error ? ct2d->jit.runtime.last_error : SLUG_JIT_ERR_BACKEND;
            if (!cxl_type2_jit_log_cfmws_join(&ct2d->jit.runtime, request_event_id, completion_event_id, request_id,
                                              response.server_sequence, is_write, failure, &join_error)) {
                error_report_err(join_error);
            }
            error_prepend(&local_err, "SlugArch CFMWS completion record failed: ");
            result = cxl_type2_slugarch_fail_locked(ct2d, SLUGARCH_T2_IO_OK, false, local_err);
            goto out;
        }
        if (!cxl_type2_jit_log_cfmws_join(&ct2d->jit.runtime, request_event_id, completion_event_id, request_id,
                                          response.server_sequence, false, SLUG_JIT_OK, &local_err)) {
            result = cxl_type2_slugarch_fail_locked(ct2d, SLUGARCH_T2_IO_OK, false, local_err);
            goto out;
        }
    }
    if (!cxl_type2_slugarch_emit_completion_locked(ct2d, is_write, dpa, size, request_id, response.server_sequence,
                                                   response.modeled_latency, payload, &delay, &local_err)) {
        result = cxl_type2_slugarch_fail_locked(ct2d, SLUGARCH_T2_IO_SYSTEM_ERROR, false, local_err);
        goto out;
    }

    if (is_write) {
        ct2d->slugarch.completed_writes++;
        ct2d->slugarch.written_bytes += size;
    } else {
        ct2d->slugarch.completed_reads++;
        ct2d->slugarch.read_bytes += size;
    }
    ct2d->slugarch.direct_cfmws_completions++;
    ct2d->slugarch.delay_events++;
    if (delay.undershot) {
        ct2d->slugarch.delay_undershoots++;
    }
    if (!cxl_type2_slugarch_emit_path_counters_locked(ct2d, &local_err)) {
        result = cxl_type2_slugarch_fail_locked(ct2d, SLUGARCH_T2_IO_SYSTEM_ERROR, false, local_err);
        goto out;
    }

    if (is_write && ct2d->slugarch.shadow_after_write) {
        shadow = memory_region_get_ram_ptr(&ct2d->device_mem);
        if (shadow) {
            stn_le_p(shadow + dpa, size, *value);
        }
    }
    result = MEMTX_OK;

out:
    qemu_mutex_unlock(&ct2d->memsim.lock);
    return result;
}

MemTxResult cxl_type2_cfmws_read(PCIDevice *pdev, hwaddr dpa, uint64_t *value, unsigned size, MemTxAttrs attrs) {
    CXLType2State *ct2d = CXL_TYPE2(pdev);
    Error *local_err = NULL;

    if (ct2d->memsim_v2.enabled) {
        CxlMemsimV2Client *client =
            cxl_memsim_v2_path_client(&ct2d->memsim_v2.endpoints, CXL_MEMSIM_V2_PATH_CFMWS_HOST);

        (void)attrs;
        if (!value || dpa > ct2d->device_mem_size || size > ct2d->device_mem_size - dpa ||
            !cxl_memsim_v2_load(client, dpa, size, value, ct2d->memsim_v2.timeout_ms, &local_err)) {
            if (local_err) {
                error_prepend(&local_err, "CXL Type2 protocol-v2 CFMWS read failed: ");
                error_report_err(local_err);
            }
            return MEMTX_ERROR;
        }
        return MEMTX_OK;
    }
    return cxl_type2_slugarch_access(CXL_TYPE2(pdev), false, dpa, value, size, attrs);
}

MemTxResult cxl_type2_cfmws_write(PCIDevice *pdev, hwaddr dpa, uint64_t value, unsigned size, MemTxAttrs attrs) {
    CXLType2State *ct2d = CXL_TYPE2(pdev);
    Error *local_err = NULL;

    if (ct2d->memsim_v2.enabled) {
        CxlMemsimV2Client *client =
            cxl_memsim_v2_path_client(&ct2d->memsim_v2.endpoints, CXL_MEMSIM_V2_PATH_CFMWS_HOST);

        (void)attrs;
        if (dpa > ct2d->device_mem_size || size > ct2d->device_mem_size - dpa ||
            !cxl_memsim_v2_store(client, dpa, size, value, ct2d->memsim_v2.timeout_ms, &local_err)) {
            if (local_err) {
                error_prepend(&local_err, "CXL Type2 protocol-v2 CFMWS write failed: ");
                error_report_err(local_err);
            }
            return MEMTX_ERROR;
        }
        return MEMTX_OK;
    }
    return cxl_type2_slugarch_access(CXL_TYPE2(pdev), true, dpa, &value, size, attrs);
}

/* ========================================================================
 * Memory Access Handlers with Coherency
 * ======================================================================== */

/* Forward declaration for GPU command handler */
static uint64_t cxl_type2_gpu_cmd_read(void *opaque, hwaddr addr, unsigned size);
static void cxl_type2_gpu_cmd_write(void *opaque, hwaddr addr, uint64_t value, unsigned size);

static uint64_t cxl_type2_cache_read(void *opaque, hwaddr addr, unsigned size) {
    CXLType2State *ct2d = opaque;
    CXLCacheLine *line;
    uint64_t value = 0;
    uint64_t cache_line_addr = addr & ~0x3F;
    size_t offset = addr & 0x3F;

    /* Check if this is a GPU command register access */
    if (addr < CXL_GPU_CMD_REG_SIZE) {
        return cxl_type2_gpu_cmd_read(opaque, addr, size);
    }

    ct2d->stats.cpu_accesses++;

    /* Track with enhanced BAR coherency */
    if (ct2d->bar_coherency.enabled) {
        cxl_bar_coherency_request(&ct2d->bar_coherency, CXL_COH_REQ_RD_SHARED, addr, size, CXL_DOMAIN_CPU, NULL);
    }

    /* Check coherency protocol */
    line = cxl_type2_cache_lookup(ct2d, addr);

    if (line && line->state != CXL_COHERENCY_INVALID) {
        /* Cache hit */
        memcpy(&value, &line->data[offset], MIN(size, 64 - offset));
        if (!ct2d->slugarch.enabled) {
            ct2d->slugarch.local_cache_completions++;
        }

        qemu_log_mask(LOG_TRACE, "CXL Type2: Cache read hit at 0x%lx = 0x%lx\n", addr, value);
    } else {
        /* Cache miss - fetch from device memory */
        uint8_t *mem_ptr = memory_region_get_ram_ptr(&ct2d->device_mem);
        if (mem_ptr && addr < ct2d->device_mem_size) {
            memcpy(&value, mem_ptr + addr, size);

            /* Insert into cache */
            uint8_t cache_data[64];
            memcpy(cache_data, mem_ptr + cache_line_addr, 64);
            cxl_type2_cache_insert(ct2d, addr, cache_data, CXL_COHERENCY_SHARED);
        }

        cxl_type2_memsim_request(ct2d, CXL_OP_READ, addr, size, NULL, NULL);

        qemu_log_mask(LOG_TRACE, "CXL Type2: Cache read miss at 0x%lx = 0x%lx\n", addr, value);
    }

    ct2d->stats.read_ops++;
    return value;
}

static void cxl_type2_cache_write(void *opaque, hwaddr addr, uint64_t value, unsigned size) {
    CXLType2State *ct2d = opaque;
    CXLCacheLine *line;
    uint64_t cache_line_addr = addr & ~0x3F;
    size_t offset = addr & 0x3F;

    /* Check if this is a GPU command register access */
    if (addr < CXL_GPU_CMD_REG_SIZE) {
        cxl_type2_gpu_cmd_write(opaque, addr, value, size);
        return;
    }

    ct2d->stats.cpu_accesses++;
    ct2d->stats.write_ops++;

    /* Track with enhanced BAR coherency - write requires exclusive access */
    if (ct2d->bar_coherency.enabled) {
        cxl_bar_coherency_request(&ct2d->bar_coherency, CXL_COH_REQ_WR_INV, addr, size, CXL_DOMAIN_CPU, NULL);
    }

    /* Check if we have the cache line */
    line = cxl_type2_cache_lookup(ct2d, addr);

    if (!line || line->state == CXL_COHERENCY_INVALID) {
        /* Need to fetch cache line first */
        uint8_t *mem_ptr = memory_region_get_ram_ptr(&ct2d->device_mem);
        if (mem_ptr && cache_line_addr < ct2d->device_mem_size) {
            uint8_t cache_data[64];
            memcpy(cache_data, mem_ptr + cache_line_addr, 64);
            cxl_type2_cache_insert(ct2d, addr, cache_data, CXL_COHERENCY_MODIFIED);
            line = cxl_type2_cache_lookup(ct2d, addr);
        }
    }

    if (line) {
        /* Update cache line */
        qemu_mutex_lock(&ct2d->coherency.lock);
        memcpy(&line->data[offset], &value, MIN(size, 64 - offset));
        line->state = CXL_COHERENCY_MODIFIED;
        line->dirty = true;
        line->timestamp = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        qemu_mutex_unlock(&ct2d->coherency.lock);

        /* Write through to device memory */
        uint8_t *mem_ptr = memory_region_get_ram_ptr(&ct2d->device_mem);
        if (mem_ptr && addr < ct2d->device_mem_size) {
            memcpy(mem_ptr + addr, &value, size);
        }

        cxl_type2_memsim_request(ct2d, CXL_OP_WRITE, addr, size, (const uint8_t *)&value, NULL);
    }

    // qemu_log_mask(LOG_TRACE, "CXL Type2: Cache write at 0x%lx = 0x%lx\n",
    //              addr, value);
}

static bool cxl_type2_fabric_access_allowed(CXLType2State *ct2d, uint64_t addr, uint64_t size, bool is_write,
                                            bool is_atomic);

static uint64_t cxl_type2_device_mem_read(void *opaque, hwaddr addr, unsigned size) {
    CXLType2State *ct2d = opaque;
    uint64_t value = 0;

    if (ct2d->slugarch.enabled) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "CXL Type2: synchronous BAR4 access outside "
                      "an explicit overlay at 0x%" HWADDR_PRIx "\n",
                      addr);
        return 0;
    }

    if (!cxl_type2_fabric_access_allowed(ct2d, addr, size, false, false)) {
        return 0;
    }

    /* Fast path: bulk staging region and coherent pool bypass coherency.
     * These regions are used for direct CPU<->GPU pointer sharing
     * and high-throughput data staging; coherency is tracked in commands. */
    if (addr < ct2d->bulk_transfer_size || addr >= ct2d->coherent_pool.base_offset) {
        uint8_t *mem = memory_region_get_ram_ptr(&ct2d->device_mem);
        if (mem && addr + size <= ct2d->device_mem_size) {
            memcpy(&value, mem + addr, MIN(size, 8));
        }
        return value;
    }

    /* Slow path: full coherency tracking for normal device memory */
    return cxl_type2_cache_read(opaque, addr, size);
}

static void cxl_type2_device_mem_write(void *opaque, hwaddr addr, uint64_t value, unsigned size) {
    CXLType2State *ct2d = opaque;

    if (ct2d->slugarch.enabled) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "CXL Type2: synchronous BAR4 write outside "
                      "an explicit overlay at 0x%" HWADDR_PRIx "\n",
                      addr);
        return;
    }

    if (!cxl_type2_fabric_access_allowed(ct2d, addr, size, true, false)) {
        return;
    }

    /* Fast path: bulk staging region and coherent pool bypass coherency */
    if (addr < ct2d->bulk_transfer_size || addr >= ct2d->coherent_pool.base_offset) {
        uint8_t *mem = memory_region_get_ram_ptr(&ct2d->device_mem);
        if (mem && addr + size <= ct2d->device_mem_size) {
            memcpy(mem + addr, &value, MIN(size, 8));
        }
        return;
    }

    /* Slow path: full coherency tracking for normal device memory */
    cxl_type2_cache_write(opaque, addr, value, size);
}

static const MemoryRegionOps cxl_type2_cache_ops = {
    .read = cxl_type2_cache_read,
    .write = cxl_type2_cache_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid =
        {
            .min_access_size = 1,
            .max_access_size = 8,
        },
    .impl =
        {
            .min_access_size = 1,
            .max_access_size = 8,
        },
};

static const MemoryRegionOps cxl_type2_device_mem_ops = {
    .read = cxl_type2_device_mem_read,
    .write = cxl_type2_device_mem_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid =
        {
            .min_access_size = 1,
            .max_access_size = 8,
        },
    .impl =
        {
            .min_access_size = 1,
            .max_access_size = 8,
        },
};

static void cxl_type2_slugarch_overlay_complete(CXLType2State *ct2d, bool coherent_pool) {
    Error *local_err = NULL;

    if (!ct2d->slugarch.enabled) {
        ct2d->slugarch.bar4_overlay_completions++;
        if (coherent_pool) {
            ct2d->slugarch.coherent_pool_completions++;
        } else {
            ct2d->slugarch.bulk_overlay_completions++;
        }
        return;
    }

    qemu_mutex_lock(&ct2d->memsim.lock);
    ct2d->slugarch.bar4_overlay_completions++;
    if (coherent_pool) {
        ct2d->slugarch.coherent_pool_completions++;
    } else {
        ct2d->slugarch.bulk_overlay_completions++;
    }
    if (!cxl_type2_slugarch_emit_path_counters_locked(ct2d, &local_err)) {
        ct2d->slugarch.connection_failed = true;
        cxl_type2_close_memsim_socket(ct2d);
        error_report_err(local_err);
    }
    qemu_mutex_unlock(&ct2d->memsim.lock);
}

static uint64_t cxl_type2_bulk_proxy_read(void *opaque, hwaddr addr, unsigned size) {
    CXLType2State *ct2d = opaque;
    uint64_t value = 0;

    if (ct2d->bulk_transfer_ptr && addr <= ct2d->bulk_transfer_size && size <= ct2d->bulk_transfer_size - addr) {
        value = ldn_le_p((uint8_t *)ct2d->bulk_transfer_ptr + addr, size);
        cxl_type2_slugarch_overlay_complete(ct2d, false);
    }
    return value;
}

static void cxl_type2_bulk_proxy_write(void *opaque, hwaddr addr, uint64_t value, unsigned size) {
    CXLType2State *ct2d = opaque;

    if (ct2d->bulk_transfer_ptr && addr <= ct2d->bulk_transfer_size && size <= ct2d->bulk_transfer_size - addr) {
        stn_le_p((uint8_t *)ct2d->bulk_transfer_ptr + addr, size, value);
        cxl_type2_slugarch_overlay_complete(ct2d, false);
    }
}

static const MemoryRegionOps cxl_type2_bulk_proxy_ops = {
    .read = cxl_type2_bulk_proxy_read,
    .write = cxl_type2_bulk_proxy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid =
        {
            .min_access_size = 1,
            .max_access_size = 8,
        },
    .impl =
        {
            .min_access_size = 1,
            .max_access_size = 8,
        },
};

static uint64_t cxl_type2_coherent_proxy_read(void *opaque, hwaddr addr, unsigned size) {
    CXLType2State *ct2d = opaque;
    CxlMemsimV2Client *client;
    Error *local_err = NULL;
    uint64_t value = 0;

    if (!ct2d->coherent_pool_host_ptr || addr > ct2d->coherent_pool.size ||
        size > ct2d->coherent_pool.size - addr) {
        ct2d->coherent_pool_access_errors++;
        return 0;
    }

    if (ct2d->memsim_v2.enabled) {
        client = cxl_memsim_v2_path_client(&ct2d->memsim_v2.endpoints,
                                           CXL_MEMSIM_V2_PATH_CFMWS_HOST);
        if (!cxl_memsim_v2_load(client, ct2d->coherent_pool.base_offset + addr,
                                size, &value, ct2d->memsim_v2.timeout_ms,
                                &local_err)) {
            ct2d->coherent_pool_access_errors++;
            if (local_err) {
                error_prepend(&local_err,
                              "CXL Type2 coherent-pool host read failed: ");
                error_report_err(local_err);
            }
            return 0;
        }
        stn_le_p(ct2d->coherent_pool_host_ptr + addr, size, value);
    } else {
        value = ldn_le_p(ct2d->coherent_pool_host_ptr + addr, size);
    }
    ct2d->coherent_pool_host_loads++;
    cxl_type2_slugarch_overlay_complete(ct2d, true);
    return value;
}

static void cxl_type2_coherent_proxy_write(void *opaque, hwaddr addr, uint64_t value, unsigned size) {
    CXLType2State *ct2d = opaque;
    CxlMemsimV2Client *client;
    Error *local_err = NULL;

    if (!ct2d->coherent_pool_host_ptr || addr > ct2d->coherent_pool.size ||
        size > ct2d->coherent_pool.size - addr) {
        ct2d->coherent_pool_access_errors++;
        return;
    }

    if (ct2d->memsim_v2.enabled) {
        client = cxl_memsim_v2_path_client(&ct2d->memsim_v2.endpoints,
                                           CXL_MEMSIM_V2_PATH_CFMWS_HOST);
        if (!cxl_memsim_v2_store(client, ct2d->coherent_pool.base_offset + addr,
                                 size, value, ct2d->memsim_v2.timeout_ms,
                                 &local_err)) {
            ct2d->coherent_pool_access_errors++;
            if (local_err) {
                error_prepend(&local_err,
                              "CXL Type2 coherent-pool host write failed: ");
                error_report_err(local_err);
            }
            return;
        }
    }
    stn_le_p(ct2d->coherent_pool_host_ptr + addr, size, value);
    ct2d->coherent_pool_host_stores++;
    cxl_type2_slugarch_overlay_complete(ct2d, true);
}

static const MemoryRegionOps cxl_type2_coherent_proxy_ops = {
    .read = cxl_type2_coherent_proxy_read,
    .write = cxl_type2_coherent_proxy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid =
        {
            .min_access_size = 1,
            .max_access_size = 8,
        },
    .impl =
        {
            .min_access_size = 1,
            .max_access_size = 8,
        },
};

/* ========================================================================
 * DCD / GFAM / MH-SLD Fabric Memory Models
 * ======================================================================== */

static uint64_t cxl_type2_dcd_align(CXLType2State *ct2d, uint64_t value) {
    uint64_t granularity = ct2d->dcd.granularity;

    if (!granularity) {
        granularity = CXL_TYPE2_DCD_DEFAULT_GRANULARITY;
    }

    return (value + granularity - 1) & ~(granularity - 1);
}

static bool cxl_type2_range_valid(CXLType2State *ct2d, uint64_t base, uint64_t size) {
    return size && base < ct2d->device_mem_size && size <= ct2d->device_mem_size - base;
}

static bool cxl_type2_dcd_overlaps_locked(CXLType2State *ct2d, uint64_t base, uint64_t size) {
    uint64_t end = base + size;

    for (int i = 0; i < CXL_TYPE2_MAX_DCD_EXTENTS; i++) {
        CXLType2DCDExtent *extent = &ct2d->dcd.extents[i];
        uint64_t extent_end;

        if (!extent->active) {
            continue;
        }

        extent_end = extent->base + extent->size;
        if (base < extent_end && extent->base < end) {
            return true;
        }
    }

    return false;
}

static bool cxl_type2_dcd_allocated_locked(CXLType2State *ct2d, uint64_t base, uint64_t size) {
    uint64_t end = base + size;

    if (!ct2d->dcd.enabled) {
        return true;
    }

    for (int i = 0; i < CXL_TYPE2_MAX_DCD_EXTENTS; i++) {
        CXLType2DCDExtent *extent = &ct2d->dcd.extents[i];

        if (!extent->active) {
            continue;
        }
        if (base >= extent->base && end <= extent->base + extent->size) {
            return true;
        }
    }

    return false;
}

static bool cxl_type2_dcd_allocated(CXLType2State *ct2d, uint64_t base, uint64_t size) {
    bool allocated;

    qemu_mutex_lock(&ct2d->dcd.lock);
    allocated = cxl_type2_dcd_allocated_locked(ct2d, base, size);
    qemu_mutex_unlock(&ct2d->dcd.lock);

    return allocated;
}

static uint64_t cxl_type2_dcd_active_extents_locked(CXLType2State *ct2d) {
    uint64_t count = 0;

    for (int i = 0; i < CXL_TYPE2_MAX_DCD_EXTENTS; i++) {
        if (ct2d->dcd.extents[i].active) {
            count++;
        }
    }

    return count;
}

static bool cxl_type2_dcd_find_free_locked(CXLType2State *ct2d, uint64_t size, uint64_t *base) {
    uint64_t candidate = 0;

    while (cxl_type2_range_valid(ct2d, candidate, size)) {
        bool moved = false;

        for (int i = 0; i < CXL_TYPE2_MAX_DCD_EXTENTS; i++) {
            CXLType2DCDExtent *extent = &ct2d->dcd.extents[i];

            if (!extent->active) {
                continue;
            }
            if (candidate < extent->base + extent->size && extent->base < candidate + size) {
                candidate = cxl_type2_dcd_align(ct2d, extent->base + extent->size);
                moved = true;
                break;
            }
        }
        if (!moved) {
            *base = candidate;
            return true;
        }
    }

    return false;
}

static int cxl_type2_dcd_add(CXLType2State *ct2d, uint64_t requested_base, uint64_t size, uint64_t tag,
                             uint64_t *out_base, uint64_t *out_size, uint64_t *out_tag) {
    uint64_t base = requested_base;
    uint64_t aligned_size = cxl_type2_dcd_align(ct2d, size);
    int slot = -1;

    if (!ct2d->dcd.enabled) {
        return -EOPNOTSUPP;
    }

    qemu_mutex_lock(&ct2d->dcd.lock);
    ct2d->dcd.add_requests++;

    if (!aligned_size || aligned_size > ct2d->device_mem_size) {
        ct2d->dcd.failed_requests++;
        qemu_mutex_unlock(&ct2d->dcd.lock);
        return -EINVAL;
    }

    if (requested_base == UINT64_MAX) {
        if (!cxl_type2_dcd_find_free_locked(ct2d, aligned_size, &base)) {
            ct2d->dcd.failed_requests++;
            qemu_mutex_unlock(&ct2d->dcd.lock);
            return -ENOSPC;
        }
    } else if (base != cxl_type2_dcd_align(ct2d, base)) {
        ct2d->dcd.failed_requests++;
        qemu_mutex_unlock(&ct2d->dcd.lock);
        return -EINVAL;
    }

    if (!cxl_type2_range_valid(ct2d, base, aligned_size) || cxl_type2_dcd_overlaps_locked(ct2d, base, aligned_size)) {
        ct2d->dcd.failed_requests++;
        qemu_mutex_unlock(&ct2d->dcd.lock);
        return -ENOSPC;
    }

    for (int i = 0; i < CXL_TYPE2_MAX_DCD_EXTENTS; i++) {
        if (!ct2d->dcd.extents[i].active) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        ct2d->dcd.failed_requests++;
        qemu_mutex_unlock(&ct2d->dcd.lock);
        return -ENOSPC;
    }

    if (!tag) {
        tag = ct2d->dcd.next_tag++;
    }

    ct2d->dcd.extents[slot] = (CXLType2DCDExtent){
        .base = base,
        .size = aligned_size,
        .tag = tag,
        .active = true,
    };
    ct2d->dcd.allocated += aligned_size;

    if (out_base) {
        *out_base = base;
    }
    if (out_size) {
        *out_size = aligned_size;
    }
    if (out_tag) {
        *out_tag = tag;
    }

    qemu_mutex_unlock(&ct2d->dcd.lock);

    qemu_log("CXL Type2 DCD: add base=0x%" PRIx64 " size=0x%" PRIx64 " tag=%" PRIu64 "\n", base, aligned_size, tag);
    return 0;
}

static int cxl_type2_dcd_release(CXLType2State *ct2d, uint64_t base, uint64_t size, uint64_t tag) {
    uint64_t aligned_size = cxl_type2_dcd_align(ct2d, size);

    if (!ct2d->dcd.enabled) {
        return -EOPNOTSUPP;
    }

    qemu_mutex_lock(&ct2d->dcd.lock);
    ct2d->dcd.release_requests++;

    for (int i = 0; i < CXL_TYPE2_MAX_DCD_EXTENTS; i++) {
        CXLType2DCDExtent *extent = &ct2d->dcd.extents[i];

        if (!extent->active || extent->base != base || extent->size != aligned_size) {
            continue;
        }
        if (tag && extent->tag != tag) {
            continue;
        }

        extent->active = false;
        ct2d->dcd.allocated -= extent->size;
        qemu_mutex_unlock(&ct2d->dcd.lock);

        qemu_log("CXL Type2 DCD: release base=0x%" PRIx64 " size=0x%" PRIx64 " tag=%" PRIu64 "\n", base, aligned_size,
                 tag);
        return 0;
    }

    ct2d->dcd.failed_requests++;
    qemu_mutex_unlock(&ct2d->dcd.lock);
    return -ENOENT;
}

static void cxl_type2_gfam_revoke_range(CXLType2State *ct2d, uint64_t base, uint64_t size) {
    uint64_t end = base + size;

    qemu_mutex_lock(&ct2d->gfam.lock);
    for (int i = 0; i < CXL_TYPE2_MAX_GFAM_MAPPINGS; i++) {
        CXLType2GFAMMapping *mapping = &ct2d->gfam.mappings[i];
        uint64_t mapping_end;

        if (!mapping->active) {
            continue;
        }

        mapping_end = mapping->base + mapping->size;
        if (base < mapping_end && mapping->base < end) {
            mapping->active = false;
        }
    }
    qemu_mutex_unlock(&ct2d->gfam.lock);
}

static int cxl_type2_gfam_grant(CXLType2State *ct2d, uint32_t host_id, uint64_t base, uint64_t size,
                                uint32_t permissions) {
    int slot = -1;

    if (!ct2d->gfam.enabled) {
        return -EOPNOTSUPP;
    }
    if (host_id >= ct2d->gfam.num_hosts || !permissions || !cxl_type2_dcd_allocated(ct2d, base, size)) {
        return -EINVAL;
    }

    qemu_mutex_lock(&ct2d->gfam.lock);
    for (int i = 0; i < CXL_TYPE2_MAX_GFAM_MAPPINGS; i++) {
        CXLType2GFAMMapping *mapping = &ct2d->gfam.mappings[i];

        if (mapping->active && mapping->host_id == host_id && mapping->base == base && mapping->size == size) {
            mapping->permissions = permissions;
            qemu_mutex_unlock(&ct2d->gfam.lock);
            return 0;
        }
        if (!mapping->active && slot < 0) {
            slot = i;
        }
    }

    if (slot < 0) {
        qemu_mutex_unlock(&ct2d->gfam.lock);
        return -ENOSPC;
    }

    ct2d->gfam.mappings[slot] = (CXLType2GFAMMapping){
        .host_id = host_id,
        .permissions = permissions,
        .base = base,
        .size = size,
        .active = true,
    };

    qemu_mutex_unlock(&ct2d->gfam.lock);
    return 0;
}

static int cxl_type2_gfam_revoke(CXLType2State *ct2d, uint32_t host_id, uint64_t base, uint64_t size) {
    if (!ct2d->gfam.enabled) {
        return -EOPNOTSUPP;
    }

    qemu_mutex_lock(&ct2d->gfam.lock);
    for (int i = 0; i < CXL_TYPE2_MAX_GFAM_MAPPINGS; i++) {
        CXLType2GFAMMapping *mapping = &ct2d->gfam.mappings[i];

        if (mapping->active && mapping->host_id == host_id && mapping->base == base && mapping->size == size) {
            mapping->active = false;
            qemu_mutex_unlock(&ct2d->gfam.lock);
            return 0;
        }
    }
    qemu_mutex_unlock(&ct2d->gfam.lock);
    return -ENOENT;
}

static uint64_t cxl_type2_gfam_active_mappings_locked(CXLType2State *ct2d) {
    uint64_t count = 0;

    for (int i = 0; i < CXL_TYPE2_MAX_GFAM_MAPPINGS; i++) {
        if (ct2d->gfam.mappings[i].active) {
            count++;
        }
    }

    return count;
}

static bool cxl_type2_gfam_allowed(CXLType2State *ct2d, uint32_t host_id, uint64_t base, uint64_t size, bool is_write,
                                   bool is_atomic) {
    uint32_t required = is_write ? CXL_DCD_PERM_WRITE : CXL_DCD_PERM_READ;
    uint64_t end = base + size;

    if (!ct2d->gfam.enabled) {
        return true;
    }
    if (is_atomic) {
        required |= CXL_DCD_PERM_ATOMIC;
    }

    qemu_mutex_lock(&ct2d->gfam.lock);
    for (int i = 0; i < CXL_TYPE2_MAX_GFAM_MAPPINGS; i++) {
        CXLType2GFAMMapping *mapping = &ct2d->gfam.mappings[i];

        if (!mapping->active || mapping->host_id != host_id) {
            continue;
        }
        if (base >= mapping->base && end <= mapping->base + mapping->size &&
            (mapping->permissions & required) == required) {
            ct2d->gfam.allowed_accesses++;
            ct2d->gfam.total_latency_ns += ct2d->gfam.fabric_latency_ns;
            qemu_mutex_unlock(&ct2d->gfam.lock);
            return true;
        }
    }

    ct2d->gfam.denied_accesses++;
    qemu_mutex_unlock(&ct2d->gfam.lock);
    return false;
}

static uint32_t cxl_type2_popcount32(uint32_t value) {
    uint32_t count = 0;

    while (value) {
        count += value & 1;
        value >>= 1;
    }

    return count;
}

static void cxl_type2_mhsld_record(CXLType2State *ct2d, uint64_t addr, bool is_write, bool is_atomic) {
    CXLType2MHSLDLine *line;
    uint64_t line_addr;
    uint32_t head;
    uint32_t head_mask;
    uint32_t other_sharers;
    uint32_t index;

    if (!ct2d->mhsld.enabled) {
        return;
    }

    line_addr = addr & CXL_CACHE_LINE_MASK;
    index = (line_addr >> 6) % CXL_TYPE2_MAX_MHSLD_LINES;
    head = ct2d->mhsld.local_head_id;
    head_mask = head < 32 ? 1u << head : 0;

    qemu_mutex_lock(&ct2d->mhsld.lock);
    line = &ct2d->mhsld.lines[index];
    if (!line->valid || line->line_addr != line_addr) {
        *line = (CXLType2MHSLDLine){
            .line_addr = line_addr,
            .owner_head = head,
            .sharer_mask = head_mask,
            .valid = true,
            .modified = false,
        };
    }

    if (is_write || is_atomic) {
        other_sharers = line->sharer_mask & ~head_mask;
        if ((line->modified && line->owner_head != head) || other_sharers) {
            ct2d->mhsld.conflicts++;
            ct2d->mhsld.invalidations += cxl_type2_popcount32(other_sharers);
        }
        line->owner_head = head;
        line->sharer_mask = head_mask;
        line->modified = true;
        ct2d->mhsld.writes++;
        if (is_atomic) {
            ct2d->mhsld.atomics++;
        }
    } else {
        if (line->modified && line->owner_head != head) {
            ct2d->mhsld.conflicts++;
            line->modified = false;
        }
        line->sharer_mask |= head_mask;
        ct2d->mhsld.reads++;
    }

    qemu_mutex_unlock(&ct2d->mhsld.lock);
}

static bool cxl_type2_fabric_access_allowed(CXLType2State *ct2d, uint64_t addr, uint64_t size, bool is_write,
                                            bool is_atomic) {
    if (!cxl_type2_range_valid(ct2d, addr, size)) {
        return false;
    }

    if (!cxl_type2_dcd_allocated(ct2d, addr, size)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "CXL Type2 DCD: access outside allocated capacity "
                      "addr=0x%" PRIx64 " size=0x%" PRIx64 "\n",
                      addr, size);
        return false;
    }

    if (!cxl_type2_gfam_allowed(ct2d, ct2d->gfam.local_host_id, addr, size, is_write, is_atomic)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "CXL Type2 GFAM: denied host=%u addr=0x%" PRIx64 " size=0x%" PRIx64 " write=%d\n",
                      ct2d->gfam.local_host_id, addr, size, is_write);
        return false;
    }

    cxl_type2_mhsld_record(ct2d, addr, is_write, is_atomic);
    return true;
}

static void cxl_type2_fabric_features_init(CXLType2State *ct2d) {
    bool use_dcd = ct2d->dcd.enabled || ct2d->gfam.enabled;
    uint64_t initial_size;

    qemu_mutex_init(&ct2d->dcd.lock);
    qemu_mutex_init(&ct2d->gfam.lock);
    qemu_mutex_init(&ct2d->mhsld.lock);

    if (!ct2d->dcd.granularity || (ct2d->dcd.granularity & (ct2d->dcd.granularity - 1))) {
        ct2d->dcd.granularity = CXL_TYPE2_DCD_DEFAULT_GRANULARITY;
    }
    ct2d->dcd.next_tag = 1;

    if (ct2d->gfam.num_hosts == 0) {
        ct2d->gfam.num_hosts = 1;
    }
    if (ct2d->gfam.num_hosts > CXL_TYPE2_MAX_GFAM_HOSTS) {
        ct2d->gfam.num_hosts = CXL_TYPE2_MAX_GFAM_HOSTS;
    }
    if (ct2d->gfam.local_host_id >= ct2d->gfam.num_hosts) {
        ct2d->gfam.local_host_id = 0;
    }
    if (!ct2d->gfam.default_permissions) {
        ct2d->gfam.default_permissions = CXL_DCD_PERM_ALL;
    }
    if (!ct2d->gfam.fabric_latency_ns) {
        ct2d->gfam.fabric_latency_ns = 150;
    }
    if (!ct2d->gfam.bandwidth_mbps) {
        ct2d->gfam.bandwidth_mbps = 32768;
    }

    if (!ct2d->mhsld.num_heads) {
        ct2d->mhsld.num_heads = ct2d->gfam.num_hosts;
    }
    if (ct2d->mhsld.num_heads > 32) {
        ct2d->mhsld.num_heads = 32;
    }
    if (ct2d->mhsld.local_head_id >= ct2d->mhsld.num_heads) {
        ct2d->mhsld.local_head_id = 0;
    }
    if (!ct2d->mhsld.coherency_latency_ns) {
        ct2d->mhsld.coherency_latency_ns = 200;
    }

    if (!use_dcd) {
        return;
    }

    ct2d->dcd.enabled = true;
    initial_size = ct2d->dcd.initial_size;
    if (initial_size == CXL_TYPE2_DCD_INIT_AUTO) {
        initial_size = ct2d->device_mem_size;
    }
    if (initial_size) {
        uint64_t base;
        uint64_t size;
        uint64_t tag;

        if (cxl_type2_dcd_add(ct2d, 0, initial_size, 1, &base, &size, &tag) == 0 && ct2d->gfam.enabled) {
            for (uint32_t host = 0; host < ct2d->gfam.num_hosts; host++) {
                cxl_type2_gfam_grant(ct2d, host, base, size, ct2d->gfam.default_permissions);
            }
        }
    }

    qemu_log("CXL Type2 fabric: DCD=%d GFAM=%d MHSLD=%d initial=%" PRIu64 "MB hosts=%u heads=%u\n", ct2d->dcd.enabled,
             ct2d->gfam.enabled, ct2d->mhsld.enabled, initial_size / MiB, ct2d->gfam.num_hosts, ct2d->mhsld.num_heads);
}

static void cxl_type2_fabric_features_cleanup(CXLType2State *ct2d) {
    qemu_mutex_destroy(&ct2d->mhsld.lock);
    qemu_mutex_destroy(&ct2d->gfam.lock);
    qemu_mutex_destroy(&ct2d->dcd.lock);
}

/* ========================================================================
 * Coherent Pool Allocator
 * ======================================================================== */

/* First-fit allocator from free list, page-aligned (4KB) */
static int64_t cxl_coherent_pool_alloc(CXLType2State *ct2d, uint64_t size) {
    uint64_t aligned_size = (size + 0xFFF) & ~0xFFFULL; /* 4KB page align */
    CXLCohFreeBlock **prev = &ct2d->coherent_pool.free_list;
    CXLCohFreeBlock *blk = ct2d->coherent_pool.free_list;

    qemu_mutex_lock(&ct2d->coherent_pool.lock);

    while (blk) {
        if (blk->size >= aligned_size) {
            uint64_t alloc_offset = blk->offset;

            if (blk->size == aligned_size) {
                /* Exact fit - remove block */
                *prev = blk->next;
                g_free(blk);
            } else {
                /* Split block */
                blk->offset += aligned_size;
                blk->size -= aligned_size;
            }

            /* Track allocation */
            uint64_t *key = g_new(uint64_t, 1);
            uint64_t *val = g_new(uint64_t, 1);
            *key = alloc_offset;
            *val = aligned_size;
            g_hash_table_insert(ct2d->coherent_pool.allocations, key, val);
            ct2d->coherent_pool.used += aligned_size;

            qemu_mutex_unlock(&ct2d->coherent_pool.lock);

            qemu_log("CXL Type2: Coherent pool alloc: offset=0x%lx size=%lu\n", (unsigned long)alloc_offset,
                     (unsigned long)aligned_size);
            return (int64_t)alloc_offset;
        }
        prev = &blk->next;
        blk = blk->next;
    }

    qemu_mutex_unlock(&ct2d->coherent_pool.lock);
    qemu_log("CXL Type2: Coherent pool alloc FAILED: size=%lu (used=%lu/%lu)\n", (unsigned long)size,
             (unsigned long)ct2d->coherent_pool.used, (unsigned long)ct2d->coherent_pool.size);
    return -1;
}

/* Free allocation and coalesce with adjacent free blocks */
static int cxl_coherent_pool_free(CXLType2State *ct2d, uint64_t offset) {
    qemu_mutex_lock(&ct2d->coherent_pool.lock);

    uint64_t *alloc_size = g_hash_table_lookup(ct2d->coherent_pool.allocations, &offset);
    if (!alloc_size) {
        qemu_mutex_unlock(&ct2d->coherent_pool.lock);
        qemu_log("CXL Type2: Coherent pool free FAILED: offset=0x%lx not found\n", (unsigned long)offset);
        return -1;
    }

    uint64_t size = *alloc_size;
    g_hash_table_remove(ct2d->coherent_pool.allocations, &offset);
    ct2d->coherent_pool.used -= size;

    /* Insert back into sorted free list with coalescing */
    CXLCohFreeBlock *new_blk = g_new0(CXLCohFreeBlock, 1);
    new_blk->offset = offset;
    new_blk->size = size;
    new_blk->next = NULL;

    CXLCohFreeBlock **prev = &ct2d->coherent_pool.free_list;
    CXLCohFreeBlock *cur = ct2d->coherent_pool.free_list;

    /* Find insertion point (sorted by offset) */
    while (cur && cur->offset < offset) {
        prev = &cur->next;
        cur = cur->next;
    }

    new_blk->next = cur;
    *prev = new_blk;

    /* Coalesce with next block */
    if (new_blk->next && new_blk->offset + new_blk->size == new_blk->next->offset) {
        CXLCohFreeBlock *merged = new_blk->next;
        new_blk->size += merged->size;
        new_blk->next = merged->next;
        g_free(merged);
    }

    /* Coalesce with previous block */
    if (prev != &ct2d->coherent_pool.free_list) {
        CXLCohFreeBlock *prev_blk = ct2d->coherent_pool.free_list;
        while (prev_blk && prev_blk->next != new_blk) {
            prev_blk = prev_blk->next;
        }
        if (prev_blk && prev_blk->offset + prev_blk->size == new_blk->offset) {
            prev_blk->size += new_blk->size;
            prev_blk->next = new_blk->next;
            g_free(new_blk);
        }
    }

    qemu_mutex_unlock(&ct2d->coherent_pool.lock);

    qemu_log("CXL Type2: Coherent pool free: offset=0x%lx size=%lu\n", (unsigned long)offset, (unsigned long)size);
    return 0;
}

/* ========================================================================
 * GPU Command Interface
 * ======================================================================== */

static void cxl_type2_jit_capture_diagnostic(CXLType2State *ct2d, CXLType2JitState *runtime) {
    Error *local_error = NULL;
    uint32_t required = 0;
    uint32_t written = 0;

    ct2d->jit.diagnostic_length = 0;
    memset(ct2d->jit.diagnostic, 0, sizeof(ct2d->jit.diagnostic));
    if (!cxl_type2_jit_copy_diagnostic(runtime, NULL, 0, &required, &local_error)) {
        error_free(local_error);
        return;
    }
    if (!required || required > sizeof(ct2d->jit.diagnostic)) {
        return;
    }
    if (!cxl_type2_jit_copy_diagnostic(runtime, ct2d->jit.diagnostic, required, &written, &local_error)) {
        error_free(local_error);
        return;
    }
    ct2d->jit.diagnostic_length = written;
}

static uint32_t cxl_type2_jit_replace_policy(CXLType2State *ct2d, const uint8_t *policy, size_t policy_len) {
    CXLType2JitState candidate;
    CXLType2JitState *runtime = &ct2d->jit.runtime;
    Error *local_error = NULL;
    uint32_t failure;

    cxl_type2_jit_state_init(&candidate, runtime->requested_backend, ct2d->jit.strict, CXL_TYPE2_JIT_DIAGNOSTIC_SIZE);
    if (!cxl_type2_jit_open(&candidate, ct2d->jit.library_path, &local_error) ||
        !cxl_type2_jit_load_policy(&candidate, policy, policy_len, &local_error) ||
        (ct2d->jit.log_path && !cxl_type2_jit_open_log(&candidate, ct2d->jit.log_path, &local_error))) {
        failure = candidate.last_error;
        if (failure == SLUG_JIT_OK) {
            failure = candidate.library ? SLUG_JIT_ERR_IO : SLUG_JIT_ERR_BACKEND;
        }
        cxl_type2_jit_capture_diagnostic(ct2d, &candidate);
        runtime->last_error = failure;
        if (local_error) {
            qemu_log_mask(LOG_GUEST_ERROR, "CXL GPU JIT policy replacement failed: %s\n",
                          error_get_pretty(local_error));
        }
        error_free(local_error);
        cxl_type2_jit_close(&candidate);
        return failure;
    }

    cxl_type2_jit_close(runtime);
    *runtime = candidate;
    ct2d->jit.diagnostic_length = 0;
    memset(ct2d->jit.diagnostic, 0, sizeof(ct2d->jit.diagnostic));
    return SLUG_JIT_OK;
}

static uint32_t cxl_type2_jit_replace_policy_file(CXLType2State *ct2d) {
    g_autofree char *contents = NULL;
    g_autoptr(GError) local_error = NULL;
    gsize length = 0;

    if (!g_file_get_contents(ct2d->jit.policy_path, &contents, &length, &local_error)) {
        size_t diagnostic_length = MIN(strlen(local_error->message), sizeof(ct2d->jit.diagnostic));

        memcpy(ct2d->jit.diagnostic, local_error->message, diagnostic_length);
        ct2d->jit.diagnostic_length = diagnostic_length;
        ct2d->jit.runtime.last_error = SLUG_JIT_ERR_IO;
        return SLUG_JIT_ERR_IO;
    }
    return cxl_type2_jit_replace_policy(ct2d, (const uint8_t *)contents, length);
}

static uint32_t cxl_type2_jit_get_diagnostic(CXLType2State *ct2d, uint32_t capacity, uint32_t *required,
                                             uint32_t *written) {
    CXLType2JitState *runtime = &ct2d->jit.runtime;
    g_autofree uint8_t *live = NULL;
    Error *local_error = NULL;
    const uint8_t *source = ct2d->jit.diagnostic;
    uint32_t source_length = ct2d->jit.diagnostic_length;
    uint32_t live_length = 0;
    uint32_t live_written = 0;

    if (!source_length && !cxl_type2_jit_copy_diagnostic(runtime, NULL, 0, &live_length, &local_error)) {
        error_free(local_error);
        return runtime->last_error ? runtime->last_error : SLUG_JIT_ERR_BACKEND;
    }
    if (!source_length && live_length) {
        if (live_length > CXL_TYPE2_JIT_DIAGNOSTIC_SIZE) {
            return SLUG_JIT_ERR_BUDGET_EXCEEDED;
        }
        live = g_malloc(live_length);
        if (!cxl_type2_jit_copy_diagnostic(runtime, live, live_length, &live_written, &local_error)) {
            error_free(local_error);
            return runtime->last_error ? runtime->last_error : SLUG_JIT_ERR_BACKEND;
        }
        source = live;
        source_length = live_written;
    }

    *required = source_length;
    *written = MIN(capacity, source_length);
    if (capacity) {
        memset(ct2d->gpu_cmd.data, 0, capacity);
    }
    if (*written) {
        memcpy(ct2d->gpu_cmd.data, source, *written);
    }
    return SLUG_JIT_OK;
}

static bool cxl_type2_memsim_v2_range_valid(CXLType2State *ct2d, uint64_t address, unsigned size) {
    return address <= ct2d->device_mem_size && size <= ct2d->device_mem_size - address;
}

static CxlMemsimV2Client *cxl_type2_memsim_v2_device_client(CXLType2State *ct2d) {
    return cxl_memsim_v2_path_client(&ct2d->memsim_v2.endpoints, CXL_MEMSIM_V2_PATH_BAR2_DEVICE);
}

static void cxl_type2_memsim_v2_command_error(Error *err) {
    if (err) {
        error_prepend(&err, "CXL Type2 BAR2 protocol-v2 command failed: ");
        error_report_err(err);
    }
}

static void cxl_type2_gpu_execute_cmd(CXLType2State *ct2d, uint32_t cmd) {
    HetGPUState *hetgpu = &ct2d->gpu_info.hetgpu_state;
    HetGPUError err;
    uint64_t dev_ptr;
    size_t size;
    bool jit_command = cmd >= CXL_GPU_CMD_J_QUERY && cmd <= CXL_GPU_CMD_J_GET_DIAGNOSTIC;
    bool memsim_v2_command =
        (cmd >= CXL_GPU_CMD_COHERENT_LOAD && cmd <= CXL_GPU_CMD_COHERENT_CAS) || cmd == CXL_GPU_CMD_COHERENT_FENCE;

    qemu_log_mask(LOG_GUEST_ERROR, "CXL GPU: execute cmd 0x%x, hetgpu_init=%d, ctx=%p\n", cmd, hetgpu->initialized,
                  hetgpu->context);

    ct2d->gpu_cmd.cmd_status = CXL_GPU_CMD_STATUS_RUNNING;
    ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
    if (jit_command || memsim_v2_command) {
        memset(ct2d->gpu_cmd.results, 0, sizeof(ct2d->gpu_cmd.results));
    }

    switch (cmd) {
    case CXL_GPU_CMD_NOP:
        break;

    case CXL_GPU_CMD_INIT:
        if (!hetgpu->initialized) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_NOT_INITIALIZED;
        }
        break;

    case CXL_GPU_CMD_GET_DEVICE_COUNT:
        ct2d->gpu_cmd.results[0] = hetgpu->initialized ? 1 : 0;
        break;

    case CXL_GPU_CMD_GET_DEVICE:
        if (ct2d->gpu_cmd.params[0] == 0 && hetgpu->initialized) {
            ct2d->gpu_cmd.results[0] = 0; /* Device handle */
        } else {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_DEVICE;
        }
        break;

    case CXL_GPU_CMD_GET_DEVICE_PROPS:
        if (hetgpu->initialized) {
            HetGPUDeviceProps props;
            err = hetgpu_get_device_props(hetgpu, &props);
            if (err == HETGPU_SUCCESS) {
                /* Copy device name to data buffer */
                memcpy(ct2d->gpu_cmd.data, props.name, sizeof(props.name));
                ct2d->gpu_cmd.results[0] = props.total_memory;
                ct2d->gpu_cmd.results[1] = (props.compute_capability_major << 16) | props.compute_capability_minor;
                ct2d->gpu_cmd.results[2] = props.multiprocessor_count;
                ct2d->gpu_cmd.results[3] = props.max_threads_per_block;
            } else {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_DEVICE;
            }
        } else {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_NOT_INITIALIZED;
        }
        break;

    case CXL_GPU_CMD_GET_TOTAL_MEM:
        if (hetgpu->initialized) {
            ct2d->gpu_cmd.results[0] = hetgpu->props.total_memory;
        } else {
            ct2d->gpu_cmd.results[0] = ct2d->device_mem_size;
        }
        break;

    case CXL_GPU_CMD_CTX_CREATE:
        if (hetgpu->initialized) {
            err = hetgpu_create_context(hetgpu);
            if (err == HETGPU_SUCCESS) {
                ct2d->gpu_cmd.results[0] = (uint64_t)(uintptr_t)hetgpu->context;
            } else {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_CONTEXT;
            }
        } else {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_NOT_INITIALIZED;
        }
        break;

    case CXL_GPU_CMD_CTX_DESTROY:
        /* Context destroyed on device cleanup */
        break;

    case CXL_GPU_CMD_CTX_SYNC:
        if (hetgpu->initialized) {
            err = hetgpu_synchronize(hetgpu);
            if (err != HETGPU_SUCCESS) {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_UNKNOWN;
            }
        }
        break;

    case CXL_GPU_CMD_MEM_ALLOC:
        size = ct2d->gpu_cmd.params[0];
        if (hetgpu->initialized) {
            err = hetgpu_malloc(hetgpu, size, HETGPU_MEM_DEFAULT, &dev_ptr);
            if (err == HETGPU_SUCCESS) {
                ct2d->gpu_cmd.results[0] = dev_ptr;
            } else {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_OUT_OF_MEMORY;
            }
        } else {
            /* Fallback: allocate from device memory region */
            /* Simple bump allocator - in real impl, use proper allocator */
            static uint64_t next_alloc = 0;
            if (next_alloc + size <= ct2d->device_mem_size) {
                ct2d->gpu_cmd.results[0] = next_alloc;
                next_alloc += (size + 0xFFF) & ~0xFFF; /* Page align */
            } else {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_OUT_OF_MEMORY;
            }
        }
        break;

    case CXL_GPU_CMD_MEM_FREE:
        dev_ptr = ct2d->gpu_cmd.params[0];
        if (hetgpu->initialized) {
            err = hetgpu_free(hetgpu, dev_ptr);
            if (err != HETGPU_SUCCESS) {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
            }
        }
        /* Fallback: no-op for simple allocator */
        break;

    case CXL_GPU_CMD_MEM_COPY_HTOD:
        dev_ptr = ct2d->gpu_cmd.params[0]; /* dst device ptr */
        size = ct2d->gpu_cmd.params[1]; /* size */
        /* Data is in ct2d->gpu_cmd.data buffer */
        if (size > ct2d->gpu_cmd.data_size) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
            break;
        }
        if (hetgpu->initialized) {
            err = hetgpu_memcpy_htod(hetgpu, dev_ptr, ct2d->gpu_cmd.data, size);
            if (err != HETGPU_SUCCESS) {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
            }
            /* Also update shadow copy in device_mem for coherency tracking */
            if (dev_ptr + size <= ct2d->device_mem_size &&
                cxl_type2_fabric_access_allowed(ct2d, dev_ptr, size, true, false)) {
                uint8_t *mem = memory_region_get_ram_ptr(&ct2d->device_mem);
                if (mem) {
                    memcpy(mem + dev_ptr, ct2d->gpu_cmd.data, size);
                    /* Notify BAR coherency layer of GPU write */
                    if (ct2d->bar_coherency.enabled) {
                        cxl_bar_notify_gpu_access(&ct2d->bar_coherency, dev_ptr, size, true);
                    }
                }
            }
        } else {
            /* Fallback: copy to device memory region */
            if (dev_ptr + size <= ct2d->device_mem_size &&
                cxl_type2_fabric_access_allowed(ct2d, dev_ptr, size, true, false)) {
                uint8_t *mem = memory_region_get_ram_ptr(&ct2d->device_mem);
                if (mem) {
                    memcpy(mem + dev_ptr, ct2d->gpu_cmd.data, size);
                    /* Notify BAR coherency layer of GPU write */
                    if (ct2d->bar_coherency.enabled) {
                        cxl_bar_notify_gpu_access(&ct2d->bar_coherency, dev_ptr, size, true);
                    }
                }
            } else {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
            }
        }
        break;

    case CXL_GPU_CMD_MEM_COPY_DTOH:
        dev_ptr = ct2d->gpu_cmd.params[0]; /* src device ptr */
        size = ct2d->gpu_cmd.params[1]; /* size */
        if (size > ct2d->gpu_cmd.data_size) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
            break;
        }
        if (hetgpu->initialized) {
            /* Notify BAR coherency layer before GPU read */
            if (ct2d->bar_coherency.enabled) {
                cxl_bar_notify_gpu_access(&ct2d->bar_coherency, dev_ptr, size, false);
            }
            err = hetgpu_memcpy_dtoh(hetgpu, ct2d->gpu_cmd.data, dev_ptr, size);
            if (err != HETGPU_SUCCESS) {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
            }
            /* Update shadow copy from GPU for coherency */
            if (dev_ptr + size <= ct2d->device_mem_size &&
                cxl_type2_fabric_access_allowed(ct2d, dev_ptr, size, false, false)) {
                uint8_t *mem = memory_region_get_ram_ptr(&ct2d->device_mem);
                if (mem) {
                    memcpy(mem + dev_ptr, ct2d->gpu_cmd.data, size);
                }
            }
        } else {
            /* Fallback: copy from device memory region */
            if (dev_ptr + size <= ct2d->device_mem_size &&
                cxl_type2_fabric_access_allowed(ct2d, dev_ptr, size, false, false)) {
                uint8_t *mem = memory_region_get_ram_ptr(&ct2d->device_mem);
                if (mem) {
                    /* Notify BAR coherency layer before GPU read */
                    if (ct2d->bar_coherency.enabled) {
                        cxl_bar_notify_gpu_access(&ct2d->bar_coherency, dev_ptr, size, false);
                    }
                    memcpy(ct2d->gpu_cmd.data, mem + dev_ptr, size);
                }
            } else {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
            }
        }
        break;

    case CXL_GPU_CMD_MODULE_LOAD_PTX:
        if (hetgpu->initialized && ct2d->gpu_cmd.num_modules < 64) {
            /* PTX source is in data buffer */
            void *module = NULL;
            err = hetgpu_load_ptx(hetgpu, (const char *)ct2d->gpu_cmd.data, (HetGPUModule *)&module);
            if (err == HETGPU_SUCCESS) {
                ct2d->gpu_cmd.modules[ct2d->gpu_cmd.num_modules] = module;
                /* Return 1-based ID so guest handles are never NULL-looking */
                ct2d->gpu_cmd.results[0] = ct2d->gpu_cmd.num_modules + 1;
                ct2d->gpu_cmd.num_modules++;
            } else {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_PTX;
            }
        } else {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_NOT_INITIALIZED;
        }
        break;

    case CXL_GPU_CMD_FUNC_GET:
        if (hetgpu->initialized && ct2d->gpu_cmd.num_functions < 256) {
            /* Guest sends 1-based module ID */
            uint32_t module_id_1based = ct2d->gpu_cmd.params[0];
            uint32_t module_id = module_id_1based - 1;
            /* Function name is in data buffer */
            if (module_id < ct2d->gpu_cmd.num_modules) {
                void *func = NULL;
                err = hetgpu_get_function(hetgpu, ct2d->gpu_cmd.modules[module_id], (const char *)ct2d->gpu_cmd.data,
                                          (HetGPUFunction *)&func);
                if (err == HETGPU_SUCCESS) {
                    ct2d->gpu_cmd.functions[ct2d->gpu_cmd.num_functions] = func;
                    /* Return 1-based ID */
                    ct2d->gpu_cmd.results[0] = ct2d->gpu_cmd.num_functions + 1;
                    ct2d->gpu_cmd.num_functions++;
                } else {
                    ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_NOT_FOUND;
                }
            } else {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_HANDLE;
            }
        } else {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_NOT_INITIALIZED;
        }
        break;

    case CXL_GPU_CMD_LAUNCH_KERNEL:
        if (hetgpu->initialized) {
            /* Guest sends 1-based function ID */
            uint32_t func_id = ct2d->gpu_cmd.params[0] - 1;
            if (func_id < ct2d->gpu_cmd.num_functions) {
                HetGPULaunchConfig config;
                config.grid_dim[0] = ct2d->gpu_cmd.params[1] & 0xFFFFFFFF;
                config.grid_dim[1] = (ct2d->gpu_cmd.params[1] >> 32) & 0xFFFFFFFF;
                config.grid_dim[2] = ct2d->gpu_cmd.params[2] & 0xFFFFFFFF;
                config.block_dim[0] = (ct2d->gpu_cmd.params[2] >> 32) & 0xFFFFFFFF;
                config.block_dim[1] = ct2d->gpu_cmd.params[3] & 0xFFFFFFFF;
                config.block_dim[2] = (ct2d->gpu_cmd.params[3] >> 32) & 0xFFFFFFFF;
                config.shared_mem_bytes = ct2d->gpu_cmd.params[4] & 0xFFFFFFFF;
                config.stream = NULL;

                /* Kernel args are in data buffer as array of pointers */
                uint32_t num_args = (ct2d->gpu_cmd.params[4] >> 32) & 0xFF;
                void **args = (void **)ct2d->gpu_cmd.data;

                err = hetgpu_launch_kernel(hetgpu, ct2d->gpu_cmd.functions[func_id], &config, args, num_args);
                if (err != HETGPU_SUCCESS) {
                    ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_LAUNCH_FAILED;
                }
            } else {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_HANDLE;
            }
        } else {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_NOT_INITIALIZED;
        }
        break;

    /* Bulk transfer commands - optimized for large memory operations */
    case CXL_GPU_CMD_BULK_HTOD:
        /* Bulk host-to-device transfer via BAR4 staging RAM */
        {
            uint64_t bar4_offset = ct2d->gpu_cmd.params[0]; /* Offset in staging */
            uint64_t dst_dev_ptr = ct2d->gpu_cmd.params[1]; /* Device destination */
            size_t xfer_size = ct2d->gpu_cmd.params[2]; /* Transfer size */

            if (hetgpu->initialized) {
                /* Guest wrote data into the bulk staging RAM subregion
                 * (priority 2 window, no vmexits).  Read from staging ptr. */
                if (ct2d->bulk_transfer_ptr && bar4_offset <= ct2d->bulk_transfer_size &&
                    xfer_size <= ct2d->bulk_transfer_size - bar4_offset && xfer_size <= CXL_GPU_BULK_TRANSFER_SIZE &&
                    cxl_type2_fabric_access_allowed(ct2d, bar4_offset, xfer_size, false, false)) {
                    err = hetgpu_memcpy_htod(hetgpu, dst_dev_ptr, (uint8_t *)ct2d->bulk_transfer_ptr + bar4_offset,
                                             xfer_size);
                    if (err != HETGPU_SUCCESS) {
                        ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
                    }
                } else {
                    ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
                }
            } else {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_NOT_INITIALIZED;
            }
        }
        break;

    case CXL_GPU_CMD_BULK_DTOH:
        /* Bulk device-to-host transfer via BAR4 staging RAM */
        {
            uint64_t src_dev_ptr = ct2d->gpu_cmd.params[0]; /* Device source */
            uint64_t bar4_offset = ct2d->gpu_cmd.params[1]; /* Offset in staging */
            size_t xfer_size = ct2d->gpu_cmd.params[2]; /* Transfer size */

            if (hetgpu->initialized) {
                /* Write data into the bulk staging RAM subregion
                 * (priority 2 window).  Guest reads directly, no vmexits. */
                if (ct2d->bulk_transfer_ptr && bar4_offset <= ct2d->bulk_transfer_size &&
                    xfer_size <= ct2d->bulk_transfer_size - bar4_offset && xfer_size <= CXL_GPU_BULK_TRANSFER_SIZE &&
                    cxl_type2_fabric_access_allowed(ct2d, bar4_offset, xfer_size, true, false)) {
                    err = hetgpu_memcpy_dtoh(hetgpu, (uint8_t *)ct2d->bulk_transfer_ptr + bar4_offset, src_dev_ptr,
                                             xfer_size);
                    if (err != HETGPU_SUCCESS) {
                        ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
                    }
                } else {
                    ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
                }
            } else {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_NOT_INITIALIZED;
            }
        }
        break;

    case CXL_GPU_CMD_BULK_DTOD:
        /* Bulk device-to-device transfer */
        {
            uint64_t src_dev_ptr = ct2d->gpu_cmd.params[0]; /* Source device ptr */
            uint64_t dst_dev_ptr = ct2d->gpu_cmd.params[1]; /* Dest device ptr */
            size_t xfer_size = ct2d->gpu_cmd.params[2]; /* Transfer size */

            if (hetgpu->initialized) {
                err = hetgpu_memcpy_dtod(hetgpu, dst_dev_ptr, src_dev_ptr, xfer_size);
                if (err != HETGPU_SUCCESS) {
                    ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
                }
            } else {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_NOT_INITIALIZED;
            }
        }
        break;

    /* CXL.cache coherency commands */
    case CXL_GPU_CMD_CACHE_FLUSH:
        /* Flush cache lines to device - notify CXLMemSim */
        {
            uint64_t flush_addr = ct2d->gpu_cmd.params[0];
            size_t flush_size = ct2d->gpu_cmd.params[1];

            if (ct2d->coherency.coherency_enabled && ct2d->memsim.connected) {
                cxl_type2_memsim_request(ct2d, CXL_OP_FENCE, flush_addr, flush_size, NULL, NULL);
                /* Invalidate local cache entries */
                for (uint64_t addr = flush_addr; addr < flush_addr + flush_size; addr += 64) {
                    cxl_type2_cache_invalidate(ct2d, addr);
                }
                ct2d->coherency.coherency_ops++;
            }
        }
        break;

    case CXL_GPU_CMD_CACHE_INVALIDATE:
        /* Invalidate cache lines */
        {
            uint64_t inv_addr = ct2d->gpu_cmd.params[0];
            size_t inv_size = ct2d->gpu_cmd.params[1];

            if (ct2d->coherency.coherency_enabled) {
                for (uint64_t addr = inv_addr; addr < inv_addr + inv_size; addr += 64) {
                    cxl_type2_cache_invalidate(ct2d, addr);
                }
                ct2d->coherency.coherency_ops++;
            }
        }
        break;

    case CXL_GPU_CMD_CACHE_WRITEBACK:
        /* Writeback dirty cache lines */
        {
            uint64_t wb_addr = ct2d->gpu_cmd.params[0];
            size_t wb_size = ct2d->gpu_cmd.params[1];

            if (ct2d->coherency.coherency_enabled) {
                for (uint64_t addr = wb_addr; addr < wb_addr + wb_size; addr += 64) {
                    cxl_type2_cache_writeback(ct2d, addr);
                }
                ct2d->coherency.coherency_ops++;
            }
        }
        break;

    case CXL_GPU_CMD_CACHE_PREFETCH:
        /* Prefetch cache lines into the Type2 coherent cache */
        {
            uint64_t pf_addr = ct2d->gpu_cmd.params[0];
            uint64_t pf_size = ct2d->gpu_cmd.params[1];
            bool write_intent = (ct2d->gpu_cmd.params[2] & 1) != 0;

            cxl_type2_cache_prefetch(ct2d, pf_addr, pf_size, write_intent);
            ct2d->gpu_cmd.results[0] = pf_size;
            ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
        }
        break;

    case CXL_GPU_CMD_COHERENT_LOAD: {
        uint64_t address = ct2d->gpu_cmd.params[0];
        unsigned access_size = ct2d->gpu_cmd.params[1];
        uint64_t value = 0;
        Error *local_err = NULL;

        if (!ct2d->memsim_v2.enabled) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_NOT_INITIALIZED;
        } else if (!cxl_type2_memsim_v2_range_valid(ct2d, address, access_size)) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
        } else if (!cxl_memsim_v2_load(cxl_type2_memsim_v2_device_client(ct2d), address, access_size, &value,
                                       ct2d->memsim_v2.timeout_ms, &local_err)) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_UNKNOWN;
            cxl_type2_memsim_v2_command_error(local_err);
        } else {
            ct2d->gpu_cmd.results[0] = value;
            ct2d->gpu_cmd.results[1] = ct2d->memsim_v2.device_endpoint;
            ct2d->gpu_cmd.results[2] = cxl_memsim_v2_client_session(cxl_type2_memsim_v2_device_client(ct2d));
        }
    } break;

    case CXL_GPU_CMD_COHERENT_STORE: {
        uint64_t address = ct2d->gpu_cmd.params[0];
        unsigned access_size = ct2d->gpu_cmd.params[1];
        uint64_t value = ct2d->gpu_cmd.params[2];
        Error *local_err = NULL;

        if (!ct2d->memsim_v2.enabled) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_NOT_INITIALIZED;
        } else if (!cxl_type2_memsim_v2_range_valid(ct2d, address, access_size)) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
        } else if (!cxl_memsim_v2_store(cxl_type2_memsim_v2_device_client(ct2d), address, access_size, value,
                                        ct2d->memsim_v2.timeout_ms, &local_err)) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_UNKNOWN;
            cxl_type2_memsim_v2_command_error(local_err);
        } else {
            ct2d->gpu_cmd.results[1] = ct2d->memsim_v2.device_endpoint;
            ct2d->gpu_cmd.results[2] = cxl_memsim_v2_client_session(cxl_type2_memsim_v2_device_client(ct2d));
        }
    } break;

    case CXL_GPU_CMD_COHERENT_FAA: {
        uint64_t address = ct2d->gpu_cmd.params[0];
        uint64_t addend = ct2d->gpu_cmd.params[1];
        uint64_t old_value = 0;
        uint64_t new_value = 0;
        Error *local_err = NULL;

        if (!ct2d->memsim_v2.enabled) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_NOT_INITIALIZED;
        } else if (!cxl_type2_memsim_v2_range_valid(ct2d, address, sizeof(uint64_t))) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
        } else if (!cxl_memsim_v2_fetch_add(cxl_type2_memsim_v2_device_client(ct2d), address, addend, &old_value,
                                            &new_value, ct2d->memsim_v2.timeout_ms, &local_err)) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_UNKNOWN;
            cxl_type2_memsim_v2_command_error(local_err);
        } else {
            ct2d->gpu_cmd.results[0] = old_value;
            ct2d->gpu_cmd.results[1] = new_value;
            ct2d->gpu_cmd.results[2] = cxl_memsim_v2_client_session(cxl_type2_memsim_v2_device_client(ct2d));
        }
    } break;

    case CXL_GPU_CMD_COHERENT_CAS: {
        uint64_t address = ct2d->gpu_cmd.params[0];
        uint64_t expected_value = ct2d->gpu_cmd.params[1];
        uint64_t desired = ct2d->gpu_cmd.params[2];
        uint64_t old_value = 0;
        uint64_t new_value = 0;
        Error *local_err = NULL;

        if (!ct2d->memsim_v2.enabled) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_NOT_INITIALIZED;
        } else if (!cxl_type2_memsim_v2_range_valid(ct2d, address, sizeof(uint64_t))) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
        } else if (!cxl_memsim_v2_compare_exchange(cxl_type2_memsim_v2_device_client(ct2d), address, expected_value,
                                                   desired, &old_value, &new_value, ct2d->memsim_v2.timeout_ms,
                                                   &local_err)) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_UNKNOWN;
            cxl_type2_memsim_v2_command_error(local_err);
        } else {
            ct2d->gpu_cmd.results[0] = old_value;
            ct2d->gpu_cmd.results[1] = new_value;
            ct2d->gpu_cmd.results[2] = cxl_memsim_v2_client_session(cxl_type2_memsim_v2_device_client(ct2d));
        }
    } break;

    /* P2P DMA commands */
    case CXL_GPU_CMD_P2P_DISCOVER:
        /* Discover P2P peer devices */
        {
            int num_peers = cxl_p2p_discover_peers(&ct2d->p2p_engine);
            if (num_peers >= 0) {
                ct2d->gpu_cmd.results[0] = num_peers;
                ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
            } else {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_NOT_INITIALIZED;
            }
        }
        break;

    case CXL_GPU_CMD_P2P_GET_PEER_INFO:
        /* Get peer device info: params[0] = peer_id */
        {
            uint32_t peer_id = ct2d->gpu_cmd.params[0];
            CXLP2PPeer *peer = cxl_p2p_get_peer(&ct2d->p2p_engine, peer_id);
            if (peer && peer->active) {
                ct2d->gpu_cmd.results[0] = peer->type;
                ct2d->gpu_cmd.results[1] = peer->mem_size;
                ct2d->gpu_cmd.results[2] = peer->coherent;
                ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
            } else {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
            }
        }
        break;

    case CXL_GPU_CMD_P2P_GPU_TO_MEM:
        /* GPU -> Type3 transfer: params[0]=peer_id, params[1]=gpu_off, params[2]=mem_off, params[3]=size */
        {
            uint32_t t3_peer_id = ct2d->gpu_cmd.params[0];
            uint64_t gpu_offset = ct2d->gpu_cmd.params[1];
            uint64_t mem_offset = ct2d->gpu_cmd.params[2];
            uint64_t xfer_size = ct2d->gpu_cmd.params[3];
            uint32_t flags = CXL_P2P_FLAG_COHERENT;

            int ret = cxl_p2p_gpu_to_mem(&ct2d->p2p_engine, t3_peer_id, gpu_offset, mem_offset, xfer_size, flags);
            ct2d->gpu_cmd.cmd_result = (ret == 0) ? CXL_GPU_SUCCESS : CXL_GPU_ERROR_OUT_OF_MEMORY;
        }
        break;

    case CXL_GPU_CMD_P2P_MEM_TO_GPU:
        /* Type3 -> GPU transfer: params[0]=peer_id, params[1]=mem_off, params[2]=gpu_off, params[3]=size */
        {
            uint32_t t3_peer_id = ct2d->gpu_cmd.params[0];
            uint64_t mem_offset = ct2d->gpu_cmd.params[1];
            uint64_t gpu_offset = ct2d->gpu_cmd.params[2];
            uint64_t xfer_size = ct2d->gpu_cmd.params[3];
            uint32_t flags = CXL_P2P_FLAG_COHERENT;

            int ret = cxl_p2p_mem_to_gpu(&ct2d->p2p_engine, t3_peer_id, mem_offset, gpu_offset, xfer_size, flags);
            ct2d->gpu_cmd.cmd_result = (ret == 0) ? CXL_GPU_SUCCESS : CXL_GPU_ERROR_OUT_OF_MEMORY;
        }
        break;

    case CXL_GPU_CMD_P2P_MEM_TO_MEM:
        /* Type3 -> Type3 transfer: params[0]=src_peer, params[1]=dst_peer, params[2]=src_off, params[3]=dst_off,
         * params[4]=size */
        {
            uint32_t src_peer_id = ct2d->gpu_cmd.params[0];
            uint32_t dst_peer_id = ct2d->gpu_cmd.params[1];
            uint64_t src_offset = ct2d->gpu_cmd.params[2];
            uint64_t dst_offset = ct2d->gpu_cmd.params[3];
            uint64_t xfer_size = ct2d->gpu_cmd.params[4];
            uint32_t flags = CXL_P2P_FLAG_COHERENT;

            int ret = cxl_p2p_mem_to_mem(&ct2d->p2p_engine, src_peer_id, dst_peer_id, src_offset, dst_offset, xfer_size,
                                         flags);
            ct2d->gpu_cmd.cmd_result = (ret == 0) ? CXL_GPU_SUCCESS : CXL_GPU_ERROR_OUT_OF_MEMORY;
        }
        break;

    case CXL_GPU_CMD_P2P_SYNC:
        /* Wait for all pending P2P transfers */
        /* Currently all transfers are synchronous, so this is a no-op */
        ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
        break;

    case CXL_GPU_CMD_P2P_GET_STATUS:
        /* Get P2P engine status and stats */
        {
            ct2d->gpu_cmd.results[0] = ct2d->p2p_engine.num_peers;
            ct2d->gpu_cmd.results[1] = ct2d->p2p_engine.stats.transfers_completed;
            ct2d->gpu_cmd.results[2] = ct2d->p2p_engine.stats.bytes_transferred;
            ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
        }
        break;

    /* ---- Coherent shared memory pool commands ---- */
    case CXL_GPU_CMD_COHERENT_ALLOC: {
        uint64_t alloc_size = ct2d->gpu_cmd.params[0];
        int64_t offset = cxl_coherent_pool_alloc(ct2d, alloc_size);
        if (offset >= 0) {
            ct2d->gpu_cmd.results[0] = (uint64_t)offset;
            ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
        } else {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_OUT_OF_MEMORY;
        }
    } break;

    case CXL_GPU_CMD_COHERENT_FREE: {
        uint64_t free_offset = ct2d->gpu_cmd.params[0];
        if (cxl_coherent_pool_free(ct2d, free_offset) == 0) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
        } else {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
        }
    } break;

    case CXL_GPU_CMD_COHERENT_GET_INFO: {
        ct2d->gpu_cmd.results[0] = ct2d->coherent_pool.base_offset;
        ct2d->gpu_cmd.results[1] = ct2d->coherent_pool.size;
        ct2d->gpu_cmd.results[2] = ct2d->coherent_pool.size - ct2d->coherent_pool.used;
        ct2d->gpu_cmd.results[3] = ct2d->bar_coherency.snoop_filter_size;
        ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
    } break;

    case CXL_GPU_CMD_COHERENT_FENCE: {
        if (ct2d->memsim_v2.enabled) {
            Error *local_err = NULL;

            if (!cxl_memsim_v2_fence(cxl_type2_memsim_v2_device_client(ct2d), ct2d->memsim_v2.timeout_ms, &local_err)) {
                ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_UNKNOWN;
                cxl_type2_memsim_v2_command_error(local_err);
            } else {
                ct2d->gpu_cmd.results[1] = ct2d->memsim_v2.device_endpoint;
                ct2d->gpu_cmd.results[2] = cxl_memsim_v2_client_session(cxl_type2_memsim_v2_device_client(ct2d));
            }
        } else {
            /* Preserve the legacy local fence when v2 is disabled. */
            cxl_bar_memory_fence(&ct2d->bar_coherency, CXL_DOMAIN_CPU);
            cxl_bar_process_back_invalidations(ct2d);
            ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
        }
    } break;

    /* ---- Device-biased directory commands ---- */
    case CXL_GPU_CMD_SET_BIAS: {
        uint64_t bias_addr = ct2d->gpu_cmd.params[0];
        uint64_t bias_size = ct2d->gpu_cmd.params[1];
        uint8_t bias_mode = (uint8_t)ct2d->gpu_cmd.params[2];
        if (bias_mode > CXL_BIAS_DEVICE) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
        } else {
            cxl_bar_set_bias(&ct2d->bar_coherency, bias_addr, bias_size, bias_mode);
            ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
        }
    } break;

    case CXL_GPU_CMD_GET_BIAS: {
        uint64_t query_addr = ct2d->gpu_cmd.params[0];
        ct2d->gpu_cmd.results[0] = cxl_bar_get_bias(&ct2d->bar_coherency, query_addr);
        ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
    } break;

    case CXL_GPU_CMD_BIAS_FLIP: {
        uint64_t flip_addr = ct2d->gpu_cmd.params[0];
        uint64_t flip_size = ct2d->gpu_cmd.params[1];
        uint8_t new_bias = (uint8_t)ct2d->gpu_cmd.params[2];
        if (new_bias > CXL_BIAS_DEVICE) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
        } else {
            cxl_bar_bias_flip(ct2d, flip_addr, flip_size, new_bias);
            ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
        }
    } break;

    /* ---- Coherency statistics commands ---- */
    case CXL_GPU_CMD_COH_GET_STATS: {
        CXLBARCoherencyState *coh = &ct2d->bar_coherency;
        /* Pack stats into results and data buffer */
        ct2d->gpu_cmd.results[0] = coh->stats.snoop_hits;
        ct2d->gpu_cmd.results[1] = coh->stats.snoop_misses;
        ct2d->gpu_cmd.results[2] = coh->stats.coherency_requests;
        ct2d->gpu_cmd.results[3] = coh->stats.back_invalidations;
        /* Extended stats in data buffer */
        if (ct2d->gpu_cmd.data_size >= 64) {
            uint64_t *stats_buf = (uint64_t *)ct2d->gpu_cmd.data;
            stats_buf[0] = coh->stats.writebacks;
            stats_buf[1] = coh->stats.evictions;
            stats_buf[2] = coh->stats.bias_flips;
            stats_buf[3] = coh->stats.device_bias_hits;
            stats_buf[4] = coh->stats.host_bias_hits;
            stats_buf[5] = coh->stats.upgrades;
            stats_buf[6] = coh->stats.downgrades;
            stats_buf[7] = coh->snoop_filter_size;
        }
        ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
    } break;

    case CXL_GPU_CMD_COH_RESET_STATS: {
        memset(&ct2d->bar_coherency.stats, 0, sizeof(ct2d->bar_coherency.stats));
        ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
    } break;

    case CXL_GPU_CMD_J_QUERY:
        if (!ct2d->jit.advertised) {
            ct2d->gpu_cmd.cmd_result = SLUG_JIT_ERR_UNSUPPORTED;
            break;
        }
        ct2d->gpu_cmd.results[0] = ct2d->jit.runtime.abi_version;
        ct2d->gpu_cmd.results[1] = ct2d->jit.runtime.capabilities;
        ct2d->gpu_cmd.results[2] = ct2d->jit.runtime.selected_backend;
        ct2d->gpu_cmd.results[3] = ct2d->jit.runtime.policy_bytes;
        break;

    case CXL_GPU_CMD_J_LOAD_POLICY: {
        uint64_t policy_len = ct2d->gpu_cmd.params[0];
        g_autofree uint8_t *policy = NULL;

        if (!ct2d->jit.advertised || !ct2d->jit.runtime.ready || !ct2d->jit.strict) {
            ct2d->gpu_cmd.cmd_result = SLUG_JIT_ERR_UNSUPPORTED;
            if (ct2d->jit.advertised) {
                ct2d->jit.runtime.last_error = SLUG_JIT_ERR_UNSUPPORTED;
            }
            break;
        }
        if (!policy_len || policy_len > CXL_GPU_DATA_SIZE || policy_len > SLUG_JIT_MAX_POLICY_BYTES) {
            ct2d->gpu_cmd.cmd_result = SLUG_JIT_ERR_STRUCT_SIZE;
            ct2d->jit.runtime.last_error = SLUG_JIT_ERR_STRUCT_SIZE;
            break;
        }
        policy = g_memdup2(ct2d->gpu_cmd.data, policy_len);
        ct2d->gpu_cmd.cmd_result = cxl_type2_jit_replace_policy(ct2d, policy, policy_len);
        if (ct2d->gpu_cmd.cmd_result == SLUG_JIT_OK) {
            memset(ct2d->gpu_cmd.data, 0, policy_len);
            ct2d->gpu_cmd.results[0] = ct2d->jit.runtime.policy_bytes;
            ct2d->gpu_cmd.results[1] = ct2d->jit.runtime.selected_backend;
            ct2d->gpu_cmd.results[2] = ct2d->jit.runtime.stats.epoch;
            memcpy(&ct2d->gpu_cmd.results[3], ct2d->jit.runtime.policy_digest, sizeof(ct2d->gpu_cmd.results[3]));
        }
    } break;

    /* ---- DCD / GFAM / MH-SLD fabric-memory commands ---- */
    case CXL_GPU_CMD_DCD_ADD: {
        uint64_t base = ct2d->gpu_cmd.params[0];
        uint64_t add_size = ct2d->gpu_cmd.params[1];
        uint64_t tag = ct2d->gpu_cmd.params[2];
        uint64_t out_base = 0;
        uint64_t out_size = 0;
        uint64_t out_tag = 0;
        int ret;

        ret = cxl_type2_dcd_add(ct2d, base, add_size, tag, &out_base, &out_size, &out_tag);
        if (ret == 0) {
            ct2d->gpu_cmd.results[0] = out_base;
            ct2d->gpu_cmd.results[1] = out_size;
            ct2d->gpu_cmd.results[2] = out_tag;
            ct2d->gpu_cmd.results[3] = ct2d->dcd.allocated;
            if (ct2d->gfam.enabled) {
                for (uint32_t host = 0; host < ct2d->gfam.num_hosts; host++) {
                    cxl_type2_gfam_grant(ct2d, host, out_base, out_size, ct2d->gfam.default_permissions);
                }
            }
            ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
        } else {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_OUT_OF_MEMORY;
        }
    } break;

    case CXL_GPU_CMD_J_RESET:
        if (!ct2d->jit.advertised || !ct2d->jit.runtime.ready || !ct2d->jit.strict || !ct2d->jit.policy_path) {
            ct2d->gpu_cmd.cmd_result = SLUG_JIT_ERR_UNSUPPORTED;
            if (ct2d->jit.advertised) {
                ct2d->jit.runtime.last_error = SLUG_JIT_ERR_UNSUPPORTED;
            }
            break;
        }
        ct2d->gpu_cmd.cmd_result = cxl_type2_jit_replace_policy_file(ct2d);
        break;

    case CXL_GPU_CMD_J_GET_STATS: {
        Error *local_error = NULL;

        if (!ct2d->jit.advertised || !cxl_type2_jit_refresh_stats(&ct2d->jit.runtime, &local_error)) {
            ct2d->gpu_cmd.cmd_result =
                ct2d->jit.runtime.last_error ? ct2d->jit.runtime.last_error : SLUG_JIT_ERR_UNSUPPORTED;
            error_free(local_error);
            break;
        }
        ct2d->gpu_cmd.results[0] = ct2d->jit.runtime.stats.event_count;
        ct2d->gpu_cmd.results[1] = ct2d->jit.runtime.stats.record_count;
        ct2d->gpu_cmd.results[2] = ct2d->jit.runtime.stats.metadata_bytes;
        ct2d->gpu_cmd.results[3] = ct2d->jit.runtime.stats.epoch;
    } break;

    case CXL_GPU_CMD_J_GET_DIAGNOSTIC: {
        uint64_t capacity = ct2d->gpu_cmd.params[0];
        uint32_t required = 0;
        uint32_t written = 0;

        if (!ct2d->jit.advertised) {
            ct2d->gpu_cmd.cmd_result = SLUG_JIT_ERR_UNSUPPORTED;
            break;
        }
        if (capacity > CXL_GPU_DATA_SIZE) {
            ct2d->gpu_cmd.cmd_result = SLUG_JIT_ERR_STRUCT_SIZE;
            ct2d->jit.runtime.last_error = SLUG_JIT_ERR_STRUCT_SIZE;
            break;
        }
        ct2d->gpu_cmd.cmd_result = cxl_type2_jit_get_diagnostic(ct2d, capacity, &required, &written);
        ct2d->gpu_cmd.results[0] = required;
        ct2d->gpu_cmd.results[1] = written;
    } break;

    case CXL_GPU_CMD_DCD_RELEASE: {
        uint64_t base = ct2d->gpu_cmd.params[0];
        uint64_t release_size = ct2d->gpu_cmd.params[1];
        uint64_t tag = ct2d->gpu_cmd.params[2];
        int ret;

        ret = cxl_type2_dcd_release(ct2d, base, release_size, tag);
        if (ret == 0) {
            cxl_type2_gfam_revoke_range(ct2d, base, release_size);
            ct2d->gpu_cmd.results[0] = ct2d->dcd.allocated;
            ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
        } else {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
        }
    } break;

    case CXL_GPU_CMD_DCD_GET_INFO: {
        qemu_mutex_lock(&ct2d->dcd.lock);
        ct2d->gpu_cmd.results[0] = ct2d->device_mem_size;
        ct2d->gpu_cmd.results[1] = ct2d->dcd.enabled ? ct2d->dcd.allocated : ct2d->device_mem_size;
        ct2d->gpu_cmd.results[2] = ct2d->dcd.enabled ? ct2d->device_mem_size - ct2d->dcd.allocated : 0;
        ct2d->gpu_cmd.results[3] = cxl_type2_dcd_active_extents_locked(ct2d);
        if (ct2d->gpu_cmd.data_size >= sizeof(ct2d->dcd.extents)) {
            memcpy(ct2d->gpu_cmd.data, ct2d->dcd.extents, sizeof(ct2d->dcd.extents));
        }
        qemu_mutex_unlock(&ct2d->dcd.lock);
        ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
    } break;

    case CXL_GPU_CMD_GFAM_GRANT: {
        uint32_t host = ct2d->gpu_cmd.params[0];
        uint64_t base = ct2d->gpu_cmd.params[1];
        uint64_t grant_size = ct2d->gpu_cmd.params[2];
        uint32_t permissions = ct2d->gpu_cmd.params[3];
        int ret;

        ret = cxl_type2_gfam_grant(ct2d, host, base, grant_size, permissions);
        ct2d->gpu_cmd.cmd_result = ret == 0 ? CXL_GPU_SUCCESS : CXL_GPU_ERROR_INVALID_VALUE;
    } break;

    case CXL_GPU_CMD_GFAM_REVOKE: {
        uint32_t host = ct2d->gpu_cmd.params[0];
        uint64_t base = ct2d->gpu_cmd.params[1];
        uint64_t revoke_size = ct2d->gpu_cmd.params[2];
        int ret;

        ret = cxl_type2_gfam_revoke(ct2d, host, base, revoke_size);
        ct2d->gpu_cmd.cmd_result = ret == 0 ? CXL_GPU_SUCCESS : CXL_GPU_ERROR_INVALID_VALUE;
    } break;

    case CXL_GPU_CMD_GFAM_GET_INFO: {
        qemu_mutex_lock(&ct2d->gfam.lock);
        ct2d->gpu_cmd.results[0] = ct2d->gfam.num_hosts;
        ct2d->gpu_cmd.results[1] = cxl_type2_gfam_active_mappings_locked(ct2d);
        ct2d->gpu_cmd.results[2] = ct2d->gfam.allowed_accesses;
        ct2d->gpu_cmd.results[3] = ct2d->gfam.denied_accesses;
        if (ct2d->gpu_cmd.data_size >= sizeof(ct2d->gfam.mappings)) {
            memcpy(ct2d->gpu_cmd.data, ct2d->gfam.mappings, sizeof(ct2d->gfam.mappings));
        }
        qemu_mutex_unlock(&ct2d->gfam.lock);
        ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
    } break;

    case CXL_GPU_CMD_MHSLD_GET_INFO: {
        qemu_mutex_lock(&ct2d->mhsld.lock);
        ct2d->gpu_cmd.results[0] = ct2d->mhsld.num_heads;
        ct2d->gpu_cmd.results[1] = ct2d->mhsld.local_head_id;
        ct2d->gpu_cmd.results[2] = ct2d->mhsld.reads;
        ct2d->gpu_cmd.results[3] = ct2d->mhsld.writes;
        if (ct2d->gpu_cmd.data_size >= 4 * sizeof(uint64_t)) {
            uint64_t *stats = (uint64_t *)ct2d->gpu_cmd.data;

            stats[0] = ct2d->mhsld.atomics;
            stats[1] = ct2d->mhsld.conflicts;
            stats[2] = ct2d->mhsld.invalidations;
            stats[3] = ct2d->mhsld.coherency_latency_ns;
        }
        qemu_mutex_unlock(&ct2d->mhsld.lock);
        ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
    } break;

    case CXL_GPU_CMD_MHSLD_SET_HEAD: {
        uint32_t head = ct2d->gpu_cmd.params[0];

        if (head >= ct2d->mhsld.num_heads) {
            ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
        } else {
            ct2d->mhsld.local_head_id = head;
            ct2d->gpu_cmd.results[0] = head;
            ct2d->gpu_cmd.cmd_result = CXL_GPU_SUCCESS;
        }
    } break;

    default:
        ct2d->gpu_cmd.cmd_result = CXL_GPU_ERROR_INVALID_VALUE;
        break;
    }

    ct2d->gpu_cmd.cmd_status =
        jit_command && ct2d->gpu_cmd.cmd_result != SLUG_JIT_OK ? CXL_GPU_CMD_STATUS_ERROR : CXL_GPU_CMD_STATUS_COMPLETE;
    qemu_log_mask(LOG_GUEST_ERROR, "CXL GPU: cmd 0x%x done, result=%u results[0]=0x%lx\n", cmd,
                  ct2d->gpu_cmd.cmd_result, (unsigned long)ct2d->gpu_cmd.results[0]);
}

static uint64_t cxl_type2_gpu_cmd_read(void *opaque, hwaddr addr, unsigned size) {
    CXLType2State *ct2d = opaque;
    HetGPUState *hetgpu = &ct2d->gpu_info.hetgpu_state;
    uint64_t value = 0;

    if (ct2d->jit.advertised && addr >= CXL_GPU_REG_J_MAGIC && addr < CXL_GPU_REG_J_END) {
        if (addr >= CXL_GPU_REG_J_POLICY_DIGEST && addr < CXL_GPU_REG_J_POLICY_DIGEST + SLUG_JIT_DIGEST_BYTES) {
            size_t offset = addr - CXL_GPU_REG_J_POLICY_DIGEST;
            size_t available = SLUG_JIT_DIGEST_BYTES - offset;

            memcpy(&value, ct2d->jit.runtime.policy_digest + offset, MIN((size_t)size, MIN(available, sizeof(value))));
            return value;
        }

        switch (addr) {
        case CXL_GPU_REG_J_MAGIC:
            return CXL_GPU_J_MAGIC;
        case CXL_GPU_REG_J_ABI_VERSION:
            return ct2d->jit.runtime.abi_version;
        case CXL_GPU_REG_J_CAPS:
            return ct2d->jit.runtime.capabilities;
        case CXL_GPU_REG_J_STATUS:
            return ct2d->jit.runtime.status;
        case CXL_GPU_REG_J_BACKEND:
            return ct2d->jit.runtime.selected_backend;
        case CXL_GPU_REG_J_POLICY_BYTES:
            return ct2d->jit.runtime.policy_bytes;
        case CXL_GPU_REG_J_LAST_ERROR:
            return ct2d->jit.runtime.last_error;
        case CXL_GPU_REG_J_RECORD_COUNT:
            return ct2d->jit.runtime.stats.record_count;
        case CXL_GPU_REG_J_METADATA_BYTES:
            return ct2d->jit.runtime.stats.metadata_bytes;
        case CXL_GPU_REG_J_EVENT_COUNT:
            return ct2d->jit.runtime.stats.event_count;
        case CXL_GPU_REG_J_REJECT_COUNT:
            return ct2d->jit.runtime.stats.reject_count;
        case CXL_GPU_REG_J_DROP_COUNT:
            return ct2d->jit.runtime.stats.drop_count;
        case CXL_GPU_REG_J_EPOCH:
            return ct2d->jit.runtime.stats.epoch;
        default:
            return 0;
        }
    }

    switch (addr) {
    case CXL_GPU_REG_MAGIC:
        value = CXL_GPU_MAGIC;
        break;
    case CXL_GPU_REG_VERSION:
        value = CXL_GPU_VERSION;
        break;
    case CXL_GPU_REG_STATUS:
        value = ct2d->gpu_cmd.status;
        if (hetgpu->initialized) {
            value |= CXL_GPU_STATUS_READY;
        }
        if (hetgpu->context) {
            value |= CXL_GPU_STATUS_CTX_ACTIVE;
        }
        break;
    case CXL_GPU_REG_CAPS:
        value = ct2d->gpu_cmd.capabilities;
        break;
    case CXL_GPU_REG_CMD_STATUS:
        value = ct2d->gpu_cmd.cmd_status;
        // fprintf(stderr, "CXL GPU: read CMD_STATUS = %lu\n", (unsigned long)value);
        break;
    case CXL_GPU_REG_CMD_RESULT:
        value = ct2d->gpu_cmd.cmd_result;
        break;
    case CXL_GPU_REG_RESULT0:
        value = ct2d->gpu_cmd.results[0];
        // fprintf(stderr, "CXL GPU: read RESULT0 = 0x%lx\n", (unsigned long)value);
        break;
    case CXL_GPU_REG_RESULT1:
        value = ct2d->gpu_cmd.results[1];
        break;
    case CXL_GPU_REG_RESULT2:
        value = ct2d->gpu_cmd.results[2];
        break;
    case CXL_GPU_REG_RESULT3:
        value = ct2d->gpu_cmd.results[3];
        break;
    case CXL_GPU_REG_TOTAL_MEM:
        if (ct2d->dcd.enabled) {
            value = ct2d->dcd.allocated;
        } else if (hetgpu->initialized && hetgpu->props.total_memory > 0) {
            value = hetgpu->props.total_memory;
        } else {
            value = ct2d->device_mem_size;
        }
        // fprintf(stderr, "CXL GPU: read TOTAL_MEM = 0x%lx (%lu MB)\n",
        //         (unsigned long)value, (unsigned long)(value / (1024*1024)));
        break;
    case CXL_GPU_REG_FREE_MEM:
        if (ct2d->dcd.enabled) {
            value = ct2d->device_mem_size - ct2d->dcd.allocated;
        } else {
            value = ct2d->device_mem_size;
        }
        break;
    case CXL_GPU_REG_CC_MAJOR:
        value = hetgpu->initialized ? hetgpu->props.compute_capability_major : 8;
        break;
    case CXL_GPU_REG_CC_MINOR:
        value = hetgpu->initialized ? hetgpu->props.compute_capability_minor : 0;
        break;
    case CXL_GPU_REG_MP_COUNT:
        value = hetgpu->initialized ? hetgpu->props.multiprocessor_count : 80;
        break;
    case CXL_GPU_REG_MAX_THREADS:
        value = hetgpu->initialized ? hetgpu->props.max_threads_per_block : 1024;
        break;
    case CXL_GPU_REG_WARP_SIZE:
        value = hetgpu->initialized ? hetgpu->props.warp_size : 32;
        break;
    case CXL_GPU_REG_BACKEND:
        value = hetgpu->backend;
        break;
    /* Coherent pool registers */
    case CXL_GPU_REG_COH_POOL_BASE:
        value = ct2d->coherent_pool.base_offset;
        break;
    case CXL_GPU_REG_COH_POOL_SIZE:
        value = ct2d->coherent_pool.size;
        break;
    case CXL_GPU_REG_COH_POOL_FREE:
        value = ct2d->coherent_pool.size - ct2d->coherent_pool.used;
        break;
    case CXL_GPU_REG_COH_DIR_SIZE:
        value = ct2d->bar_coherency.snoop_filter_capacity;
        break;
    case CXL_GPU_REG_COH_DIR_USED:
        value = ct2d->bar_coherency.snoop_filter_size;
        break;
    case CXL_GPU_REG_DCD_TOTAL:
        value = ct2d->device_mem_size;
        break;
    case CXL_GPU_REG_DCD_ALLOCATED:
        value = ct2d->dcd.enabled ? ct2d->dcd.allocated : ct2d->device_mem_size;
        break;
    case CXL_GPU_REG_DCD_FREE:
        value = ct2d->dcd.enabled ? ct2d->device_mem_size - ct2d->dcd.allocated : 0;
        break;
    case CXL_GPU_REG_DCD_EXTENTS:
        qemu_mutex_lock(&ct2d->dcd.lock);
        value = cxl_type2_dcd_active_extents_locked(ct2d);
        qemu_mutex_unlock(&ct2d->dcd.lock);
        break;
    case CXL_GPU_REG_GFAM_HOSTS:
        value = ct2d->gfam.num_hosts;
        break;
    case CXL_GPU_REG_GFAM_MAPPINGS:
        qemu_mutex_lock(&ct2d->gfam.lock);
        value = cxl_type2_gfam_active_mappings_locked(ct2d);
        qemu_mutex_unlock(&ct2d->gfam.lock);
        break;
    case CXL_GPU_REG_GFAM_DENIED:
        value = ct2d->gfam.denied_accesses;
        break;
    case CXL_GPU_REG_MHSLD_HEADS:
        value = ct2d->mhsld.num_heads;
        break;
    case CXL_GPU_REG_MHSLD_HEAD_ID:
        value = ct2d->mhsld.local_head_id;
        break;
    case CXL_GPU_REG_MHSLD_CONFLICTS:
        value = ct2d->mhsld.conflicts;
        break;
    case CXL_GPU_REG_MHSLD_INV:
        value = ct2d->mhsld.invalidations;
        break;

    default:
        /* Data region */
        if (addr >= CXL_GPU_DATA_OFFSET && addr < CXL_GPU_DATA_OFFSET + CXL_GPU_DATA_SIZE) {
            size_t offset = addr - CXL_GPU_DATA_OFFSET;
            if (offset + size <= ct2d->gpu_cmd.data_size) {
                memcpy(&value, &ct2d->gpu_cmd.data[offset], MIN(size, 8));
            }
        } else if (addr >= CXL_GPU_REG_DEV_NAME && addr < CXL_GPU_REG_DEV_NAME + 64) {
            /* Device name */
            size_t offset = addr - CXL_GPU_REG_DEV_NAME;
            if (hetgpu->initialized) {
                memcpy(&value, &hetgpu->props.name[offset], MIN(size, 8));
            }
        }
        break;
    }

    return value;
}

static void cxl_type2_gpu_cmd_write(void *opaque, hwaddr addr, uint64_t value, unsigned size) {
    CXLType2State *ct2d = opaque;

    switch (addr) {
    case CXL_GPU_REG_CMD:
        /* Execute command */
        cxl_type2_gpu_execute_cmd(ct2d, value);
        break;
    case CXL_GPU_REG_PARAM0:
        ct2d->gpu_cmd.params[0] = value;
        break;
    case CXL_GPU_REG_PARAM1:
        ct2d->gpu_cmd.params[1] = value;
        break;
    case CXL_GPU_REG_PARAM2:
        ct2d->gpu_cmd.params[2] = value;
        break;
    case CXL_GPU_REG_PARAM3:
        ct2d->gpu_cmd.params[3] = value;
        break;
    case CXL_GPU_REG_PARAM4:
        ct2d->gpu_cmd.params[4] = value;
        break;
    case CXL_GPU_REG_PARAM5:
        ct2d->gpu_cmd.params[5] = value;
        break;
    case CXL_GPU_REG_PARAM6:
        ct2d->gpu_cmd.params[6] = value;
        break;
    case CXL_GPU_REG_PARAM7:
        ct2d->gpu_cmd.params[7] = value;
        break;
    default:
        /* Data region */
        if (addr >= CXL_GPU_DATA_OFFSET && addr < CXL_GPU_DATA_OFFSET + CXL_GPU_DATA_SIZE) {
            size_t offset = addr - CXL_GPU_DATA_OFFSET;
            if (offset + size <= ct2d->gpu_cmd.data_size) {
                memcpy(&ct2d->gpu_cmd.data[offset], &value, MIN(size, 8));
            }
        }
        break;
    }
}

/* GPU command ops handled directly in cache_read/cache_write */

/* ========================================================================
 * DVSEC Configuration
 * ======================================================================== */

static void cxl_type2_bi_control_write(CXLComponentState *cxl_cstate, uint32_t old_ctrl, uint32_t new_ctrl,
                                       void *opaque) {
    CXLType2State *ct2d = opaque;
    bool old_enabled = FIELD_EX32(old_ctrl, CXL_BI_DECODER_CTRL, BI_ENABLE);
    bool new_enabled = FIELD_EX32(new_ctrl, CXL_BI_DECODER_CTRL, BI_ENABLE);

    if (old_enabled == new_enabled) {
        return;
    }

    ct2d->bi_enabled = new_enabled;
    cxl_type2_memsim_request(ct2d, new_enabled ? CXL_OP_BI_ENABLE : CXL_OP_BI_DISABLE, 0, ct2d->device_mem_size, NULL,
                             NULL);
    qemu_log("CXL Type2: BI %s via decoder control\n", new_enabled ? "enabled" : "disabled");
}

static void build_dvsecs(CXLType2State *ct2d) {
    CXLComponentState *cxl_cstate = &ct2d->cxl_cstate;
    uint8_t *dvsec;

    /* Type 2 Device DVSEC - includes both cache and memory capabilities */
    dvsec = (uint8_t *)&(CXLDVSECDevice){
        .cap = 0x1f, /* Cache+, IO+, Mem+, Mem HWInit+, HDMCount=1 */
        .ctrl = 0x7, /* Cache+, IO+, Mem+ enabled */
        .status = 0,
        .ctrl2 = 0,
        .status2 = 0x2,
        .lock = 0,
        .cap2 = ((((ct2d->cache_size / MiB) & 0xff) << 8) | 2),
        .range1_size_hi = ct2d->device_mem_size >> 32,
        .range1_size_lo = (2U << 5) | (2U << 2) | 0x3U | (ct2d->device_mem_size & 0xf0000000U),
        .range1_base_hi = 0,
        .range1_base_lo = 0,
        .range2_size_hi = 0,
        .range2_size_lo = 0,
        .range2_base_hi = 0,
        .range2_base_lo = 0,
    };

    cxl_component_create_dvsec(cxl_cstate, CXL2_TYPE3_DEVICE, PCIE_CXL_DEVICE_DVSEC_LENGTH, PCIE_CXL_DEVICE_DVSEC,
                               PCIE_CXL31_DEVICE_DVSEC_REVID, dvsec);

    /* BAR2 remains cache memory; the device-register block follows BAR0. */
    dvsec = (uint8_t *)&(CXLDVSECRegisterLocator){
        .rsvd = 0,
        .reg0_base_lo = RBI_COMPONENT_REG | CXL_COMPONENT_REG_BAR_IDX,
        .reg0_base_hi = 0,
        .reg1_base_lo = CXL_TYPE2_DEVICE_REG_OFFSET | RBI_CXL_DEVICE_REG | CXL_COMPONENT_REG_BAR_IDX,
        .reg1_base_hi = 0,
    };

    cxl_component_create_dvsec(cxl_cstate, CXL2_TYPE3_DEVICE, REG_LOC_DVSEC_LENGTH, REG_LOC_DVSEC, REG_LOC_DVSEC_REVID,
                               dvsec);

    /* FlexBus Port DVSEC */
    dvsec = (uint8_t *)&(CXLDVSECPortFlexBus){
        .cap = 0x26,
        .ctrl = 0x02,
        .status = ct2d->flitmode ? 0x6 : 0x26,
        .rcvd_mod_ts_data_phase1 = 0xef,
    };

    cxl_component_create_dvsec(cxl_cstate, CXL2_TYPE3_DEVICE, PCIE_CXL3_FLEXBUS_PORT_DVSEC_LENGTH,
                               PCIE_FLEXBUS_PORT_DVSEC, PCIE_CXL3_FLEXBUS_PORT_DVSEC_REVID, dvsec);
}

/* ========================================================================
 * Device Lifecycle
 * ======================================================================== */

static bool cxl_type2_jit_backend(CXLType2State *ct2d, uint32_t *backend, Error **errp) {
    const char *mode = ct2d->jit.mode ?: "off";

    if (strcmp(mode, "off") == 0) {
        *backend = SLUG_JIT_BACKEND_NONE;
        return true;
    }
    if (strcmp(mode, "rust") == 0) {
        *backend = SLUG_JIT_BACKEND_RUST;
        return true;
    }
    if (strcmp(mode, "gpu") == 0) {
        *backend = SLUG_JIT_BACKEND_GPU;
        return true;
    }
    if (strcmp(mode, "fpga-verilator") == 0) {
        *backend = SLUG_JIT_BACKEND_FPGA_VERILATOR;
        return true;
    }
    if (strcmp(mode, "auto") == 0) {
        error_setg(errp, "slugarch-j-ext=auto has no eligible probed backend; "
                         "select rust or fpga-verilator explicitly");
        return false;
    }

    error_setg(errp, "slugarch-j-ext must be off, auto, rust, gpu, "
                     "or fpga-verilator");
    return false;
}

static bool cxl_type2_jit_realize(CXLType2State *ct2d, Error **errp) {
    CXLType2JitState *runtime = &ct2d->jit.runtime;
    uint32_t backend;

    ct2d->jit.advertised = false;
    if (!cxl_type2_jit_backend(ct2d, &backend, errp)) {
        return false;
    }

    cxl_type2_jit_close(runtime);
    cxl_type2_jit_state_init(runtime, backend, ct2d->jit.strict, 16 * KiB);
    if (backend == SLUG_JIT_BACKEND_NONE) {
        return true;
    }
    if (!ct2d->jit.library_path || !ct2d->jit.policy_path) {
        error_setg(errp, "slugarch-jit-lib and slugarch-jit-policy are required "
                         "when the J-extension is enabled");
        return false;
    }
    if (!cxl_type2_jit_open(runtime, ct2d->jit.library_path, errp) ||
        !cxl_type2_jit_load_policy_file(runtime, ct2d->jit.policy_path, errp) ||
        (ct2d->jit.log_path && !cxl_type2_jit_open_log(runtime, ct2d->jit.log_path, errp))) {
        error_prepend(errp, "SlugArch J-extension initialization failed: ");
        cxl_type2_jit_close(runtime);
        return false;
    }

    ct2d->jit.advertised = true;
    return true;
}

static void cxl_type2_reset(DeviceState *dev) {
    CXLType2State *ct2d = CXL_TYPE2(dev);
    CXLComponentState *cxl_cstate = &ct2d->cxl_cstate;
    uint32_t *reg_state = cxl_cstate->crb.cache_mem_registers;
    uint32_t *write_msk = cxl_cstate->crb.cache_mem_regs_write_mask;

    pcie_cap_fill_link_ep_usp(PCI_DEVICE(dev), ct2d->width, ct2d->speed, ct2d->flitmode);
    cxl_component_register_init_common(reg_state, write_msk, CXL2_TYPE3_DEVICE, ct2d->hdmdb);
    cxl_cstate->bi_control_write = cxl_type2_bi_control_write;
    cxl_cstate->bi_control_opaque = ct2d;
    ct2d->bi_enabled = false;
    if (ct2d->cci.initialized) {
        cxl_destroy_cci(&ct2d->cci);
    }
    cxl_device_register_init_t2(ct2d, 0);

    /* Reset statistics */
    memset(&ct2d->stats, 0, sizeof(ct2d->stats));

    qemu_log("CXL Type2: Device reset\n");
}

static void cxl_type2_realize(PCIDevice *pci_dev, Error **errp) {
    CXLType2State *ct2d = CXL_TYPE2(pci_dev);
    CXLComponentState *cxl_cstate = &ct2d->cxl_cstate;
    Error *local_err = NULL;

    pci_config_set_prog_interface(pci_dev->config, 0x10);

    /* Set default values */
    if (!ct2d->memsim.server_addr) {
        ct2d->memsim.server_addr = g_strdup("127.0.0.1");
    }
    if (ct2d->memsim.server_port == 0) {
        ct2d->memsim.server_port = 9999;
    }
    if (ct2d->cache_size == 0) {
        ct2d->cache_size = CXL_TYPE2_DEFAULT_CACHE_SIZE;
    }
    if (ct2d->device_mem_size == 0) {
        ct2d->device_mem_size = CXL_TYPE2_DEFAULT_MEM_SIZE;
    }
    if (ct2d->memsim_v2.enabled) {
        uint32_t cache_lines = ct2d->memsim_v2.cache_capacity / CXL_MEMSIM_V2_LINE_SIZE;

        if (ct2d->slugarch.enabled) {
            error_setg(errp, "coherence-v2 and sync-type2-wire are mutually exclusive");
            return;
        }
        if (ct2d->device_mem_size != 256 * MiB) {
            error_setg(errp, "coherence-v2 requires mem-size=268435456");
            return;
        }
        if (ct2d->memsim_v2.host_endpoint >= CXL_MEMSIM_V2_MAX_ENDPOINTS ||
            ct2d->memsim_v2.device_endpoint >= CXL_MEMSIM_V2_MAX_ENDPOINTS ||
            ct2d->memsim_v2.host_endpoint == ct2d->memsim_v2.device_endpoint) {
            error_setg(errp, "coherence-v2 host/device endpoints must be distinct values below 64");
            return;
        }
        if (ct2d->memsim_v2.cache_capacity < CXL_MEMSIM_V2_LINE_SIZE ||
            ct2d->memsim_v2.cache_capacity % CXL_MEMSIM_V2_LINE_SIZE || !ct2d->memsim_v2.cache_ways ||
            cache_lines % ct2d->memsim_v2.cache_ways || !ct2d->memsim_v2.timeout_ms) {
            error_setg(errp, "invalid coherence-v2 cache geometry or timeout");
            return;
        }
    }
    if (ct2d->cache_size < MiB || ct2d->cache_size > 255 * MiB || ct2d->cache_size % MiB != 0) {
        error_setg(errp, "cache-size must be an integral value from "
                         "1 MiB through 255 MiB");
        return;
    }
    if (!cxl_type2_jit_realize(ct2d, errp)) {
        return;
    }

    if (ct2d->hdmdb && !ct2d->flitmode) {
        error_setg(errp, "hdm-db requires operating in 256B flit mode");
        return;
    }

    /* Initialize Type2 CXL.mem fabric feature models. */
    cxl_type2_fabric_features_init(ct2d);

    /* Initialize coherency protocol */
    cxl_type2_coherency_init(ct2d);

    /* Initialize enhanced BAR coherency tracking */
    cxl_bar_coherency_init(&ct2d->bar_coherency);

    /* Initialize P2P DMA engine */
    cxl_p2p_dma_init(&ct2d->p2p_engine, ct2d);

    /* Initialize CXLMemSim connection */
    qemu_mutex_init(&ct2d->memsim.lock);
    ct2d->slugarch.lock_initialized = true;

    /* Setup PCIe capabilities */
    pcie_endpoint_cap_init(pci_dev, 0x80);
    if (ct2d->sn != 0) {
        pcie_dev_ser_num_init(pci_dev, 0x100, ct2d->sn);
        cxl_cstate->dvsec_offset = 0x100 + 0x0c;
    } else {
        cxl_cstate->dvsec_offset = 0x100;
    }

    ct2d->cxl_cstate.pdev = pci_dev;
    build_dvsecs(ct2d);

    cxl_component_register_block_init(OBJECT(pci_dev), cxl_cstate, TYPE_CXL_TYPE2);

    ct2d->cxl_dstate.static_mem_size = ct2d->device_mem_size;
    ct2d->cxl_dstate.vmem_size = ct2d->device_mem_size;
    ct2d->cxl_dstate.pmem_size = 0;
    cxl_device_register_block_init(OBJECT(pci_dev), &ct2d->cxl_dstate, &ct2d->cci);

    /* BAR0: component registers followed by Type-2 device registers. */
    memory_region_init(&ct2d->bar0, OBJECT(ct2d), "cxl-type2-bar0", CXL_TYPE2_BAR0_SIZE);
    memory_region_add_subregion(&ct2d->bar0, 0, &cxl_cstate->crb.component_registers);
    memory_region_add_subregion(&ct2d->bar0, CXL_TYPE2_DEVICE_REG_OFFSET, &ct2d->cxl_dstate.device_registers);

    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64, &ct2d->bar0);

    /* BAR2: Cache memory region (Type 1 feature) */
    memory_region_init_ram(&ct2d->cache_mem, OBJECT(ct2d), "cxl-type2-cache", ct2d->cache_size, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    memory_region_init_io(&ct2d->cache_io, OBJECT(ct2d), &cxl_type2_cache_ops, ct2d, "cxl-type2-cache-io",
                          ct2d->cache_size);

    memory_region_add_subregion_overlap(&ct2d->cache_mem, 0, &ct2d->cache_io, 1);

    pci_register_bar(pci_dev, 2,
                     PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64 | PCI_BASE_ADDRESS_MEM_PREFETCH,
                     &ct2d->cache_mem);

    /* BAR4: Device-attached memory (Type 3 feature) */
    memory_region_init_ram(&ct2d->device_mem, OBJECT(ct2d), "cxl-type2-device-mem", ct2d->device_mem_size, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }

    /* I/O overlay for per-access coherency tracking on BAR4.
     * The device_mem_read/write handlers fast-path the bulk staging region
     * (offset 0..64MB) and coherent pool (top of BAR4) to avoid coherency
     * overhead on high-throughput regions. Normal device memory goes through
     * full cache coherency tracking. */
    memory_region_init_io(&ct2d->device_mem_io, OBJECT(ct2d), &cxl_type2_device_mem_ops, ct2d,
                          "cxl-type2-device-mem-io", ct2d->device_mem_size);
    memory_region_add_subregion_overlap(&ct2d->device_mem, 0, &ct2d->device_mem_io, 1);

    ct2d->bulk_transfer_size = MIN(CXL_GPU_BULK_TRANSFER_SIZE, ct2d->device_mem_size);

    if (ct2d->slugarch.enabled) {
        ct2d->bulk_transfer_ptr = g_malloc0(ct2d->bulk_transfer_size);
        memory_region_init_io(&ct2d->bulk_transfer_region, OBJECT(ct2d), &cxl_type2_bulk_proxy_ops, ct2d,
                              "cxl-type2-bulk-staging-proxy", ct2d->bulk_transfer_size);
    } else {
        /*
         * Legacy mode keeps the priority-2 RAM fast path with no vmexits.
         */
        memory_region_init_ram(&ct2d->bulk_transfer_region, OBJECT(ct2d), "cxl-type2-bulk-staging",
                               ct2d->bulk_transfer_size, &local_err);
        if (local_err) {
            error_propagate(errp, local_err);
            return;
        }
        ct2d->bulk_transfer_ptr = memory_region_get_ram_ptr(&ct2d->bulk_transfer_region);
    }
    memory_region_add_subregion_overlap(&ct2d->device_mem, 0, &ct2d->bulk_transfer_region, 2);

    pci_register_bar(pci_dev, 4,
                     PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64 | PCI_BASE_ADDRESS_MEM_PREFETCH,
                     &ct2d->device_mem);

    /* Register BAR regions for enhanced coherency tracking */
    cxl_bar_coherency_add_region(&ct2d->bar_coherency, 2, 0, ct2d->cache_size, true, /* GPU accessible */
                                 true); /* CPU accessible */
    cxl_bar_coherency_add_region(&ct2d->bar_coherency, 4, 0, ct2d->device_mem_size, true, /* GPU accessible */
                                 true); /* CPU accessible */

    /* Initialize GPU command state (handled directly in cache_read/write) */
    memset(&ct2d->gpu_cmd, 0, sizeof(ct2d->gpu_cmd));
    /* NOTE: Do NOT set CXL_GPU_STATUS_READY here - it will be set dynamically
     * in the register read handler based on hetgpu->initialized.
     * Setting it here unconditionally would make cuInit succeed even when
     * the GPU backend failed to initialize. */
    ct2d->gpu_cmd.status = 0;
    ct2d->gpu_cmd.cmd_status = CXL_GPU_CMD_STATUS_IDLE;

    /* Allocate larger data buffer for optimized transfers (1MB) */
    ct2d->gpu_cmd.data_size = CXL_GPU_DATA_SIZE; /* 1MB */
    ct2d->gpu_cmd.data = g_malloc0(ct2d->gpu_cmd.data_size);
    if (!ct2d->gpu_cmd.data) {
        error_setg(errp, "Failed to allocate GPU command data buffer");
        return;
    }

    /* Set capabilities */
    ct2d->gpu_cmd.capabilities =
        CXL_GPU_CAP_BULK_TRANSFER | CXL_GPU_CAP_CACHE_COHERENT | CXL_GPU_CAP_COHERENT_POOL | CXL_GPU_CAP_DEVICE_BIAS;
    if (ct2d->jit.advertised) {
        ct2d->gpu_cmd.capabilities |= CXL_GPU_CAP_SLUGARCH_J_EXT;
    }
    if (ct2d->dcd.enabled) {
        ct2d->gpu_cmd.capabilities |= CXL_GPU_CAP_DCD;
    }
    if (ct2d->gfam.enabled) {
        ct2d->gpu_cmd.capabilities |= CXL_GPU_CAP_GFAM;
    }
    if (ct2d->mhsld.enabled) {
        ct2d->gpu_cmd.capabilities |= CXL_GPU_CAP_MHSLD;
    }

    /* Initialize coherent shared memory pool at top of BAR4 */
    {
        uint64_t coh_pool_size = 256 * MiB; /* Default 256MB */
        if (coh_pool_size > ct2d->device_mem_size / 2) {
            coh_pool_size = ct2d->device_mem_size / 4;
        }
        ct2d->coherent_pool.size = coh_pool_size;
        ct2d->coherent_pool.base_offset = ct2d->device_mem_size - coh_pool_size;
        ct2d->coherent_pool.used = 0;
        ct2d->coherent_pool.allocations = g_hash_table_new(g_int64_hash, g_int64_equal);
        /* Initialize free list with single block spanning the whole pool */
        CXLCohFreeBlock *initial = g_new0(CXLCohFreeBlock, 1);
        initial->offset = ct2d->coherent_pool.base_offset;
        initial->size = coh_pool_size;
        initial->next = NULL;
        ct2d->coherent_pool.free_list = initial;
        qemu_mutex_init(&ct2d->coherent_pool.lock);

        memory_region_init_ram(&ct2d->coherent_pool_backing, OBJECT(ct2d),
                               "cxl-type2-coh-pool-backing", coh_pool_size,
                               &local_err);
        if (local_err) {
            error_propagate(errp, local_err);
            return;
        }
        ct2d->coherent_pool_host_ptr =
            memory_region_get_ram_ptr(&ct2d->coherent_pool_backing);
        memory_region_init_io(&ct2d->coherent_pool_overlay, OBJECT(ct2d),
                              &cxl_type2_coherent_proxy_ops, ct2d,
                              "cxl-type2-coh-pool-overlay", coh_pool_size);
        memory_region_add_subregion_overlap(&ct2d->device_mem, ct2d->coherent_pool.base_offset,
                                            &ct2d->coherent_pool_overlay, 2);

        qemu_log("CXL Type2: Coherent pool initialized: base=0x%lx "
                 "size=%lu MB (trapping overlay at priority 2)\n",
                 (unsigned long)ct2d->coherent_pool.base_offset,
                 (unsigned long)(coh_pool_size / MiB));
    }

    qemu_log("CXL Type2: GPU command interface enabled at BAR2 offset 0\n");
    qemu_log("CXL Type2: Data buffer size: %zu KB (optimized for large transfers)\n", ct2d->gpu_cmd.data_size / 1024);

    /* Initialize MSI-X */
    if (msix_init_exclusive_bar(pci_dev, 16, 6, NULL)) {
        error_setg(errp, "Failed to initialize MSI-X");
        return;
    }

    /* Initialize GPU passthrough */
    if (cxl_type2_gpu_init(ct2d, &local_err) < 0) {
        if (local_err) {
            error_propagate(errp, local_err);
            return;
        }
    }
    if (ct2d->gpu_info.mode == CXL_TYPE2_GPU_MODE_HETGPU &&
        ct2d->gpu_info.hetgpu_state.initialized) {
        HetGPUError gpu_err = hetgpu_register_coherent_region(
            &ct2d->gpu_info.hetgpu_state,
            ct2d->coherent_pool_host_ptr,
            ct2d->coherent_pool.size,
            &ct2d->coherent_pool_gpu_region);

        if (gpu_err != HETGPU_SUCCESS) {
            cxl_type2_gpu_cleanup(ct2d);
            error_setg(errp, "failed to register Type-2 coherent pool "
                       "with real GPU: %s",
                       hetgpu_get_error_string(gpu_err));
            return;
        }
        qemu_log("CXL Type2: Coherent pool GPU mapping: host=%p "
                 "device=0x%" PRIx64 " size=%" PRIu64 "\n",
                 ct2d->coherent_pool_gpu_region.host_ptr,
                 ct2d->coherent_pool_gpu_region.device_ptr,
                 ct2d->coherent_pool_gpu_region.size);
    }

    /* Protocol v2 owns two duplex sessions; v1 remains the legacy path. */
    if (ct2d->memsim_v2.enabled) {
        if (!cxl_type2_memsim_v2_connect(ct2d, errp)) {
            cxl_type2_gpu_cleanup(ct2d);
            return;
        }
    } else if (ct2d->slugarch.enabled) {
        if (!cxl_type2_slugarch_handshake(ct2d, errp)) {
            cxl_type2_gpu_cleanup(ct2d);
            return;
        }
    } else {
        cxlmemsim_connect(ct2d);
    }

    qemu_log("CXL Type2: Device realized - Cache: %zu MB, DevMem: %zu MB\n", ct2d->cache_size / MiB,
             ct2d->device_mem_size / MiB);
}

static void cxl_type2_exit(PCIDevice *pci_dev) {
    CXLType2State *ct2d = CXL_TYPE2(pci_dev);
    Error *local_err = NULL;

    if (ct2d->slugarch.enabled && ct2d->slugarch.event_log) {
        qemu_mutex_lock(&ct2d->memsim.lock);
        if (!cxl_type2_slugarch_emit_path_counters_locked(ct2d, &local_err)) {
            error_report_err(local_err);
        }
        qemu_mutex_unlock(&ct2d->memsim.lock);
    }
    /* Disconnect from CXLMemSim */
    cxl_type2_memsim_v2_disconnect(ct2d);
    cxlmemsim_disconnect(ct2d);
    cxl_type2_slugarch_close_event_log(ct2d);
    qemu_mutex_destroy(&ct2d->memsim.lock);
    ct2d->slugarch.lock_initialized = false;

    /* Cleanup GPU passthrough */
    cxl_type2_gpu_cleanup(ct2d);
    cxl_type2_jit_close(&ct2d->jit.runtime);
    ct2d->jit.advertised = false;

    /* Cleanup coherency protocol */
    cxl_type2_coherency_cleanup(ct2d);

    /* Cleanup enhanced BAR coherency tracking */
    cxl_bar_coherency_cleanup(&ct2d->bar_coherency);

    /* Cleanup P2P DMA engine */
    cxl_p2p_dma_cleanup(&ct2d->p2p_engine);

    if (ct2d->cci.initialized) {
        cxl_destroy_cci(&ct2d->cci);
    }

    /* Cleanup coherent pool */
    if (ct2d->coherent_pool.allocations) {
        g_hash_table_destroy(ct2d->coherent_pool.allocations);
        ct2d->coherent_pool.allocations = NULL;
    }
    {
        CXLCohFreeBlock *blk = ct2d->coherent_pool.free_list;
        while (blk) {
            CXLCohFreeBlock *next = blk->next;
            g_free(blk);
            blk = next;
        }
        ct2d->coherent_pool.free_list = NULL;
    }
    qemu_mutex_destroy(&ct2d->coherent_pool.lock);

    /* Free GPU command data buffer */
    if (ct2d->gpu_cmd.data) {
        g_free(ct2d->gpu_cmd.data);
        ct2d->gpu_cmd.data = NULL;
    }

    ct2d->coherent_pool_host_ptr = NULL;
    if (ct2d->slugarch.enabled) {
        g_clear_pointer(&ct2d->bulk_transfer_ptr, g_free);
    } else {
        /*
         * Legacy pointers belong to QEMU RAM regions and are not g_malloc().
         */
        ct2d->bulk_transfer_ptr = NULL;
    }

    cxl_type2_fabric_features_cleanup(ct2d);

    qemu_log("CXL Type2: Device exit complete\n");
}

static const Property cxl_type2_props[] = {
    DEFINE_PROP_SIZE("cache-size", CXLType2State, cache_size, CXL_TYPE2_DEFAULT_CACHE_SIZE),
    DEFINE_PROP_SIZE("mem-size", CXLType2State, device_mem_size, CXL_TYPE2_DEFAULT_MEM_SIZE),
    DEFINE_PROP_UINT64("sn", CXLType2State, sn, 0),
    DEFINE_PROP_PCIE_LINK_SPEED("x-speed", CXLType2State, speed, PCIE_LINK_SPEED_64),
    DEFINE_PROP_PCIE_LINK_WIDTH("x-width", CXLType2State, width, PCIE_LINK_WIDTH_32),
    DEFINE_PROP_BOOL("x-256b-flit", CXLType2State, flitmode, true),
    DEFINE_PROP_BOOL("hdm-db", CXLType2State, hdmdb, true),
    DEFINE_PROP_STRING("cxlmemsim-addr", CXLType2State, memsim.server_addr),
    DEFINE_PROP_UINT16("cxlmemsim-port", CXLType2State, memsim.server_port, 9999),
    DEFINE_PROP_BOOL("coherence-v2", CXLType2State, memsim_v2.enabled, false),
    DEFINE_PROP_UINT16("coherence-v2-host-endpoint", CXLType2State, memsim_v2.host_endpoint,
                       CXL_MEMSIM_V2_HOST_ENDPOINT),
    DEFINE_PROP_UINT16("coherence-v2-device-endpoint", CXLType2State, memsim_v2.device_endpoint,
                       CXL_MEMSIM_V2_DEVICE_ENDPOINT),
    DEFINE_PROP_UINT32("coherence-v2-cache-capacity", CXLType2State, memsim_v2.cache_capacity, 256 * KiB),
    DEFINE_PROP_UINT16("coherence-v2-cache-ways", CXLType2State, memsim_v2.cache_ways, 4),
    DEFINE_PROP_UINT32("coherence-v2-timeout-ms", CXLType2State, memsim_v2.timeout_ms, 2000),
    DEFINE_PROP_BOOL("coherence-v2-write-through", CXLType2State, memsim_v2.write_through, false),
    DEFINE_PROP_BOOL("sync-type2-wire", CXLType2State, slugarch.enabled, false),
    DEFINE_PROP_UINT16("type2-wire-version", CXLType2State, slugarch.wire_version, SLUGARCH_T2_VERSION),
    DEFINE_PROP_STRING("slugarch-event-log", CXLType2State, slugarch.event_log_path),
    DEFINE_PROP_BOOL("slugarch-shadow-after-write", CXLType2State, slugarch.shadow_after_write, false),
    DEFINE_PROP_STRING("slugarch-j-ext", CXLType2State, jit.mode),
    DEFINE_PROP_STRING("slugarch-jit-lib", CXLType2State, jit.library_path),
    DEFINE_PROP_STRING("slugarch-jit-policy", CXLType2State, jit.policy_path),
    DEFINE_PROP_STRING("slugarch-jit-log", CXLType2State, jit.log_path),
    DEFINE_PROP_BOOL("slugarch-jit-strict", CXLType2State, jit.strict, true),
    DEFINE_PROP_STRING("gpu-device", CXLType2State, gpu_info.vfio_device),
    DEFINE_PROP_BOOL("coherency-enabled", CXLType2State, coherency.coherency_enabled, true),
    /* hetGPU backend configuration */
    DEFINE_PROP_UINT32("gpu-mode", CXLType2State, gpu_info.mode, CXL_TYPE2_GPU_MODE_AUTO),
    DEFINE_PROP_STRING("hetgpu-lib", CXLType2State, gpu_info.hetgpu_lib_path),
    DEFINE_PROP_INT32("hetgpu-device", CXLType2State, gpu_info.hetgpu_device_index, 0),
    DEFINE_PROP_UINT32("hetgpu-backend", CXLType2State, gpu_info.hetgpu_backend, HETGPU_BACKEND_AUTO),
    DEFINE_PROP_BOOL("dcd", CXLType2State, dcd.enabled, false),
    DEFINE_PROP_SIZE("dcd-granularity", CXLType2State, dcd.granularity, CXL_TYPE2_DCD_DEFAULT_GRANULARITY),
    DEFINE_PROP_SIZE("dcd-initial-size", CXLType2State, dcd.initial_size, CXL_TYPE2_DCD_INIT_AUTO),
    DEFINE_PROP_BOOL("gfam", CXLType2State, gfam.enabled, false),
    DEFINE_PROP_UINT32("gfam-hosts", CXLType2State, gfam.num_hosts, 1),
    DEFINE_PROP_UINT32("gfam-host-id", CXLType2State, gfam.local_host_id, 0),
    DEFINE_PROP_UINT32("gfam-default-perms", CXLType2State, gfam.default_permissions, CXL_DCD_PERM_ALL),
    DEFINE_PROP_UINT32("gfam-latency-ns", CXLType2State, gfam.fabric_latency_ns, 150),
    DEFINE_PROP_UINT32("gfam-bandwidth-mbps", CXLType2State, gfam.bandwidth_mbps, 32768),
    DEFINE_PROP_BOOL("mhsld", CXLType2State, mhsld.enabled, false),
    DEFINE_PROP_UINT32("mhsld-heads", CXLType2State, mhsld.num_heads, 1),
    DEFINE_PROP_UINT32("mhsld-head-id", CXLType2State, mhsld.local_head_id, 0),
    DEFINE_PROP_UINT32("mhsld-coh-latency-ns", CXLType2State, mhsld.coherency_latency_ns, 200),
};

static bool cxl_type2_slugarch_phase_valid(const char *phase) {
    size_t length;
    size_t i;

    if (!phase) {
        return false;
    }
    length = strlen(phase);
    if (length < 1 || length > 96) {
        return false;
    }
    for (i = 0; i < length; i++) {
        if (!g_ascii_isalnum(phase[i]) && phase[i] != '_' && phase[i] != '.' && phase[i] != ':' && phase[i] != '-') {
            return false;
        }
    }
    return true;
}

static char *cxl_type2_slugarch_get_phase(Object *obj, Error **errp) {
    CXLType2State *ct2d = CXL_TYPE2(obj);
    char *phase;

    if (ct2d->slugarch.lock_initialized) {
        qemu_mutex_lock(&ct2d->memsim.lock);
    }
    phase = g_strdup(ct2d->slugarch.phase_id ? ct2d->slugarch.phase_id : "idle");
    if (ct2d->slugarch.lock_initialized) {
        qemu_mutex_unlock(&ct2d->memsim.lock);
    }
    return phase;
}

static void cxl_type2_slugarch_set_phase(Object *obj, const char *value, Error **errp) {
    CXLType2State *ct2d = CXL_TYPE2(obj);
    Error *local_err = NULL;

    if (!cxl_type2_slugarch_phase_valid(value)) {
        error_setg(errp, "slugarch-phase-id must match [A-Za-z0-9_.:-]{1,96}");
        return;
    }
    if (ct2d->slugarch.lock_initialized) {
        qemu_mutex_lock(&ct2d->memsim.lock);
    }
    g_free(ct2d->slugarch.phase_id);
    ct2d->slugarch.phase_id = g_strdup(value);
    if (ct2d->slugarch.event_log && !cxl_type2_slugarch_emit_path_counters_locked(ct2d, &local_err)) {
        ct2d->slugarch.connection_failed = true;
        cxl_type2_close_memsim_socket(ct2d);
        error_propagate(errp, local_err);
    }
    if (ct2d->slugarch.lock_initialized) {
        qemu_mutex_unlock(&ct2d->memsim.lock);
    }
}

static char *cxl_type2_jit_get_digest(Object *obj, Error **errp) {
    static const char hex[] = "0123456789abcdef";
    CXLType2State *ct2d = CXL_TYPE2(obj);
    char *digest = g_malloc(SLUG_JIT_DIGEST_BYTES * 2 + 1);
    size_t i;

    for (i = 0; i < SLUG_JIT_DIGEST_BYTES; i++) {
        digest[i * 2] = hex[ct2d->jit.runtime.policy_digest[i] >> 4];
        digest[i * 2 + 1] = hex[ct2d->jit.runtime.policy_digest[i] & 0xf];
    }
    digest[SLUG_JIT_DIGEST_BYTES * 2] = '\0';
    return digest;
}

static bool cxl_type2_jit_get_advertised(Object *obj, Error **errp) { return CXL_TYPE2(obj)->jit.advertised; }

static bool cxl_type2_coherent_pool_gpu_registered(Object *obj,
                                                   Error **errp)
{
    return CXL_TYPE2(obj)->coherent_pool_gpu_region.host_registered;
}

static void cxl_type2_instance_init(Object *obj) {
    CXLType2State *ct2d = CXL_TYPE2(obj);

    cxl_type2_jit_state_init(&ct2d->jit.runtime, SLUG_JIT_BACKEND_NONE, true, 16 * KiB);
    ct2d->slugarch.phase_id = g_strdup("idle");
    object_property_add_str(obj, "slugarch-phase-id", cxl_type2_slugarch_get_phase, cxl_type2_slugarch_set_phase);
    object_property_add_uint64_ptr(obj, "slugarch-client-id", &ct2d->slugarch.client_id, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-completed-reads", &ct2d->slugarch.completed_reads,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-completed-writes", &ct2d->slugarch.completed_writes,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-read-bytes", &ct2d->slugarch.read_bytes, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-written-bytes", &ct2d->slugarch.written_bytes, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-failed-requests", &ct2d->slugarch.failed_requests,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-timed-out-requests", &ct2d->slugarch.timed_out_requests,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-partial-io-failures", &ct2d->slugarch.partial_io_failures,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-mismatched-responses", &ct2d->slugarch.mismatched_responses,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-direct-cfmws", &ct2d->slugarch.direct_cfmws_completions,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-bar4-overlay", &ct2d->slugarch.bar4_overlay_completions,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-bulk-overlay", &ct2d->slugarch.bulk_overlay_completions,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-coherent-pool", &ct2d->slugarch.coherent_pool_completions,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "coherent-pool-host-loads",
                                   &ct2d->coherent_pool_host_loads,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "coherent-pool-host-stores",
                                   &ct2d->coherent_pool_host_stores,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "coherent-pool-access-errors",
                                   &ct2d->coherent_pool_access_errors,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_bool(obj, "coherent-pool-gpu-registered",
                             cxl_type2_coherent_pool_gpu_registered, NULL);
    object_property_add_uint64_ptr(obj, "slugarch-local-shadow", &ct2d->slugarch.local_shadow_completions,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-local-cache", &ct2d->slugarch.local_cache_completions,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-delay-events", &ct2d->slugarch.delay_events, OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-delay-undershoots", &ct2d->slugarch.delay_undershoots,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "slugarch-jit-status", &ct2d->jit.runtime.status, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "slugarch-jit-backend", &ct2d->jit.runtime.selected_backend,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_str(obj, "slugarch-jit-policy-digest", cxl_type2_jit_get_digest, NULL);
    object_property_add_bool(obj, "slugarch-jit-advertised", cxl_type2_jit_get_advertised, NULL);
    object_property_add_uint64_ptr(obj, "slugarch-jit-event-count", &ct2d->jit.runtime.stats.event_count,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-jit-record-count", &ct2d->jit.runtime.stats.record_count,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-jit-metadata-bytes", &ct2d->jit.runtime.stats.metadata_bytes,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-jit-reject-count", &ct2d->jit.runtime.stats.reject_count,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-jit-drop-count", &ct2d->jit.runtime.stats.drop_count,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "slugarch-jit-epoch", &ct2d->jit.runtime.stats.epoch, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "slugarch-jit-last-error", &ct2d->jit.runtime.last_error, OBJ_PROP_FLAG_READ);
}

static void cxl_type2_instance_finalize(Object *obj) {
    CXLType2State *ct2d = CXL_TYPE2(obj);

    cxl_type2_slugarch_close_event_log(ct2d);
    cxl_type2_close_memsim_socket(ct2d);
    if (ct2d->slugarch.lock_initialized) {
        qemu_mutex_destroy(&ct2d->memsim.lock);
        ct2d->slugarch.lock_initialized = false;
    }
    if (ct2d->slugarch.enabled) {
        g_clear_pointer(&ct2d->bulk_transfer_ptr, g_free);
    }
    cxl_type2_jit_close(&ct2d->jit.runtime);
    ct2d->jit.advertised = false;
    g_clear_pointer(&ct2d->slugarch.phase_id, g_free);
}

static void cxl_type2_class_init(ObjectClass *oc, const void *data) {
    DeviceClass *dc = DEVICE_CLASS(oc);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(oc);

    pc->realize = cxl_type2_realize;
    pc->exit = cxl_type2_exit;
    pc->vendor_id = CXL_TYPE2_VENDOR_ID;
    pc->device_id = CXL_TYPE2_DEVICE_ID;
    pc->revision = 1;
    pc->class_id = PCI_CLASS_MEMORY_CXL;

    dc->desc = "CXL Type 2 Accelerator Device with Coherent Memory (GPU Passthrough)";
    device_class_set_legacy_reset(dc, cxl_type2_reset);
    device_class_set_props(dc, cxl_type2_props);
}

static const TypeInfo cxl_type2_info = {
    .name = TYPE_CXL_TYPE2,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(CXLType2State),
    .instance_init = cxl_type2_instance_init,
    .instance_finalize = cxl_type2_instance_finalize,
    .class_init = cxl_type2_class_init,
    .interfaces = (InterfaceInfo[]){{INTERFACE_PCIE_DEVICE}, {INTERFACE_CXL_DEVICE}, {}},
};

static void cxl_type2_register_types(void) { type_register_static(&cxl_type2_info); }

type_init(cxl_type2_register_types)
