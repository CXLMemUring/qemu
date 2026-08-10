/*
 * CXL Type 2 GPU Command Interface
 * Defines the command protocol between guest libcuda and host hetGPU
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef CXL_TYPE2_GPU_CMD_H
#define CXL_TYPE2_GPU_CMD_H

#include <stdint.h>

/* GPU Command Register Offsets (from BAR2 base) */
#define CXL_GPU_REG_MAGIC           0x0000  /* Magic number: 0x43584C32 "CXL2" */
#define CXL_GPU_REG_VERSION         0x0004  /* Interface version */
#define CXL_GPU_REG_STATUS          0x0008  /* Device status */
#define CXL_GPU_REG_CAPS            0x000C  /* Device capabilities */

#define CXL_GPU_REG_CMD             0x0010  /* Command register */
#define CXL_GPU_REG_CMD_STATUS      0x0014  /* Command status */
#define CXL_GPU_REG_CMD_RESULT      0x0018  /* Command result/error code */
#define CXL_GPU_REG_CMD_DATA_LO     0x001C  /* Command data low 32 bits */
#define CXL_GPU_REG_CMD_DATA_HI     0x0020  /* Command data high 32 bits */

#define CXL_GPU_REG_PARAM0          0x0040  /* Parameter 0 */
#define CXL_GPU_REG_PARAM1          0x0048  /* Parameter 1 */
#define CXL_GPU_REG_PARAM2          0x0050  /* Parameter 2 */
#define CXL_GPU_REG_PARAM3          0x0058  /* Parameter 3 */
#define CXL_GPU_REG_PARAM4          0x0060  /* Parameter 4 */
#define CXL_GPU_REG_PARAM5          0x0068  /* Parameter 5 */
#define CXL_GPU_REG_PARAM6          0x0070  /* Parameter 6 */
#define CXL_GPU_REG_PARAM7          0x0078  /* Parameter 7 */

#define CXL_GPU_REG_RESULT0         0x0080  /* Result 0 */
#define CXL_GPU_REG_RESULT1         0x0088  /* Result 1 */
#define CXL_GPU_REG_RESULT2         0x0090  /* Result 2 */
#define CXL_GPU_REG_RESULT3         0x0098  /* Result 3 */

/* Device info registers */
#define CXL_GPU_REG_DEV_NAME        0x0100  /* Device name (64 bytes) */
#define CXL_GPU_REG_TOTAL_MEM       0x0140  /* Total memory */
#define CXL_GPU_REG_FREE_MEM        0x0148  /* Free memory */
#define CXL_GPU_REG_CC_MAJOR        0x0150  /* Compute capability major */
#define CXL_GPU_REG_CC_MINOR        0x0154  /* Compute capability minor */
#define CXL_GPU_REG_MP_COUNT        0x0158  /* Multiprocessor count */
#define CXL_GPU_REG_MAX_THREADS     0x015C  /* Max threads per block */
#define CXL_GPU_REG_WARP_SIZE       0x0160  /* Warp size */
#define CXL_GPU_REG_BACKEND         0x0164  /* Backend type */

/* Data transfer region - OPTIMIZED for larger chunks */
#define CXL_GPU_DATA_OFFSET         0x1000    /* Data buffer offset */
#define CXL_GPU_DATA_SIZE           0x100000  /* Data buffer size (1MB) - was 60KB */

/* Command register size - increased to accommodate larger data buffer */
#define CXL_GPU_CMD_REG_SIZE        0x101000  /* ~1MB + 4KB registers */

/* Bulk transfer region (BAR4 direct access) */
#define CXL_GPU_BULK_TRANSFER_SIZE  0x4000000 /* 64MB bulk transfer region */

/* Capability bits */
#define CXL_GPU_CAP_BULK_TRANSFER   (1 << 0)  /* Supports bulk transfer mode */
#define CXL_GPU_CAP_CACHE_COHERENT  (1 << 1)  /* CXL.cache coherent memory */
#define CXL_GPU_CAP_DMA_ENGINE      (1 << 2)  /* Hardware DMA engine available */
#define CXL_GPU_CAP_COHERENT_POOL   (1 << 3)  /* Coherent shared memory pool */
#define CXL_GPU_CAP_DEVICE_BIAS     (1 << 4)  /* Device-biased directory mode */
#define CXL_GPU_CAP_SLUGARCH_J_EXT  (1U << 5) /* Vendor SlugArch J-extension */

/*
 * SlugArch vendor J-extension registers.  This is a local BAR2 ABI, not a
 * standardized CXL capability.
 */
#define CXL_GPU_REG_J_MAGIC          0x0400
#define CXL_GPU_REG_J_ABI_VERSION    0x0404
#define CXL_GPU_REG_J_CAPS           0x0408
#define CXL_GPU_REG_J_STATUS         0x040c
#define CXL_GPU_REG_J_BACKEND        0x0410
#define CXL_GPU_REG_J_POLICY_BYTES   0x0414
#define CXL_GPU_REG_J_LAST_ERROR     0x0418
#define CXL_GPU_REG_J_POLICY_DIGEST  0x0420
#define CXL_GPU_REG_J_RECORD_COUNT   0x0440
#define CXL_GPU_REG_J_METADATA_BYTES 0x0448
#define CXL_GPU_REG_J_EVENT_COUNT    0x0450
#define CXL_GPU_REG_J_REJECT_COUNT   0x0458
#define CXL_GPU_REG_J_DROP_COUNT     0x0460
#define CXL_GPU_REG_J_EPOCH          0x0468
#define CXL_GPU_REG_J_END            0x0500

#define CXL_GPU_J_MAGIC              0x4a474c53U /* "SLGJ", little endian */
#define CXL_GPU_J_ABI_VERSION        1U

#define CXL_GPU_J_CAP_POLICY         (1U << 0)
#define CXL_GPU_J_CAP_RECORD         (1U << 1)
#define CXL_GPU_J_CAP_GPU_DIAGNOSTIC (1U << 2)
#define CXL_GPU_J_CAP_FPGA_RTL       (1U << 3)

#define CXL_GPU_J_STATUS_DISABLED    0U
#define CXL_GPU_J_STATUS_LOADING     1U
#define CXL_GPU_J_STATUS_READY       2U
#define CXL_GPU_J_STATUS_ERROR       3U

#define CXL_GPU_J_BACKEND_NONE            0U
#define CXL_GPU_J_BACKEND_RUST            1U
#define CXL_GPU_J_BACKEND_GPU             2U
#define CXL_GPU_J_BACKEND_FPGA_VERILATOR  3U

/* Stable J-extension result codes, mirrored from ABI version 1. */
#define CXL_GPU_J_OK                        0U
#define CXL_GPU_J_ERR_NULL                  1U
#define CXL_GPU_J_ERR_STRUCT_SIZE           2U
#define CXL_GPU_J_ERR_ABI_VERSION           3U
#define CXL_GPU_J_ERR_PARSE                 4U
#define CXL_GPU_J_ERR_POLICY_VERSION        5U
#define CXL_GPU_J_ERR_TOO_MANY_INSTRUCTIONS 6U
#define CXL_GPU_J_ERR_TOO_MANY_RANGES       7U
#define CXL_GPU_J_ERR_INVALID_RANGE         8U
#define CXL_GPU_J_ERR_INVALID_STRIDE        9U
#define CXL_GPU_J_ERR_BUDGET_EXCEEDED       10U
#define CXL_GPU_J_ERR_INVALID_CONTROL_FLOW  11U
#define CXL_GPU_J_ERR_UNSUPPORTED           12U
#define CXL_GPU_J_ERR_DIGEST_MISMATCH       13U
#define CXL_GPU_J_ERR_REJECTED              14U
#define CXL_GPU_J_ERR_DROP                  15U
#define CXL_GPU_J_ERR_TIMEOUT               16U
#define CXL_GPU_J_ERR_BACKEND               17U
#define CXL_GPU_J_ERR_IO                    18U
#define CXL_GPU_J_ERR_POISONED              19U
#define CXL_GPU_J_ERR_PANIC                 20U
#define CXL_GPU_CAP_DCD             (1U << 6) /* Dynamic Capacity Device model */
#define CXL_GPU_CAP_GFAM            (1U << 7) /* Global Fabric Attached Memory */
#define CXL_GPU_CAP_MHSLD           (1U << 8) /* Multi-headed SLD coherency */

/* Magic number */
#define CXL_GPU_MAGIC               0x43584C32  /* "CXL2" */
#define CXL_GPU_VERSION             0x00010000  /* v1.0.0 */

/* Device status bits */
#define CXL_GPU_STATUS_READY        (1 << 0)
#define CXL_GPU_STATUS_BUSY         (1 << 1)
#define CXL_GPU_STATUS_ERROR        (1 << 2)
#define CXL_GPU_STATUS_CTX_ACTIVE   (1 << 3)

/* Command status */
#define CXL_GPU_CMD_STATUS_IDLE     0
#define CXL_GPU_CMD_STATUS_PENDING  1
#define CXL_GPU_CMD_STATUS_RUNNING  2
#define CXL_GPU_CMD_STATUS_COMPLETE 3
#define CXL_GPU_CMD_STATUS_ERROR    4

/* GPU Commands */
typedef enum {
    CXL_GPU_CMD_NOP             = 0x00,
    CXL_GPU_CMD_INIT            = 0x01,
    CXL_GPU_CMD_GET_DEVICE_COUNT= 0x02,
    CXL_GPU_CMD_GET_DEVICE      = 0x03,
    CXL_GPU_CMD_GET_DEVICE_NAME = 0x04,
    CXL_GPU_CMD_GET_DEVICE_PROPS= 0x05,
    CXL_GPU_CMD_GET_TOTAL_MEM   = 0x06,

    CXL_GPU_CMD_CTX_CREATE      = 0x10,
    CXL_GPU_CMD_CTX_DESTROY     = 0x11,
    CXL_GPU_CMD_CTX_SYNC        = 0x12,

    CXL_GPU_CMD_MEM_ALLOC       = 0x20,
    CXL_GPU_CMD_MEM_FREE        = 0x21,
    CXL_GPU_CMD_MEM_COPY_HTOD   = 0x22,
    CXL_GPU_CMD_MEM_COPY_DTOH   = 0x23,
    CXL_GPU_CMD_MEM_COPY_DTOD   = 0x24,
    CXL_GPU_CMD_MEM_SET         = 0x25,
    CXL_GPU_CMD_MEM_GET_INFO    = 0x26,

    CXL_GPU_CMD_MODULE_LOAD_PTX = 0x30,
    CXL_GPU_CMD_MODULE_UNLOAD   = 0x31,
    CXL_GPU_CMD_FUNC_GET        = 0x32,

    CXL_GPU_CMD_LAUNCH_KERNEL   = 0x40,

    CXL_GPU_CMD_STREAM_CREATE   = 0x50,
    CXL_GPU_CMD_STREAM_DESTROY  = 0x51,
    CXL_GPU_CMD_STREAM_SYNC     = 0x52,

    CXL_GPU_CMD_EVENT_CREATE    = 0x60,
    CXL_GPU_CMD_EVENT_DESTROY   = 0x61,
    CXL_GPU_CMD_EVENT_RECORD    = 0x62,
    CXL_GPU_CMD_EVENT_SYNC      = 0x63,

    /* Bulk transfer commands (optimized for large transfers) */
    CXL_GPU_CMD_BULK_HTOD       = 0x70,  /* Bulk host-to-device via BAR4 */
    CXL_GPU_CMD_BULK_DTOH       = 0x71,  /* Bulk device-to-host via BAR4 */
    CXL_GPU_CMD_BULK_DTOD       = 0x72,  /* Bulk device-to-device */

    /* CXL.cache coherency commands */
    CXL_GPU_CMD_CACHE_FLUSH     = 0x80,  /* Flush cache lines to device */
    CXL_GPU_CMD_CACHE_INVALIDATE= 0x81,  /* Invalidate cache lines */
    CXL_GPU_CMD_CACHE_WRITEBACK = 0x82,  /* Writeback dirty cache lines */
    CXL_GPU_CMD_COHERENT_LOAD   = 0x83,  /* Protocol-v2 device load */
    CXL_GPU_CMD_COHERENT_STORE  = 0x84,  /* Protocol-v2 device store */
    CXL_GPU_CMD_COHERENT_FAA    = 0x85,  /* Protocol-v2 fetch-and-add */
    CXL_GPU_CMD_COHERENT_CAS    = 0x86,  /* Protocol-v2 compare-and-swap */
    CXL_GPU_CMD_CACHE_PREFETCH  = 0x88,  /* Prefetch cache lines into Type2 cache */

    /* P2P DMA commands: defined as macros in cxl_p2p_dma.h (0x90-0x96) */

    /* Coherent shared memory pool commands */
    CXL_GPU_CMD_COHERENT_ALLOC      = 0xA0,  /* Allocate from coherent pool */
    CXL_GPU_CMD_COHERENT_FREE       = 0xA1,  /* Free coherent pool allocation */
    CXL_GPU_CMD_COHERENT_GET_INFO   = 0xA2,  /* Get coherent pool info */
    CXL_GPU_CMD_COHERENT_FENCE      = 0xA3,  /* Coherent memory fence */

    /* Device-biased directory commands */
    CXL_GPU_CMD_SET_BIAS            = 0xA4,  /* Set bias mode for region */
    CXL_GPU_CMD_GET_BIAS            = 0xA5,  /* Get bias mode for address */
    CXL_GPU_CMD_BIAS_FLIP           = 0xA6,  /* Flip bias with cache flush */

    /* Coherency statistics commands */
    CXL_GPU_CMD_COH_GET_STATS       = 0xB0,  /* Get coherency statistics */
    CXL_GPU_CMD_COH_RESET_STATS     = 0xB1,  /* Reset coherency statistics */
    /*
     * Acquire: P0=absolute BAR4 offset, P1=size, P2=CXLCohRangeIntent;
     *          R0=lines granted, R1=CUDA pointer, R2=endpoint, R3=session.
     * Release: P0=absolute BAR4 offset, P1=size, P2=dirty;
     *          R0=lines released, R1=endpoint, R2=session.
     */
    CXL_GPU_CMD_COH_ACQUIRE_RANGE   = 0xB2,
    CXL_GPU_CMD_COH_RELEASE_RANGE   = 0xB3,

    /* DCD/GFAM/MH-SLD fabric-memory commands */
    CXL_GPU_CMD_DCD_ADD             = 0xC0,  /* params: base, size, tag */
    CXL_GPU_CMD_DCD_RELEASE         = 0xC1,  /* params: base, size, tag */
    CXL_GPU_CMD_DCD_GET_INFO        = 0xC2,  /* results: total, alloc, free */
    CXL_GPU_CMD_GFAM_GRANT          = 0xC8,  /* params: host, base, size, perms */
    CXL_GPU_CMD_GFAM_REVOKE         = 0xC9,  /* params: host, base, size */
    CXL_GPU_CMD_GFAM_GET_INFO       = 0xCA,  /* results: hosts, mappings, deny */
    CXL_GPU_CMD_MHSLD_GET_INFO      = 0xD0,  /* results: heads, current, stats */
    CXL_GPU_CMD_MHSLD_SET_HEAD      = 0xD1,  /* params: head_id */

    /* SlugArch vendor J-extension commands */
    CXL_GPU_CMD_J_QUERY             = 0xE0,
    CXL_GPU_CMD_J_LOAD_POLICY       = 0xE1,
    CXL_GPU_CMD_J_RESET             = 0xE2,
    CXL_GPU_CMD_J_GET_STATS         = 0xE3,
    CXL_GPU_CMD_J_GET_DIAGNOSTIC    = 0xE4,
} CXLGPUCommand;

/* P2P register offsets and peer types: defined in cxl_p2p_dma.h */

/* Coherent pool register offsets (in GPU command region) */
#define CXL_GPU_REG_COH_POOL_BASE   0x0300  /* Coherent pool base offset */
#define CXL_GPU_REG_COH_POOL_SIZE   0x0308  /* Coherent pool total size */
#define CXL_GPU_REG_COH_POOL_FREE   0x0310  /* Coherent pool free space */
#define CXL_GPU_REG_COH_DIR_SIZE    0x0318  /* Directory size (entries) */
#define CXL_GPU_REG_COH_DIR_USED    0x0320  /* Directory used entries */

/* DCD/GFAM/MH-SLD status registers */
#define CXL_GPU_REG_DCD_TOTAL       0x0330  /* DCD total capacity */
#define CXL_GPU_REG_DCD_ALLOCATED   0x0338  /* DCD allocated capacity */
#define CXL_GPU_REG_DCD_FREE        0x0340  /* DCD free capacity */
#define CXL_GPU_REG_DCD_EXTENTS     0x0348  /* Active DCD extent count */
#define CXL_GPU_REG_GFAM_HOSTS      0x0350  /* Configured GFAM hosts */
#define CXL_GPU_REG_GFAM_MAPPINGS   0x0358  /* Active GFAM mappings */
#define CXL_GPU_REG_GFAM_DENIED     0x0360  /* Denied GFAM accesses */
#define CXL_GPU_REG_MHSLD_HEADS     0x0370  /* MH-SLD head count */
#define CXL_GPU_REG_MHSLD_HEAD_ID   0x0378  /* Local MH-SLD head id */
#define CXL_GPU_REG_MHSLD_CONFLICTS 0x0380  /* MH-SLD coherency conflicts */
#define CXL_GPU_REG_MHSLD_INV       0x0388  /* MH-SLD invalidations */

/* Bias mode constants */
#define CXL_BIAS_HOST               0       /* Host-biased: CPU is coherence home */
#define CXL_BIAS_DEVICE             1       /* Device-biased: GPU snoop filter is home */

/* DCD/GFAM permission bits */
#define CXL_DCD_PERM_READ           (1 << 0)
#define CXL_DCD_PERM_WRITE          (1 << 1)
#define CXL_DCD_PERM_ATOMIC         (1 << 2)
#define CXL_DCD_PERM_SHARED         (1 << 3)
#define CXL_DCD_PERM_ALL            (CXL_DCD_PERM_READ | \
                                     CXL_DCD_PERM_WRITE | \
                                     CXL_DCD_PERM_ATOMIC | \
                                     CXL_DCD_PERM_SHARED)

/* Error codes (matching CUDA error codes) */
typedef enum {
    CXL_GPU_SUCCESS                     = 0,
    CXL_GPU_ERROR_INVALID_VALUE         = 1,
    CXL_GPU_ERROR_OUT_OF_MEMORY         = 2,
    CXL_GPU_ERROR_NOT_INITIALIZED       = 3,
    CXL_GPU_ERROR_DEINITIALIZED         = 4,
    CXL_GPU_ERROR_NO_DEVICE             = 100,
    CXL_GPU_ERROR_INVALID_DEVICE        = 101,
    CXL_GPU_ERROR_INVALID_CONTEXT       = 201,
    CXL_GPU_ERROR_INVALID_HANDLE        = 400,
    CXL_GPU_ERROR_NOT_FOUND             = 500,
    CXL_GPU_ERROR_NOT_READY             = 600,
    CXL_GPU_ERROR_LAUNCH_FAILED         = 700,
    CXL_GPU_ERROR_INVALID_PTX           = 800,
    CXL_GPU_ERROR_UNKNOWN               = 999,
    CXL_GPU_ERROR_COHERENCY             = 1000,
} CXLGPUError;

typedef enum {
    CXL_COH_RANGE_READ = 0,
    CXL_COH_RANGE_WRITE = 1,
} CXLCohRangeIntent;

/* Memory allocation info */
typedef struct {
    uint64_t device_ptr;
    uint64_t size;
    uint32_t flags;
    uint32_t reserved;
} CXLGPUMemInfo;

/* Kernel launch configuration */
typedef struct {
    uint32_t grid_dim_x;
    uint32_t grid_dim_y;
    uint32_t grid_dim_z;
    uint32_t block_dim_x;
    uint32_t block_dim_y;
    uint32_t block_dim_z;
    uint32_t shared_mem_bytes;
    uint32_t stream;
    uint64_t function_handle;
    uint64_t args_ptr;      /* Pointer to kernel arguments in data region */
    uint32_t num_args;
    uint32_t reserved;
} CXLGPUKernelLaunch;

/* Bulk transfer descriptor (for large memory operations) */
typedef struct {
    uint64_t host_addr;      /* Host virtual address (for BAR4 offset) */
    uint64_t device_ptr;     /* GPU device pointer */
    uint64_t size;           /* Transfer size in bytes */
    uint32_t flags;          /* Transfer flags */
    uint32_t stream;         /* Stream for async transfers */
} CXLGPUBulkTransfer;

/* Bulk transfer flags */
#define CXL_GPU_BULK_FLAG_ASYNC     (1 << 0)  /* Asynchronous transfer */
#define CXL_GPU_BULK_FLAG_COHERENT  (1 << 1)  /* Use CXL.cache coherency */
#define CXL_GPU_BULK_FLAG_NOCACHE   (1 << 2)  /* Bypass cache (write-combining) */

/* Cache operation descriptor */
typedef struct {
    uint64_t addr;           /* Start address */
    uint64_t size;           /* Size of region */
    uint32_t operation;      /* Flush/Invalidate/Writeback */
    uint32_t flags;          /* Operation flags */
} CXLGPUCacheOp;

#endif /* CXL_TYPE2_GPU_CMD_H */
