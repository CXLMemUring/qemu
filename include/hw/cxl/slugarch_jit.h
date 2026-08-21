/*
 * SlugArch J-extension dynamic runtime adapter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CXL_SLUGARCH_JIT_H
#define HW_CXL_SLUGARCH_JIT_H

#include "qemu/typedefs.h"

#define SLUG_JIT_ABI_VERSION 1U
#define SLUG_JIT_PAYLOAD_BYTES 64U
#define SLUG_JIT_DIGEST_BYTES 32U
#define SLUG_JIT_MAX_DIAGNOSTIC_BYTES (64U * 1024U)
#define SLUG_JIT_MAX_POLICY_BYTES (64U * 1024U)

enum {
    SLUG_JIT_BACKEND_NONE = 0,
    SLUG_JIT_BACKEND_RUST = 1,
    SLUG_JIT_BACKEND_GPU = 2,
    SLUG_JIT_BACKEND_FPGA_VERILATOR = 3,
};

enum {
    SLUG_JIT_DIRECTION_HOST_TO_DEVICE = 0,
    SLUG_JIT_DIRECTION_DEVICE_TO_HOST = 1,
};

enum {
    SLUG_JIT_EVENT_CXL_MEM_READ = 1,
    SLUG_JIT_EVENT_CXL_MEM_WRITE = 2,
    SLUG_JIT_EVENT_CXL_MEM_DATA = 3,
    SLUG_JIT_EVENT_COMPLETION = 4,
    SLUG_JIT_EVENT_PTX_MODULE_LOAD = 5,
    SLUG_JIT_EVENT_KERNEL_LAUNCH = 6,
    SLUG_JIT_EVENT_PHASE = 7,
    SLUG_JIT_EVENT_FENCE = 8,
};

enum {
    SLUG_JIT_CAP_POLICY = UINT64_C(1) << 0,
    SLUG_JIT_CAP_RECORD = UINT64_C(1) << 1,
    SLUG_JIT_CAP_GPU_DIAGNOSTIC = UINT64_C(1) << 2,
    SLUG_JIT_CAP_FPGA_RTL = UINT64_C(1) << 3,
};

enum {
    SLUG_JIT_OK = 0,
    SLUG_JIT_ERR_NULL = 1,
    SLUG_JIT_ERR_STRUCT_SIZE = 2,
    SLUG_JIT_ERR_ABI_VERSION = 3,
    SLUG_JIT_ERR_PARSE = 4,
    SLUG_JIT_ERR_POLICY_VERSION = 5,
    SLUG_JIT_ERR_TOO_MANY_INSTRUCTIONS = 6,
    SLUG_JIT_ERR_TOO_MANY_RANGES = 7,
    SLUG_JIT_ERR_INVALID_RANGE = 8,
    SLUG_JIT_ERR_INVALID_STRIDE = 9,
    SLUG_JIT_ERR_BUDGET_EXCEEDED = 10,
    SLUG_JIT_ERR_INVALID_CONTROL_FLOW = 11,
    SLUG_JIT_ERR_UNSUPPORTED = 12,
    SLUG_JIT_ERR_DIGEST_MISMATCH = 13,
    SLUG_JIT_ERR_REJECTED = 14,
    SLUG_JIT_ERR_DROP = 15,
    SLUG_JIT_ERR_TIMEOUT = 16,
    SLUG_JIT_ERR_BACKEND = 17,
    SLUG_JIT_ERR_IO = 18,
    SLUG_JIT_ERR_POISONED = 19,
    SLUG_JIT_ERR_PANIC = 20,
};

enum {
    SLUG_JIT_STATUS_OFF = 0,
    SLUG_JIT_STATUS_LOADING = 1,
    SLUG_JIT_STATUS_READY = 2,
    SLUG_JIT_STATUS_ERROR = 3,
};

typedef struct SlugJitHandle SlugJitHandle;

typedef struct SlugJitCreateArgs {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t backend;
    uint32_t strict;
    uint32_t diagnostic_capacity;
    uint32_t reserved;
} SlugJitCreateArgs;

typedef struct SlugJitEvent {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t event_id;
    uint64_t client_id;
    uint32_t direction;
    uint32_t event_class;
    uint32_t opcode;
    uint32_t payload_len;
    uint64_t address;
    uint64_t tag;
    uint64_t phase_id;
    uint64_t monotonic_ns;
    uint32_t status;
    uint32_t reserved;
    uint8_t payload[SLUG_JIT_PAYLOAD_BYTES];
} SlugJitEvent;

typedef struct SlugJitPolicyInfo {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t backend;
    uint32_t canonical_bytes;
    uint8_t digest[SLUG_JIT_DIGEST_BYTES];
    uint32_t instruction_count;
    uint32_t range_count;
    uint32_t metadata_budget;
    uint32_t reserved;
} SlugJitPolicyInfo;

typedef struct SlugJitDecision {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t accepted;
    uint32_t emitted;
    uint32_t error_code;
    uint32_t record_bytes;
    uint32_t payload_bytes;
    uint32_t reserved;
    uint64_t epoch;
    uint64_t record_id;
} SlugJitDecision;

typedef struct SlugJitStats {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t event_count;
    uint64_t record_count;
    uint64_t metadata_bytes;
    uint64_t reject_count;
    uint64_t drop_count;
    uint64_t epoch;
} SlugJitStats;

typedef struct CXLType2JitEvent {
    uint64_t client_id;
    uint32_t direction;
    uint32_t event_class;
    uint32_t opcode;
    uint32_t payload_len;
    uint64_t address;
    uint64_t tag;
    uint64_t phase_id;
    uint64_t monotonic_ns;
    uint32_t status;
    uint8_t payload[SLUG_JIT_PAYLOAD_BYTES];
} CXLType2JitEvent;

typedef uint32_t (*SlugJitAbiVersionFn)(void);
typedef uint64_t (*SlugJitBackendCapsFn)(void);
typedef int32_t (*SlugJitCreateFn)(const SlugJitCreateArgs *,
                                  SlugJitHandle **);
typedef int32_t (*SlugJitLoadPolicyFn)(SlugJitHandle *, const uint8_t *,
                                      uint32_t, SlugJitPolicyInfo *);
typedef int32_t (*SlugJitObserveFn)(SlugJitHandle *, const SlugJitEvent *,
                                   SlugJitDecision *);
typedef int32_t (*SlugJitStatsFn)(SlugJitHandle *, SlugJitStats *);
typedef int32_t (*SlugJitDiagnosticFn)(SlugJitHandle *, uint8_t *, uint32_t,
                                      uint32_t *);
typedef void (*SlugJitDestroyFn)(SlugJitHandle *);

/*
 * Provider exports.  QEMU resolves these names dynamically; these declarations
 * also let ABI-compatible test providers receive normal prototype checking.
 */
uint32_t slugarch_jit_abi_version(void);
uint64_t slugarch_jit_backend_caps(void);
int32_t slugarch_jit_create(const SlugJitCreateArgs *args,
                            SlugJitHandle **out);
int32_t slugarch_jit_load_policy(SlugJitHandle *handle,
                                 const uint8_t *policy, uint32_t policy_len,
                                 SlugJitPolicyInfo *out);
int32_t slugarch_jit_observe(SlugJitHandle *handle,
                             const SlugJitEvent *event,
                             SlugJitDecision *out);
int32_t slugarch_jit_stats(SlugJitHandle *handle, SlugJitStats *out);
int32_t slugarch_jit_last_diagnostic(SlugJitHandle *handle, uint8_t *out,
                                     uint32_t capacity, uint32_t *written);
void slugarch_jit_destroy(SlugJitHandle *handle);

typedef struct CXLType2JitState {
    void *library;
    SlugJitHandle *handle;
    char *library_path;
    char *policy_path;
    char *log_path;
    FILE *log_file;

    uint32_t abi_version;
    uint32_t requested_backend;
    uint32_t selected_backend;
    uint32_t status;
    uint32_t last_error;
    uint32_t diagnostic_capacity;
    uint64_t capabilities;
    uint64_t next_event_id;
    bool strict;
    bool ready;

    uint8_t policy_digest[SLUG_JIT_DIGEST_BYTES];
    uint32_t policy_bytes;
    SlugJitStats stats;
    SlugJitEvent last_event;
    SlugJitDecision last_decision;

    SlugJitAbiVersionFn abi_version_fn;
    SlugJitBackendCapsFn backend_caps_fn;
    SlugJitCreateFn create_fn;
    SlugJitLoadPolicyFn load_policy_fn;
    SlugJitObserveFn observe_fn;
    SlugJitStatsFn stats_fn;
    SlugJitDiagnosticFn diagnostic_fn;
    SlugJitDestroyFn destroy_fn;
} CXLType2JitState;

void cxl_type2_jit_state_init(CXLType2JitState *state,
                              uint32_t requested_backend, bool strict,
                              uint32_t diagnostic_capacity);
bool cxl_type2_jit_open(CXLType2JitState *state, const char *library_path,
                        Error **errp);
bool cxl_type2_jit_load_policy(CXLType2JitState *state,
                               const uint8_t *policy, size_t policy_len,
                               Error **errp);
bool cxl_type2_jit_load_policy_file(CXLType2JitState *state,
                                    const char *policy_path, Error **errp);
bool cxl_type2_jit_open_log(CXLType2JitState *state, const char *log_path,
                            Error **errp);
bool cxl_type2_jit_observe_event(CXLType2JitState *state,
                                 const CXLType2JitEvent *event,
                                 SlugJitDecision *decision, Error **errp);
bool cxl_type2_jit_refresh_stats(CXLType2JitState *state, Error **errp);
bool cxl_type2_jit_copy_diagnostic(CXLType2JitState *state, uint8_t *out,
                                   uint32_t capacity, uint32_t *written,
                                   Error **errp);
bool cxl_type2_jit_log_cfmws_join(CXLType2JitState *state,
                                  uint64_t request_event_id,
                                  uint64_t completion_event_id,
                                  uint64_t request_id,
                                  uint64_t server_sequence,
                                  bool external_commit,
                                  uint32_t effective_error,
                                  Error **errp);
void cxl_type2_jit_close(CXLType2JitState *state);

QEMU_BUILD_BUG_ON(sizeof(SlugJitCreateArgs) != 24);
QEMU_BUILD_BUG_ON(sizeof(SlugJitEvent) != 144);
QEMU_BUILD_BUG_ON(sizeof(SlugJitPolicyInfo) != 64);
QEMU_BUILD_BUG_ON(sizeof(SlugJitDecision) != 48);
QEMU_BUILD_BUG_ON(sizeof(SlugJitStats) != 56);

#endif
