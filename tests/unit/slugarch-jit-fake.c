/*
 * ABI-compatible fake SlugArch JIT shared library.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/cxl/slugarch_jit.h"

typedef enum FakePolicyMode {
    FAKE_POLICY_NONE,
    FAKE_POLICY_VALID,
    FAKE_POLICY_REJECT,
    FAKE_POLICY_DROP,
    FAKE_POLICY_PANIC,
    FAKE_POLICY_WRONG_BACKEND,
} FakePolicyMode;

struct SlugJitHandle {
    uint32_t backend;
    uint32_t diagnostic_capacity;
    FakePolicyMode mode;
    SlugJitStats stats;
    uint64_t last_event_id;
    char diagnostic[128];
};

static bool fake_prefix_valid(uint32_t size, uint32_t version,
                              size_t expected)
{
    return size >= expected && version == SLUG_JIT_ABI_VERSION;
}

static void fake_set_diagnostic(SlugJitHandle *handle, const char *message)
{
    g_strlcpy(handle->diagnostic, message, sizeof(handle->diagnostic));
}

static bool fake_tail_is_zero(const SlugJitEvent *event)
{
    for (size_t i = event->payload_len; i < SLUG_JIT_PAYLOAD_BYTES; i++) {
        if (event->payload[i] != 0) {
            return false;
        }
    }
    return true;
}

uint32_t slugarch_jit_abi_version(void)
{
    return g_getenv("SLUGARCH_JIT_FAKE_BAD_ABI") ?
        SLUG_JIT_ABI_VERSION + 1 : SLUG_JIT_ABI_VERSION;
}

uint64_t slugarch_jit_backend_caps(void)
{
    uint64_t capabilities =
        SLUG_JIT_CAP_POLICY | SLUG_JIT_CAP_RECORD |
        SLUG_JIT_CAP_GPU_DIAGNOSTIC | SLUG_JIT_CAP_FPGA_RTL;

    if (g_getenv("SLUGARCH_JIT_FAKE_NO_FPGA")) {
        capabilities &= ~SLUG_JIT_CAP_FPGA_RTL;
    }
    if (g_getenv("SLUGARCH_JIT_FAKE_MISSING_CORE_CAP")) {
        capabilities &= ~SLUG_JIT_CAP_RECORD;
    }
    return capabilities;
}

int32_t slugarch_jit_create(const SlugJitCreateArgs *args,
                            SlugJitHandle **out)
{
    SlugJitHandle *handle;

    if (!args || !out) {
        return SLUG_JIT_ERR_NULL;
    }
    *out = NULL;
    if (!fake_prefix_valid(args->struct_size, args->abi_version,
                           sizeof(*args))) {
        return SLUG_JIT_ERR_STRUCT_SIZE;
    }
    if (args->strict != 1 || args->reserved != 0 ||
        args->backend < SLUG_JIT_BACKEND_RUST ||
        args->backend > SLUG_JIT_BACKEND_FPGA_VERILATOR) {
        return SLUG_JIT_ERR_UNSUPPORTED;
    }
    if (args->diagnostic_capacity > SLUG_JIT_MAX_DIAGNOSTIC_BYTES) {
        return SLUG_JIT_ERR_BUDGET_EXCEEDED;
    }

    handle = g_new0(SlugJitHandle, 1);
    handle->backend = args->backend;
    handle->diagnostic_capacity = args->diagnostic_capacity;
    handle->stats.struct_size = sizeof(handle->stats);
    handle->stats.abi_version = SLUG_JIT_ABI_VERSION;
    *out = handle;
    return SLUG_JIT_OK;
}

int32_t slugarch_jit_load_policy(SlugJitHandle *handle,
                                 const uint8_t *policy, uint32_t policy_len,
                                 SlugJitPolicyInfo *out)
{
    g_autofree char *text = NULL;

    if (!handle || !policy || !out) {
        return SLUG_JIT_ERR_NULL;
    }
    if (!fake_prefix_valid(out->struct_size, out->abi_version, sizeof(*out))) {
        return SLUG_JIT_ERR_STRUCT_SIZE;
    }
    text = g_strndup((const char *)policy, policy_len);
    if (strcmp(text, "invalid") == 0) {
        fake_set_diagnostic(handle, "fake policy parse error");
        return SLUG_JIT_ERR_PARSE;
    }

    if (strcmp(text, "valid") == 0 || strcmp(text, "bad-digest") == 0) {
        handle->mode = FAKE_POLICY_VALID;
    } else if (strcmp(text, "reject") == 0) {
        handle->mode = FAKE_POLICY_REJECT;
    } else if (strcmp(text, "drop") == 0) {
        handle->mode = FAKE_POLICY_DROP;
    } else if (strcmp(text, "panic") == 0) {
        handle->mode = FAKE_POLICY_PANIC;
    } else if (strcmp(text, "wrong-backend") == 0) {
        handle->mode = FAKE_POLICY_WRONG_BACKEND;
    } else {
        fake_set_diagnostic(handle, "unknown fake policy");
        return SLUG_JIT_ERR_PARSE;
    }

    memset(&handle->stats, 0, sizeof(handle->stats));
    handle->stats.struct_size = sizeof(handle->stats);
    handle->stats.abi_version = SLUG_JIT_ABI_VERSION;
    handle->last_event_id = 0;
    handle->diagnostic[0] = '\0';

    memset(out, 0, sizeof(*out));
    out->struct_size = sizeof(*out);
    out->abi_version = SLUG_JIT_ABI_VERSION;
    out->backend = handle->mode == FAKE_POLICY_WRONG_BACKEND ?
        SLUG_JIT_BACKEND_GPU : handle->backend;
    out->canonical_bytes = policy_len;
    if (strcmp(text, "bad-digest") != 0) {
        memset(out->digest, 0xa5, sizeof(out->digest));
    }
    out->instruction_count = 4;
    out->range_count = 1;
    out->metadata_budget = 256;
    return SLUG_JIT_OK;
}

int32_t slugarch_jit_observe(SlugJitHandle *handle,
                             const SlugJitEvent *event,
                             SlugJitDecision *out)
{
    if (!handle || !event || !out) {
        return SLUG_JIT_ERR_NULL;
    }
    if (!fake_prefix_valid(event->struct_size, event->abi_version,
                           sizeof(*event)) ||
        !fake_prefix_valid(out->struct_size, out->abi_version, sizeof(*out))) {
        return SLUG_JIT_ERR_STRUCT_SIZE;
    }
    if (handle->mode == FAKE_POLICY_NONE) {
        return SLUG_JIT_ERR_BACKEND;
    }
    if (event->event_id != handle->last_event_id + 1 ||
        event->payload_len > SLUG_JIT_PAYLOAD_BYTES ||
        !fake_tail_is_zero(event)) {
        fake_set_diagnostic(handle, "fake event conversion mismatch");
        return SLUG_JIT_ERR_UNSUPPORTED;
    }
    handle->last_event_id = event->event_id;

    if (handle->mode == FAKE_POLICY_PANIC) {
        fake_set_diagnostic(handle, "fake panic");
        return SLUG_JIT_ERR_PANIC;
    }
    if (handle->mode == FAKE_POLICY_DROP) {
        handle->stats.drop_count++;
        fake_set_diagnostic(handle, "fake strict drop");
        return SLUG_JIT_ERR_DROP;
    }

    memset(out, 0, sizeof(*out));
    out->struct_size = sizeof(*out);
    out->abi_version = SLUG_JIT_ABI_VERSION;
    handle->stats.event_count++;
    handle->stats.epoch = event->phase_id;

    if (handle->mode == FAKE_POLICY_REJECT) {
        handle->stats.reject_count++;
        out->error_code = SLUG_JIT_ERR_REJECTED;
        out->epoch = event->phase_id;
        return SLUG_JIT_OK;
    }

    handle->stats.record_count++;
    handle->stats.metadata_bytes += 8;
    out->accepted = 1;
    out->emitted = 1;
    out->record_bytes = 104;
    out->payload_bytes = 8;
    out->epoch = event->phase_id;
    out->record_id = handle->stats.record_count;
    return SLUG_JIT_OK;
}

#ifndef SLUGARCH_JIT_FAKE_OMIT_STATS
int32_t slugarch_jit_stats(SlugJitHandle *handle, SlugJitStats *out)
{
    if (!handle || !out) {
        return SLUG_JIT_ERR_NULL;
    }
    if (!fake_prefix_valid(out->struct_size, out->abi_version, sizeof(*out))) {
        return SLUG_JIT_ERR_STRUCT_SIZE;
    }
    *out = handle->stats;
    return SLUG_JIT_OK;
}
#endif

int32_t slugarch_jit_last_diagnostic(SlugJitHandle *handle, uint8_t *out,
                                     uint32_t capacity, uint32_t *written)
{
    uint32_t length;

    if (!handle || !written || (!out && capacity != 0)) {
        return SLUG_JIT_ERR_NULL;
    }
    length = MIN(strlen(handle->diagnostic), handle->diagnostic_capacity);
    *written = length;
    if (!out) {
        return SLUG_JIT_OK;
    }
    if (capacity < length) {
        return SLUG_JIT_ERR_STRUCT_SIZE;
    }
    memcpy(out, handle->diagnostic, length);
    return SLUG_JIT_OK;
}

void slugarch_jit_destroy(SlugJitHandle *handle)
{
    g_free(handle);
}
