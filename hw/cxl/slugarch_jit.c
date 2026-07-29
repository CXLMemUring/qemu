/*
 * SlugArch J-extension dynamic runtime adapter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/cxl/slugarch_jit.h"

#include <dlfcn.h>
#include <sys/stat.h>

static bool jit_path_is_regular(const char *path, const char *purpose,
                                Error **errp)
{
    struct stat st;

    if (!path || !g_path_is_absolute(path)) {
        error_setg(errp, "SlugArch JIT %s path must be absolute", purpose);
        return false;
    }
    if (stat(path, &st) < 0) {
        error_setg_errno(errp, errno,
                         "SlugArch JIT %s prerequisite for dlopen/stat '%s'",
                         purpose, path);
        return false;
    }
    if (!S_ISREG(st.st_mode)) {
        error_setg(errp, "SlugArch JIT %s path '%s' is not a regular file",
                   purpose, path);
        return false;
    }
    return true;
}

static void jit_mark_error(CXLType2JitState *state, uint32_t code)
{
    state->last_error = code;
    state->status = SLUG_JIT_STATUS_ERROR;
    state->ready = false;
}

static bool jit_digest_nonzero(const uint8_t digest[SLUG_JIT_DIGEST_BYTES])
{
    for (size_t i = 0; i < SLUG_JIT_DIGEST_BYTES; i++) {
        if (digest[i] != 0) {
            return true;
        }
    }
    return false;
}

static bool jit_payload_tail_is_zero(const CXLType2JitEvent *event)
{
    for (size_t i = event->payload_len; i < SLUG_JIT_PAYLOAD_BYTES; i++) {
        if (event->payload[i] != 0) {
            return false;
        }
    }
    return true;
}

static uint64_t jit_required_capability(uint32_t backend)
{
    switch (backend) {
    case SLUG_JIT_BACKEND_RUST:
        return 0;
    case SLUG_JIT_BACKEND_GPU:
        return SLUG_JIT_CAP_GPU_DIAGNOSTIC;
    case SLUG_JIT_BACKEND_FPGA_VERILATOR:
        return SLUG_JIT_CAP_FPGA_RTL;
    default:
        return UINT64_MAX;
    }
}

static bool jit_resolve(void *library, void *function, size_t function_size,
                        const char *name, Error **errp)
{
    const char *message;
    void *symbol;

    dlerror();
    symbol = dlsym(library, name);
    message = dlerror();
    if (message || !symbol) {
        error_setg(errp, "SlugArch JIT missing symbol %s: %s", name,
                   message ? message : "not found");
        return false;
    }
    if (function_size != sizeof(symbol)) {
        error_setg(errp, "SlugArch JIT function-pointer size mismatch for %s",
                   name);
        return false;
    }
    memcpy(function, &symbol, function_size);
    return true;
}

static bool jit_resolve_all(CXLType2JitState *state, Error **errp)
{
#define JIT_RESOLVE(field, symbol)                                           \
    do {                                                                     \
        if (!jit_resolve(state->library, &state->field,                      \
                         sizeof(state->field), symbol, errp)) {              \
            return false;                                                    \
        }                                                                    \
    } while (0)

    JIT_RESOLVE(abi_version_fn, "slugarch_jit_abi_version");
    JIT_RESOLVE(backend_caps_fn, "slugarch_jit_backend_caps");
    JIT_RESOLVE(create_fn, "slugarch_jit_create");
    JIT_RESOLVE(load_policy_fn, "slugarch_jit_load_policy");
    JIT_RESOLVE(observe_fn, "slugarch_jit_observe");
    JIT_RESOLVE(stats_fn, "slugarch_jit_stats");
    JIT_RESOLVE(diagnostic_fn, "slugarch_jit_last_diagnostic");
    JIT_RESOLVE(destroy_fn, "slugarch_jit_destroy");
    return true;

#undef JIT_RESOLVE
}

static void jit_digest_hex(const uint8_t digest[SLUG_JIT_DIGEST_BYTES],
                           char output[SLUG_JIT_DIGEST_BYTES * 2 + 1])
{
    static const char hex[] = "0123456789abcdef";

    for (size_t i = 0; i < SLUG_JIT_DIGEST_BYTES; i++) {
        output[i * 2] = hex[digest[i] >> 4];
        output[i * 2 + 1] = hex[digest[i] & 0xf];
    }
    output[SLUG_JIT_DIGEST_BYTES * 2] = '\0';
}

static uint64_t jit_payload_fnv1a64(const SlugJitEvent *event)
{
    uint64_t hash = UINT64_C(14695981039346656037);

    for (size_t i = 0; i < event->payload_len; i++) {
        hash ^= event->payload[i];
        hash *= UINT64_C(1099511628211);
    }
    return hash;
}

static bool jit_log_event(CXLType2JitState *state, int32_t result,
                          uint32_t effective_error, Error **errp)
{
    char digest[SLUG_JIT_DIGEST_BYTES * 2 + 1];
    char payload_prefix[8 * 2 + 1];
    size_t prefix_length;
    int written;

    if (!state->log_file) {
        return true;
    }

    jit_digest_hex(state->policy_digest, digest);
    prefix_length = MIN((size_t)state->last_event.payload_len,
                        sizeof(payload_prefix) / 2);
    for (size_t i = 0; i < prefix_length; i++) {
        snprintf(payload_prefix + i * 2, 3, "%02x",
                 state->last_event.payload[i]);
    }
    payload_prefix[prefix_length * 2] = '\0';
    written = fprintf(state->log_file,
        "{\"schema\":\"slugarch.qemu-jit-event.v1\","
        "\"event_id\":%" PRIu64 ",\"client_id\":%" PRIu64 ","
        "\"direction\":%u,\"event_class\":%u,\"opcode\":%u,"
        "\"address\":%" PRIu64 ",\"tag\":%" PRIu64 ","
        "\"phase_id\":%" PRIu64 ",\"monotonic_ns\":%" PRIu64 ","
        "\"status\":%u,\"payload_len\":%u,"
        "\"payload_prefix_hex\":\"%s\","
        "\"payload_fnv1a64\":\"%016" PRIx64 "\","
        "\"policy_digest\":\"%s\",\"result\":%d,"
        "\"effective_error\":%u,\"accepted\":%u,\"emitted\":%u,"
        "\"decision_error\":%u,"
        "\"event_count\":%" PRIu64 ",\"record_count\":%" PRIu64 ","
        "\"metadata_bytes\":%" PRIu64 ",\"reject_count\":%" PRIu64 ","
        "\"drop_count\":%" PRIu64 ",\"epoch\":%" PRIu64 "}\n",
        state->last_event.event_id, state->last_event.client_id,
        state->last_event.direction, state->last_event.event_class,
        state->last_event.opcode, state->last_event.address,
        state->last_event.tag, state->last_event.phase_id,
        state->last_event.monotonic_ns, state->last_event.status,
        state->last_event.payload_len, payload_prefix,
        jit_payload_fnv1a64(&state->last_event), digest,
        result, effective_error,
        state->last_decision.accepted, state->last_decision.emitted,
        state->last_decision.error_code, state->stats.event_count,
        state->stats.record_count, state->stats.metadata_bytes,
        state->stats.reject_count, state->stats.drop_count,
        state->stats.epoch);
    if (written < 0 || fflush(state->log_file) != 0) {
        error_setg_errno(errp, errno,
                         "SlugArch JIT could not write JSONL evidence");
        jit_mark_error(state, SLUG_JIT_ERR_IO);
        return false;
    }
    return true;
}

bool cxl_type2_jit_log_cfmws_join(CXLType2JitState *state,
                                  uint64_t request_event_id,
                                  uint64_t completion_event_id,
                                  uint64_t request_id,
                                  uint64_t server_sequence,
                                  bool external_commit,
                                  uint32_t effective_error,
                                  Error **errp)
{
    char digest[SLUG_JIT_DIGEST_BYTES * 2 + 1];
    int written;

    if (!state || !state->log_file || !request_event_id || !request_id) {
        error_setg(errp, "SlugArch JIT CFMWS join state is invalid");
        if (state) {
            jit_mark_error(state, SLUG_JIT_ERR_IO);
        }
        return false;
    }
    if (external_commit && (!completion_event_id || !server_sequence)) {
        error_setg(errp,
                   "SlugArch JIT external commit lacks completion join");
        jit_mark_error(state, SLUG_JIT_ERR_IO);
        return false;
    }

    jit_digest_hex(state->policy_digest, digest);
    written = fprintf(state->log_file,
        "{\"schema\":\"slugarch.qemu-cfmws-join.v1\","
        "\"request_event_id\":%" PRIu64 ","
        "\"completion_event_id\":%" PRIu64 ","
        "\"request_id\":%" PRIu64 ","
        "\"server_sequence\":%" PRIu64 ","
        "\"external_commit\":%s,\"effective_error\":%u,"
        "\"policy_digest\":\"%s\"}\n",
        request_event_id, completion_event_id, request_id,
        server_sequence, external_commit ? "true" : "false",
        effective_error, digest);
    if (written < 0 || fflush(state->log_file) != 0) {
        error_setg_errno(errp, errno,
                         "SlugArch JIT could not write CFMWS join evidence");
        jit_mark_error(state, SLUG_JIT_ERR_IO);
        return false;
    }
    return true;
}

void cxl_type2_jit_state_init(CXLType2JitState *state,
                              uint32_t requested_backend, bool strict,
                              uint32_t diagnostic_capacity)
{
    g_assert(state);
    memset(state, 0, sizeof(*state));
    state->requested_backend = requested_backend;
    state->strict = strict;
    state->diagnostic_capacity = diagnostic_capacity;
    state->status = SLUG_JIT_STATUS_OFF;
}

void cxl_type2_jit_close(CXLType2JitState *state)
{
    uint32_t requested_backend;
    uint32_t diagnostic_capacity;
    bool strict;

    if (!state) {
        return;
    }

    requested_backend = state->requested_backend;
    diagnostic_capacity = state->diagnostic_capacity;
    strict = state->strict;

    if (state->handle && state->destroy_fn) {
        state->destroy_fn(state->handle);
    }
    state->handle = NULL;
    if (state->library) {
        dlclose(state->library);
    }
    state->library = NULL;
    if (state->log_file) {
        fclose(state->log_file);
    }
    state->log_file = NULL;
    g_free(state->library_path);
    g_free(state->policy_path);
    g_free(state->log_path);

    memset(state, 0, sizeof(*state));
    state->requested_backend = requested_backend;
    state->diagnostic_capacity = diagnostic_capacity;
    state->strict = strict;
    state->status = SLUG_JIT_STATUS_OFF;
}

bool cxl_type2_jit_open(CXLType2JitState *state, const char *library_path,
                        Error **errp)
{
    SlugJitCreateArgs args = {
        .struct_size = sizeof(args),
        .abi_version = SLUG_JIT_ABI_VERSION,
    };
    uint64_t required;
    int32_t result;
    const char *message;

    if (!state) {
        error_setg(errp, "SlugArch JIT state is null");
        return false;
    }
    if (state->library || state->handle) {
        error_setg(errp, "SlugArch JIT state is already open");
        return false;
    }
    if (!jit_path_is_regular(library_path, "library", errp)) {
        return false;
    }
    if (state->diagnostic_capacity > SLUG_JIT_MAX_DIAGNOSTIC_BYTES) {
        error_setg(errp, "SlugArch JIT diagnostic capacity exceeds 64 KiB");
        return false;
    }
    required = jit_required_capability(state->requested_backend);
    if (required == UINT64_MAX) {
        error_setg(errp, "SlugArch JIT requested backend %u is unsupported",
                   state->requested_backend);
        return false;
    }

    state->library = dlopen(library_path, RTLD_NOW | RTLD_LOCAL);
    if (!state->library) {
        message = dlerror();
        error_setg(errp, "SlugArch JIT dlopen('%s') failed: %s",
                   library_path, message ? message : "unknown error");
        cxl_type2_jit_close(state);
        return false;
    }
    if (!jit_resolve_all(state, errp)) {
        cxl_type2_jit_close(state);
        return false;
    }

    state->abi_version = state->abi_version_fn();
    if (state->abi_version != SLUG_JIT_ABI_VERSION) {
        error_setg(errp, "SlugArch JIT ABI version %u is not supported",
                   state->abi_version);
        cxl_type2_jit_close(state);
        return false;
    }
    state->capabilities = state->backend_caps_fn();
    if ((state->capabilities &
         (SLUG_JIT_CAP_POLICY | SLUG_JIT_CAP_RECORD)) !=
        (SLUG_JIT_CAP_POLICY | SLUG_JIT_CAP_RECORD) ||
        (required && !(state->capabilities & required))) {
        error_setg(errp,
                   "SlugArch JIT backend %u lacks required capabilities",
                   state->requested_backend);
        cxl_type2_jit_close(state);
        return false;
    }

    args.backend = state->requested_backend;
    args.strict = state->strict ? 1 : 0;
    args.diagnostic_capacity = state->diagnostic_capacity;
    result = state->create_fn(&args, &state->handle);
    if (result != SLUG_JIT_OK || !state->handle) {
        error_setg(errp, "SlugArch JIT create failed with code %d", result);
        cxl_type2_jit_close(state);
        return false;
    }

    state->library_path = g_strdup(library_path);
    state->status = SLUG_JIT_STATUS_LOADING;
    state->last_error = SLUG_JIT_OK;
    return true;
}

bool cxl_type2_jit_refresh_stats(CXLType2JitState *state, Error **errp)
{
    SlugJitStats stats = {
        .struct_size = sizeof(stats),
        .abi_version = SLUG_JIT_ABI_VERSION,
    };
    int32_t result;

    if (!state || !state->handle || !state->stats_fn) {
        error_setg(errp, "SlugArch JIT stats requested before open");
        return false;
    }
    result = state->stats_fn(state->handle, &stats);
    if (result != SLUG_JIT_OK) {
        error_setg(errp, "SlugArch JIT stats failed with code %d", result);
        jit_mark_error(state, result);
        return false;
    }
    if (stats.struct_size < sizeof(stats) ||
        stats.abi_version != SLUG_JIT_ABI_VERSION) {
        error_setg(errp, "SlugArch JIT stats returned an invalid ABI prefix");
        jit_mark_error(state, SLUG_JIT_ERR_ABI_VERSION);
        return false;
    }
    state->stats = stats;
    return true;
}

bool cxl_type2_jit_load_policy(CXLType2JitState *state,
                               const uint8_t *policy, size_t policy_len,
                               Error **errp)
{
    SlugJitPolicyInfo info = {
        .struct_size = sizeof(info),
        .abi_version = SLUG_JIT_ABI_VERSION,
    };
    int32_t result;

    if (!state || !state->handle || !state->load_policy_fn) {
        error_setg(errp, "SlugArch JIT policy load requested before open");
        return false;
    }
    if (!policy || policy_len == 0 ||
        policy_len > SLUG_JIT_MAX_POLICY_BYTES ||
        policy_len > UINT32_MAX) {
        error_setg(errp, "SlugArch JIT policy length is invalid");
        jit_mark_error(state, SLUG_JIT_ERR_STRUCT_SIZE);
        return false;
    }

    state->ready = false;
    state->status = SLUG_JIT_STATUS_LOADING;
    result = state->load_policy_fn(state->handle, policy,
                                   (uint32_t)policy_len, &info);
    if (result != SLUG_JIT_OK) {
        error_setg(errp, "SlugArch JIT policy load failed with code %d",
                   result);
        jit_mark_error(state, result);
        return false;
    }
    if (info.struct_size < sizeof(info) ||
        info.abi_version != SLUG_JIT_ABI_VERSION) {
        error_setg(errp, "SlugArch JIT policy info has an invalid ABI prefix");
        jit_mark_error(state, SLUG_JIT_ERR_ABI_VERSION);
        return false;
    }
    if (info.backend != state->requested_backend) {
        error_setg(errp,
                   "SlugArch JIT selected backend %u differs from requested %u",
                   info.backend, state->requested_backend);
        jit_mark_error(state, SLUG_JIT_ERR_UNSUPPORTED);
        return false;
    }
    if (!info.canonical_bytes ||
        info.canonical_bytes > SLUG_JIT_MAX_POLICY_BYTES ||
        !info.instruction_count || info.instruction_count > 32 ||
        info.range_count > 4 || info.metadata_budget > 256 ||
        info.reserved != 0) {
        error_setg(errp, "SlugArch JIT policy info is outside ABI limits");
        jit_mark_error(state, SLUG_JIT_ERR_INVALID_CONTROL_FLOW);
        return false;
    }
    if (!jit_digest_nonzero(info.digest)) {
        error_setg(errp, "SlugArch JIT policy digest is all zero");
        jit_mark_error(state, SLUG_JIT_ERR_DIGEST_MISMATCH);
        return false;
    }

    state->selected_backend = info.backend;
    state->policy_bytes = info.canonical_bytes;
    memcpy(state->policy_digest, info.digest, sizeof(state->policy_digest));
    state->next_event_id = 0;
    memset(&state->last_event, 0, sizeof(state->last_event));
    memset(&state->last_decision, 0, sizeof(state->last_decision));
    if (!cxl_type2_jit_refresh_stats(state, errp)) {
        return false;
    }
    state->last_error = SLUG_JIT_OK;
    state->status = SLUG_JIT_STATUS_READY;
    state->ready = true;
    return true;
}

bool cxl_type2_jit_load_policy_file(CXLType2JitState *state,
                                    const char *policy_path, Error **errp)
{
    g_autofree char *contents = NULL;
    g_autoptr(GError) local_error = NULL;
    gsize length = 0;

    if (!jit_path_is_regular(policy_path, "policy", errp)) {
        return false;
    }
    if (!g_file_get_contents(policy_path, &contents, &length, &local_error)) {
        error_setg(errp, "SlugArch JIT could not read policy '%s': %s",
                   policy_path, local_error->message);
        return false;
    }
    if (!cxl_type2_jit_load_policy(state, (const uint8_t *)contents,
                                   length, errp)) {
        return false;
    }
    g_free(state->policy_path);
    state->policy_path = g_strdup(policy_path);
    return true;
}

bool cxl_type2_jit_open_log(CXLType2JitState *state, const char *log_path,
                            Error **errp)
{
    struct stat st;
    FILE *file;

    if (!state || !log_path || !g_path_is_absolute(log_path)) {
        error_setg(errp, "SlugArch JIT log path must be absolute");
        return false;
    }
    if (stat(log_path, &st) == 0 && !S_ISREG(st.st_mode)) {
        error_setg(errp, "SlugArch JIT log path '%s' is not a regular file",
                   log_path);
        return false;
    }
    file = g_fopen(log_path, "a");
    if (!file) {
        error_setg_errno(errp, errno,
                         "SlugArch JIT could not open log '%s'", log_path);
        return false;
    }
    if (state->log_file) {
        fclose(state->log_file);
    }
    g_free(state->log_path);
    state->log_file = file;
    state->log_path = g_strdup(log_path);
    return true;
}

bool cxl_type2_jit_copy_diagnostic(CXLType2JitState *state, uint8_t *out,
                                   uint32_t capacity, uint32_t *written,
                                   Error **errp)
{
    int32_t result;

    if (!state || !state->handle || !state->diagnostic_fn) {
        error_setg(errp, "SlugArch JIT diagnostic requested before open");
        return false;
    }
    if (!written || (!out && capacity != 0)) {
        error_setg(errp, "SlugArch JIT diagnostic output is null");
        return false;
    }
    *written = 0;
    result = state->diagnostic_fn(state->handle, out, capacity, written);
    if (result != SLUG_JIT_OK) {
        error_setg(errp, "SlugArch JIT diagnostic failed with code %d",
                   result);
        state->last_error = result;
        return false;
    }
    return true;
}

bool cxl_type2_jit_observe_event(CXLType2JitState *state,
                                 const CXLType2JitEvent *event,
                                 SlugJitDecision *decision, Error **errp)
{
    SlugJitEvent raw = {
        .struct_size = sizeof(raw),
        .abi_version = SLUG_JIT_ABI_VERSION,
    };
    SlugJitDecision output = {
        .struct_size = sizeof(output),
        .abi_version = SLUG_JIT_ABI_VERSION,
    };
    Error *stats_error = NULL;
    int32_t result;
    uint32_t failure = SLUG_JIT_OK;

    if (!state || !state->ready || !state->handle || !state->observe_fn) {
        error_setg(errp, "SlugArch JIT is not ready");
        return false;
    }
    if (!event || !decision) {
        error_setg(errp, "SlugArch JIT event or decision is null");
        return false;
    }
    if (event->direction > 1 || event->event_class < 1 ||
        event->event_class > 8 || event->opcode > UINT16_MAX ||
        event->payload_len > SLUG_JIT_PAYLOAD_BYTES ||
        !jit_payload_tail_is_zero(event)) {
        error_setg(errp, "SlugArch JIT event payload or field is unsupported");
        if (state->strict) {
            jit_mark_error(state, SLUG_JIT_ERR_UNSUPPORTED);
        } else {
            state->last_error = SLUG_JIT_ERR_UNSUPPORTED;
        }
        return false;
    }
    if (state->next_event_id == UINT64_MAX) {
        error_setg(errp, "SlugArch JIT event ID overflow");
        jit_mark_error(state, SLUG_JIT_ERR_BACKEND);
        return false;
    }

    raw.event_id = state->next_event_id + 1;
    raw.client_id = event->client_id;
    raw.direction = event->direction;
    raw.event_class = event->event_class;
    raw.opcode = event->opcode;
    raw.payload_len = event->payload_len;
    raw.address = event->address;
    raw.tag = event->tag;
    raw.phase_id = event->phase_id;
    raw.monotonic_ns = event->monotonic_ns;
    raw.status = event->status;
    memcpy(raw.payload, event->payload, event->payload_len);
    state->next_event_id = raw.event_id;
    state->last_event = raw;

    result = state->observe_fn(state->handle, &raw, &output);
    state->last_decision = output;
    if (result != SLUG_JIT_OK) {
        failure = result;
    } else if (output.struct_size < sizeof(output) ||
               output.abi_version != SLUG_JIT_ABI_VERSION ||
               output.reserved != 0 || output.accepted > 1 ||
               output.emitted > 1 || output.emitted > output.accepted) {
        failure = SLUG_JIT_ERR_ABI_VERSION;
    } else if (!output.accepted || output.error_code != SLUG_JIT_OK) {
        failure = output.error_code ?
            output.error_code : SLUG_JIT_ERR_REJECTED;
    }

    if (!cxl_type2_jit_refresh_stats(state, &stats_error)) {
        failure = state->last_error;
    }
    if (!jit_log_event(state, result, failure, errp)) {
        error_free(stats_error);
        return false;
    }
    if (failure != SLUG_JIT_OK) {
        if (state->strict) {
            jit_mark_error(state, failure);
        } else {
            state->last_error = failure;
        }
        if (stats_error) {
            error_propagate(errp, stats_error);
        } else {
            error_setg(errp, "SlugArch JIT event failed with code %u",
                       failure);
        }
        *decision = output;
        return false;
    }
    if (stats_error) {
        error_propagate(errp, stats_error);
        return false;
    }

    state->last_error = SLUG_JIT_OK;
    *decision = output;
    return true;
}
