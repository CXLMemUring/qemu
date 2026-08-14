/*
 * CXL Type-3 protocol-v2 CXLMemSim adapter
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "hw/cxl/cxl_type3_memsim_v2.h"
#include "qemu/error-report.h"
#include "qemu/units.h"

#define CXL_TYPE3_MEMSIM_V2_DEFAULT_PORT 9300
#define CXL_TYPE3_MEMSIM_V2_DEFAULT_CACHE_CAPACITY (256 * KiB)
#define CXL_TYPE3_MEMSIM_V2_DEFAULT_CACHE_WAYS 4
#define CXL_TYPE3_MEMSIM_V2_DEFAULT_TIMEOUT_MS 5000

CxlType3MemsimV2Config cxl_type3_memsim_v2_default_config(void)
{
    return (CxlType3MemsimV2Config) {
        .server_host = "127.0.0.1",
        .server_port = CXL_TYPE3_MEMSIM_V2_DEFAULT_PORT,
        .cache_capacity = CXL_TYPE3_MEMSIM_V2_DEFAULT_CACHE_CAPACITY,
        .cache_ways = CXL_TYPE3_MEMSIM_V2_DEFAULT_CACHE_WAYS,
        .timeout_ms = CXL_TYPE3_MEMSIM_V2_DEFAULT_TIMEOUT_MS,
    };
}

bool cxl_type3_memsim_v2_validate(const CxlType3MemsimV2Config *config,
                                  Error **errp)
{
    uint32_t cache_lines;

    if (!config) {
        error_setg(errp, "missing CXL Type-3 CXLMemSim v2 configuration");
        return false;
    }
    if (!config->enabled) {
        return true;
    }
    if (!config->server_host || !config->server_host[0] ||
        !config->server_port) {
        error_setg(errp, "invalid CXL Type-3 CXLMemSim v2 TCP address");
        return false;
    }
    if (config->host_id >= CXL_MEMSIM_V2_MAX_ENDPOINTS) {
        error_setg(errp, "CXL Type-3 coherence-v2-host-id must be below %u",
                   CXL_MEMSIM_V2_MAX_ENDPOINTS);
        return false;
    }
    if (config->cache_capacity < CXL_MEMSIM_V2_LINE_SIZE ||
        config->cache_capacity % CXL_MEMSIM_V2_LINE_SIZE) {
        error_setg(errp, "CXL Type-3 coherence-v2 cache capacity must be a "
                   "nonzero multiple of %u bytes",
                   CXL_MEMSIM_V2_LINE_SIZE);
        return false;
    }
    cache_lines = config->cache_capacity / CXL_MEMSIM_V2_LINE_SIZE;
    if (!config->cache_ways || cache_lines % config->cache_ways) {
        error_setg(errp, "CXL Type-3 coherence-v2 cache ways must divide "
                   "the cache line count");
        return false;
    }
    if (!config->timeout_ms || config->timeout_ms > INT_MAX) {
        error_setg(errp, "CXL Type-3 coherence-v2 timeout is invalid");
        return false;
    }
    if (config->write_through) {
        error_setg(errp, "CXL Type-3 Legofs coherence proof requires "
                   "write-back mode");
        return false;
    }
    return true;
}

bool cxl_type3_memsim_v2_realize(CxlType3MemsimV2 *state, Error **errp)
{
    Error *local_err = NULL;

    if (!state || !cxl_type3_memsim_v2_validate(state ? &state->config : NULL,
                                                 errp)) {
        return false;
    }
    state->enabled = state->config.enabled;
    if (!state->enabled) {
        return true;
    }
    state->client = cxl_memsim_v2_client_new(state->config.host_id, NULL,
                                             NULL);
    if (!state->client) {
        error_setg(errp, "cannot allocate CXL Type-3 CXLMemSim v2 client");
        return false;
    }
    if (!cxl_memsim_v2_client_set_write_policy(
            state->client, CXL_MEMSIM_V2_WRITE_BACK, &local_err) ||
        !cxl_memsim_v2_client_connect(
            state->client, state->config.server_host, state->config.server_port,
            state->config.cache_capacity, state->config.cache_ways,
            state->config.timeout_ms, &local_err)) {
        cxl_memsim_v2_client_free(state->client);
        state->client = NULL;
        error_propagate(errp, local_err);
        return false;
    }
    info_report("CXL Type3: MESI v2 host=%u session=%" PRIu64 " server=%s:%u",
                state->config.host_id,
                cxl_memsim_v2_client_session(state->client),
                state->config.server_host, state->config.server_port);
    return true;
}

void cxl_type3_memsim_v2_unrealize(CxlType3MemsimV2 *state)
{
    if (!state) {
        return;
    }
    cxl_memsim_v2_client_free(state->client);
    state->client = NULL;
    state->enabled = false;
}

MemTxResult cxl_type3_memsim_v2_read(CxlType3MemsimV2 *state,
                                     uint64_t dpa, uint64_t *value,
                                     unsigned size)
{
    Error *local_err = NULL;

    if (!state || !state->enabled || !state->client || !value ||
        !cxl_memsim_v2_load(state->client, dpa, size, value,
                            state->config.timeout_ms, &local_err)) {
        if (local_err) {
            error_report_err(local_err);
        }
        return MEMTX_ERROR;
    }
    return MEMTX_OK;
}

MemTxResult cxl_type3_memsim_v2_write(CxlType3MemsimV2 *state,
                                      uint64_t dpa, uint64_t value,
                                      unsigned size)
{
    Error *local_err = NULL;

    if (!state || !state->enabled || !state->client ||
        !cxl_memsim_v2_store(state->client, dpa, size, value,
                             state->config.timeout_ms, &local_err)) {
        if (local_err) {
            error_report_err(local_err);
        }
        return MEMTX_ERROR;
    }
    return MEMTX_OK;
}
