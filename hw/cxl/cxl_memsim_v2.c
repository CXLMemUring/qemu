/*
 * CXLMemSim protocol-v2 duplex endpoint client
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"

#include "hw/cxl/cxl_memsim_v2.h"
#include "io/channel-socket.h"
#include "qapi/qapi-types-sockets.h"
#include "qemu/bswap.h"
#include "qemu/thread.h"

#define CXL_MEMSIM_V2_RESPONSE_ACK_INTERVAL 128

typedef struct CxlMemsimV2Pending {
    uint64_t request_id;
    CxlMemsimV2Frame request;
    CxlMemsimV2Frame response;
    QemuCond completed;
    bool done;
    bool io_failed;
    bool retried;
} CxlMemsimV2Pending;

typedef struct CxlMemsimV2CacheLine {
    uint64_t address;
    uint64_t epoch;
    uint64_t last_used;
    uint8_t data[CXL_MEMSIM_V2_LINE_SIZE];
    uint8_t state;
    bool valid;
    bool dirty;
} CxlMemsimV2CacheLine;

struct CxlMemsimV2Client {
    uint16_t endpoint;
    uint64_t session_id;
    uint64_t next_request_id;
    uint64_t consumed_response_id;
    uint64_t acknowledged_response_id;
    QIOChannelSocket *socket;
    QemuThread progress_thread;
    QemuMutex state_lock;
    QemuMutex send_lock;
    QemuMutex operation_lock;
    QemuMutex cache_lock;
    GHashTable *pending;
    GHashTable *completed_retries;
    CxlMemsimV2CacheLine *cache;
    CxlMemsimV2SnoopHandler snoop_handler;
    void *snoop_opaque;
    char *connection_error;
    size_t cache_line_count;
    size_t cache_set_count;
    uint16_t cache_ways;
    int timeout_ms;
    uint64_t cache_clock;
    CxlMemsimV2WritePolicy write_policy;
    unsigned progress_starts;
    bool started;
    bool running;
    bool connected;
    bool response_ack_in_progress;
    bool progress_joinable;
};

QEMU_BUILD_BUG_ON(sizeof(CxlMemsimV2Frame) != CXL_MEMSIM_V2_FRAME_SIZE);
QEMU_BUILD_BUG_ON(offsetof(CxlMemsimV2Frame, request_id) != 24);
QEMU_BUILD_BUG_ON(offsetof(CxlMemsimV2Frame, data) != 104);

static bool cxl_memsim_v2_is_snoop(uint16_t opcode) {
    return opcode == CXL_MEMSIM_V2_OP_SNP_INV || opcode == CXL_MEMSIM_V2_OP_SNP_DOWNGRADE ||
           opcode == CXL_MEMSIM_V2_OP_SNP_DATA_INV || opcode == CXL_MEMSIM_V2_OP_SNP_DATA_DOWNGRADE ||
           opcode == CXL_MEMSIM_V2_OP_HOST_FENCE;
}

static bool cxl_memsim_v2_known_opcode(uint16_t opcode) {
    switch (opcode) {
    case CXL_MEMSIM_V2_OP_REGISTER:
    case CXL_MEMSIM_V2_OP_UNREGISTER:
    case CXL_MEMSIM_V2_OP_GETS:
    case CXL_MEMSIM_V2_OP_GETM:
    case CXL_MEMSIM_V2_OP_UPGRADE:
    case CXL_MEMSIM_V2_OP_PUTS:
    case CXL_MEMSIM_V2_OP_PUTM:
    case CXL_MEMSIM_V2_OP_ATOMIC_FAA:
    case CXL_MEMSIM_V2_OP_ATOMIC_CAS:
    case CXL_MEMSIM_V2_OP_FENCE:
    case CXL_MEMSIM_V2_OP_SNOOP_ACK:
    case CXL_MEMSIM_V2_OP_HEARTBEAT:
    case CXL_MEMSIM_V2_OP_RESPONSE:
    case CXL_MEMSIM_V2_OP_SNP_INV:
    case CXL_MEMSIM_V2_OP_SNP_DOWNGRADE:
    case CXL_MEMSIM_V2_OP_SNP_DATA_INV:
    case CXL_MEMSIM_V2_OP_SNP_DATA_DOWNGRADE:
    case CXL_MEMSIM_V2_OP_HOST_FENCE:
        return true;
    default:
        return false;
    }
}

static size_t cxl_memsim_v2_cache_set(const CxlMemsimV2Client *client, uint64_t line_address) {
    return (line_address / CXL_MEMSIM_V2_LINE_SIZE) % client->cache_set_count;
}

static CxlMemsimV2CacheLine *cxl_memsim_v2_cache_find_locked(CxlMemsimV2Client *client, uint64_t line_address) {
    size_t first;
    size_t way;

    if (!client->cache || !client->cache_set_count) {
        return NULL;
    }
    first = cxl_memsim_v2_cache_set(client, line_address) * client->cache_ways;
    for (way = 0; way < client->cache_ways; way++) {
        CxlMemsimV2CacheLine *line = &client->cache[first + way];

        if (line->valid && line->address == line_address) {
            return line;
        }
    }
    return NULL;
}

static CxlMemsimV2CacheLine *cxl_memsim_v2_cache_victim_locked(CxlMemsimV2Client *client, uint64_t line_address) {
    size_t first = cxl_memsim_v2_cache_set(client, line_address) * client->cache_ways;
    CxlMemsimV2CacheLine *victim = &client->cache[first];
    size_t way;

    for (way = 0; way < client->cache_ways; way++) {
        CxlMemsimV2CacheLine *candidate = &client->cache[first + way];

        if (!candidate->valid) {
            return candidate;
        }
        if (candidate->last_used < victim->last_used) {
            victim = candidate;
        }
    }
    return victim;
}

static void cxl_memsim_v2_cache_touch_locked(CxlMemsimV2Client *client, CxlMemsimV2CacheLine *line) {
    client->cache_clock++;
    if (!client->cache_clock) {
        size_t index;

        for (index = 0; index < client->cache_line_count; index++) {
            client->cache[index].last_used = 0;
        }
        client->cache_clock = 1;
    }
    line->last_used = client->cache_clock;
}

static bool cxl_memsim_v2_cache_snoop(CxlMemsimV2Client *client, const CxlMemsimV2Frame *snoop, CxlMemsimV2Frame *ack) {
    CxlMemsimV2CacheLine *line;
    size_t index;

    qemu_mutex_lock(&client->cache_lock);
    if (snoop->type == CXL_MEMSIM_V2_OP_HOST_FENCE) {
        for (index = 0; index < client->cache_line_count; index++) {
            if (client->cache[index].valid && client->cache[index].state == CXL_MEMSIM_V2_STATE_M) {
                qemu_mutex_unlock(&client->cache_lock);
                return true;
            }
        }
        memset(client->cache, 0, client->cache_line_count * sizeof(*client->cache));
        ack->status = CXL_MEMSIM_V2_STATUS_OK;
        ack->state = CXL_MEMSIM_V2_STATE_I;
        qemu_mutex_unlock(&client->cache_lock);
        return true;
    }

    line = cxl_memsim_v2_cache_find_locked(client, snoop->addr);
    if (!line || snoop->epoch <= line->epoch) {
        qemu_mutex_unlock(&client->cache_lock);
        return true;
    }
    switch (snoop->type) {
    case CXL_MEMSIM_V2_OP_SNP_INV:
        if (line->state != CXL_MEMSIM_V2_STATE_S && line->state != CXL_MEMSIM_V2_STATE_E) {
            break;
        }
        line->valid = false;
        line->dirty = false;
        line->state = CXL_MEMSIM_V2_STATE_I;
        line->epoch = snoop->epoch;
        ack->status = CXL_MEMSIM_V2_STATUS_OK;
        ack->state = CXL_MEMSIM_V2_STATE_I;
        break;
    case CXL_MEMSIM_V2_OP_SNP_DOWNGRADE:
        if (line->state != CXL_MEMSIM_V2_STATE_E) {
            break;
        }
        line->state = CXL_MEMSIM_V2_STATE_S;
        line->epoch = snoop->epoch;
        ack->status = CXL_MEMSIM_V2_STATUS_OK;
        ack->state = CXL_MEMSIM_V2_STATE_S;
        break;
    case CXL_MEMSIM_V2_OP_SNP_DATA_INV:
    case CXL_MEMSIM_V2_OP_SNP_DATA_DOWNGRADE:
        if (line->state != CXL_MEMSIM_V2_STATE_M) {
            break;
        }
        ack->payload_len = CXL_MEMSIM_V2_LINE_SIZE;
        memcpy(ack->data, line->data, sizeof(ack->data));
        line->dirty = false;
        line->epoch = snoop->epoch;
        if (snoop->type == CXL_MEMSIM_V2_OP_SNP_DATA_INV) {
            line->valid = false;
            line->state = CXL_MEMSIM_V2_STATE_I;
            ack->state = CXL_MEMSIM_V2_STATE_I;
        } else {
            line->state = CXL_MEMSIM_V2_STATE_S;
            ack->state = CXL_MEMSIM_V2_STATE_S;
        }
        ack->status = CXL_MEMSIM_V2_STATUS_OK;
        break;
    default:
        break;
    }
    qemu_mutex_unlock(&client->cache_lock);
    return true;
}

void cxl_memsim_v2_frame_init(CxlMemsimV2Frame *frame, CxlMemsimV2Opcode opcode) {
    memset(frame, 0, sizeof(*frame));
    frame->magic = CXL_MEMSIM_V2_MAGIC;
    frame->version = CXL_MEMSIM_V2_VERSION;
    frame->type = opcode;
}

void cxl_memsim_v2_encode_frame(const CxlMemsimV2Frame *frame, uint8_t bytes[CXL_MEMSIM_V2_FRAME_SIZE]) {
    memset(bytes, 0, CXL_MEMSIM_V2_FRAME_SIZE);
    stl_le_p(bytes + 0, frame->magic);
    stw_le_p(bytes + 4, frame->version);
    stw_le_p(bytes + 6, frame->type);
    stl_le_p(bytes + 8, frame->flags);
    stw_le_p(bytes + 12, frame->status);
    bytes[14] = frame->ack_strength;
    bytes[15] = frame->state;
    stw_le_p(bytes + 16, frame->src_host);
    stw_le_p(bytes + 18, frame->dst_host);
    stw_le_p(bytes + 20, frame->payload_len);
    stw_le_p(bytes + 22, frame->reserved0);
    stq_le_p(bytes + 24, frame->request_id);
    stq_le_p(bytes + 32, frame->snoop_id);
    stq_le_p(bytes + 40, frame->session_id);
    stq_le_p(bytes + 48, frame->addr);
    stq_le_p(bytes + 56, frame->epoch);
    stq_le_p(bytes + 64, frame->capabilities);
    stq_le_p(bytes + 72, frame->expected);
    stq_le_p(bytes + 80, frame->value);
    stq_le_p(bytes + 88, frame->old_value);
    stl_le_p(bytes + 96, frame->size);
    stl_le_p(bytes + 100, frame->reserved1);
    memcpy(bytes + 104, frame->data, sizeof(frame->data));
    memcpy(bytes + 168, frame->reserved, sizeof(frame->reserved));
}

static bool cxl_memsim_v2_frames_equal(const CxlMemsimV2Frame *left, const CxlMemsimV2Frame *right) {
    uint8_t left_bytes[CXL_MEMSIM_V2_FRAME_SIZE];
    uint8_t right_bytes[CXL_MEMSIM_V2_FRAME_SIZE];

    cxl_memsim_v2_encode_frame(left, left_bytes);
    cxl_memsim_v2_encode_frame(right, right_bytes);
    return memcmp(left_bytes, right_bytes, sizeof(left_bytes)) == 0;
}

bool cxl_memsim_v2_decode_frame(const uint8_t bytes[CXL_MEMSIM_V2_FRAME_SIZE], CxlMemsimV2Frame *frame, Error **errp) {
    CxlMemsimV2Frame decoded = {
        .magic = ldl_le_p(bytes + 0),
        .version = lduw_le_p(bytes + 4),
        .type = lduw_le_p(bytes + 6),
        .flags = ldl_le_p(bytes + 8),
        .status = lduw_le_p(bytes + 12),
        .ack_strength = bytes[14],
        .state = bytes[15],
        .src_host = lduw_le_p(bytes + 16),
        .dst_host = lduw_le_p(bytes + 18),
        .payload_len = lduw_le_p(bytes + 20),
        .reserved0 = lduw_le_p(bytes + 22),
        .request_id = ldq_le_p(bytes + 24),
        .snoop_id = ldq_le_p(bytes + 32),
        .session_id = ldq_le_p(bytes + 40),
        .addr = ldq_le_p(bytes + 48),
        .epoch = ldq_le_p(bytes + 56),
        .capabilities = ldq_le_p(bytes + 64),
        .expected = ldq_le_p(bytes + 72),
        .value = ldq_le_p(bytes + 80),
        .old_value = ldq_le_p(bytes + 88),
        .size = ldl_le_p(bytes + 96),
        .reserved1 = ldl_le_p(bytes + 100),
    };
    size_t i;

    memcpy(decoded.data, bytes + 104, sizeof(decoded.data));
    memcpy(decoded.reserved, bytes + 168, sizeof(decoded.reserved));
    if (decoded.magic != CXL_MEMSIM_V2_MAGIC || decoded.version != CXL_MEMSIM_V2_VERSION ||
        !cxl_memsim_v2_known_opcode(decoded.type) || decoded.flags || decoded.status > CXL_MEMSIM_V2_STATUS_IO_ERROR ||
        decoded.ack_strength > CXL_MEMSIM_V2_ACK_NATIVE || decoded.state > CXL_MEMSIM_V2_STATE_M ||
        decoded.payload_len > CXL_MEMSIM_V2_LINE_SIZE || decoded.reserved0 || decoded.reserved1) {
        error_setg(errp, "invalid CXLMemSim v2 frame envelope");
        return false;
    }
    for (i = decoded.payload_len; i < sizeof(decoded.data); i++) {
        if (decoded.data[i]) {
            error_setg(errp, "nonzero unused CXLMemSim v2 payload");
            return false;
        }
    }
    for (i = 0; i < sizeof(decoded.reserved); i++) {
        if (decoded.reserved[i]) {
            error_setg(errp, "nonzero CXLMemSim v2 reserved bytes");
            return false;
        }
    }
    *frame = decoded;
    return true;
}

static void cxl_memsim_v2_fail_connection_locked(CxlMemsimV2Client *client, const char *message) {
    GHashTableIter iter;
    gpointer value;

    client->connected = false;
    g_free(client->connection_error);
    client->connection_error = g_strdup(message);
    g_hash_table_iter_init(&iter, client->pending);
    while (g_hash_table_iter_next(&iter, NULL, &value)) {
        CxlMemsimV2Pending *pending = value;

        pending->io_failed = true;
        pending->done = true;
        qemu_cond_signal(&pending->completed);
    }
}

static void cxl_memsim_v2_fail_connection(CxlMemsimV2Client *client, const char *message) {
    qemu_mutex_lock(&client->state_lock);
    cxl_memsim_v2_fail_connection_locked(client, message);
    qemu_mutex_unlock(&client->state_lock);
}

static void cxl_memsim_v2_disconnect(CxlMemsimV2Client *client, const char *message) {
    cxl_memsim_v2_fail_connection(client, message);
    if (client->socket) {
        qio_channel_shutdown(QIO_CHANNEL(client->socket), QIO_CHANNEL_SHUTDOWN_BOTH, NULL);
    }
}

static bool cxl_memsim_v2_write_frame(CxlMemsimV2Client *client, const CxlMemsimV2Frame *frame, Error **errp) {
    uint8_t bytes[CXL_MEMSIM_V2_FRAME_SIZE];
    int result;

    cxl_memsim_v2_encode_frame(frame, bytes);
    qemu_mutex_lock(&client->send_lock);
    result = qio_channel_write_all(QIO_CHANNEL(client->socket), (const char *)bytes, sizeof(bytes), errp);
    qemu_mutex_unlock(&client->send_lock);
    return result == 0;
}

static bool cxl_memsim_v2_validate_response(CxlMemsimV2Client *client, const CxlMemsimV2Frame *request,
                                            const CxlMemsimV2Frame *response) {
    if (response->type != CXL_MEMSIM_V2_OP_RESPONSE || response->src_host != CXL_MEMSIM_V2_SERVER_ENDPOINT ||
        response->dst_host != client->endpoint || response->request_id != request->request_id || response->snoop_id ||
        response->addr != request->addr) {
        return false;
    }
    if (request->type == CXL_MEMSIM_V2_OP_REGISTER) {
        return response->status == CXL_MEMSIM_V2_STATUS_OK ? response->session_id != 0
                                                           : response->session_id == request->session_id;
    }
    return response->session_id == request->session_id && response->session_id != 0;
}

static bool cxl_memsim_v2_send_snoop_ack(CxlMemsimV2Client *client, const CxlMemsimV2Frame *snoop) {
    CxlMemsimV2Frame ack;
    Error *local_err = NULL;
    bool handled = false;

    cxl_memsim_v2_frame_init(&ack, CXL_MEMSIM_V2_OP_SNOOP_ACK);
    ack.status = CXL_MEMSIM_V2_STATUS_INVALID_STATE;
    ack.ack_strength = CXL_MEMSIM_V2_ACK_MODEL;
    ack.state = CXL_MEMSIM_V2_STATE_I;
    ack.src_host = client->endpoint;
    ack.dst_host = CXL_MEMSIM_V2_SERVER_ENDPOINT;
    ack.snoop_id = snoop->snoop_id;
    ack.session_id = client->session_id;
    ack.addr = snoop->addr;
    ack.epoch = snoop->epoch;
    if (client->snoop_handler) {
        handled = client->snoop_handler(client->snoop_opaque, snoop, &ack, &local_err);
    }
    if (!handled && !local_err) {
        handled = cxl_memsim_v2_cache_snoop(client, snoop, &ack);
    }
    if (!handled && local_err) {
        error_report_err(local_err);
        local_err = NULL;
    }
    if (ack.payload_len > CXL_MEMSIM_V2_LINE_SIZE || !cxl_memsim_v2_write_frame(client, &ack, &local_err)) {
        if (local_err) {
            error_report_err(local_err);
        }
        return false;
    }
    return true;
}

static bool cxl_memsim_v2_install_grant(CxlMemsimV2Client *client, const CxlMemsimV2Frame *request,
                                        const CxlMemsimV2Frame *response) {
    CxlMemsimV2CacheLine *line;
    uint64_t line_address;
    bool acquire;

    if (response->status != CXL_MEMSIM_V2_STATUS_OK) {
        return true;
    }
    acquire = request->type == CXL_MEMSIM_V2_OP_GETS || request->type == CXL_MEMSIM_V2_OP_GETM ||
              request->type == CXL_MEMSIM_V2_OP_ATOMIC_FAA || request->type == CXL_MEMSIM_V2_OP_ATOMIC_CAS;
    if (!acquire && request->type != CXL_MEMSIM_V2_OP_UPGRADE) {
        return true;
    }
    line_address = request->addr & ~(uint64_t)(CXL_MEMSIM_V2_LINE_SIZE - 1);
    qemu_mutex_lock(&client->cache_lock);
    if (request->type == CXL_MEMSIM_V2_OP_UPGRADE) {
        line = cxl_memsim_v2_cache_find_locked(client, line_address);
        if (!line || response->state != CXL_MEMSIM_V2_STATE_M || response->epoch <= line->epoch ||
            response->payload_len) {
            qemu_mutex_unlock(&client->cache_lock);
            return false;
        }
        line->state = CXL_MEMSIM_V2_STATE_M;
        line->epoch = response->epoch;
        cxl_memsim_v2_cache_touch_locked(client, line);
        qemu_mutex_unlock(&client->cache_lock);
        return true;
    }
    if (response->payload_len != CXL_MEMSIM_V2_LINE_SIZE || !response->epoch ||
        (request->type == CXL_MEMSIM_V2_OP_GETS
             ? response->state != CXL_MEMSIM_V2_STATE_S && response->state != CXL_MEMSIM_V2_STATE_E
             : response->state != CXL_MEMSIM_V2_STATE_M)) {
        qemu_mutex_unlock(&client->cache_lock);
        return false;
    }
    line = cxl_memsim_v2_cache_find_locked(client, line_address);
    if (!line) {
        line = cxl_memsim_v2_cache_victim_locked(client, line_address);
    }
    if (line->valid && line->address != line_address) {
        qemu_mutex_unlock(&client->cache_lock);
        return false;
    }
    memset(line, 0, sizeof(*line));
    line->address = line_address;
    line->epoch = response->epoch;
    line->state = response->state;
    line->valid = true;
    memcpy(line->data, response->data, sizeof(line->data));
    cxl_memsim_v2_cache_touch_locked(client, line);
    qemu_mutex_unlock(&client->cache_lock);
    return true;
}

static void *cxl_memsim_v2_progress(void *opaque) {
    CxlMemsimV2Client *client = opaque;
    uint8_t bytes[CXL_MEMSIM_V2_FRAME_SIZE];

    qemu_mutex_lock(&client->state_lock);
    client->progress_starts++;
    qemu_mutex_unlock(&client->state_lock);

    for (;;) {
        CxlMemsimV2Frame frame;
        CxlMemsimV2Pending *pending;
        Error *local_err = NULL;

        if (qio_channel_read_all(QIO_CHANNEL(client->socket), (char *)bytes, sizeof(bytes), &local_err) < 0) {
            const char *message = local_err ? error_get_pretty(local_err) : "CXLMemSim v2 read failed";

            cxl_memsim_v2_fail_connection(client, message);
            error_free(local_err);
            break;
        }
        if (!cxl_memsim_v2_decode_frame(bytes, &frame, &local_err)) {
            const char *message = error_get_pretty(local_err);

            cxl_memsim_v2_fail_connection(client, message);
            error_free(local_err);
            break;
        }
        if (frame.type == CXL_MEMSIM_V2_OP_RESPONSE) {
            CxlMemsimV2Frame *completed;

            qemu_mutex_lock(&client->state_lock);
            pending = g_hash_table_lookup(client->pending, &frame.request_id);
            if (!pending) {
                completed = g_hash_table_lookup(client->completed_retries, &frame.request_id);
                if (completed && cxl_memsim_v2_frames_equal(completed, &frame)) {
                    qemu_mutex_unlock(&client->state_lock);
                    continue;
                }
                qemu_mutex_unlock(&client->state_lock);
                cxl_memsim_v2_fail_connection(client, "uncorrelated CXLMemSim v2 response");
                break;
            }
            if (!cxl_memsim_v2_validate_response(client, &pending->request, &frame)) {
                qemu_mutex_unlock(&client->state_lock);
                cxl_memsim_v2_fail_connection(client, "uncorrelated CXLMemSim v2 response");
                break;
            }
            if (pending->done) {
                if (pending->retried && cxl_memsim_v2_frames_equal(&pending->response, &frame)) {
                    qemu_mutex_unlock(&client->state_lock);
                    continue;
                }
                qemu_mutex_unlock(&client->state_lock);
                cxl_memsim_v2_fail_connection(client, "conflicting CXLMemSim v2 duplicate response");
                break;
            }
            if (!cxl_memsim_v2_install_grant(client, &pending->request, &frame)) {
                qemu_mutex_unlock(&client->state_lock);
                cxl_memsim_v2_fail_connection(client, "invalid or unreserved CXLMemSim v2 grant");
                break;
            }
            pending->response = frame;
            pending->done = true;
            qemu_cond_signal(&pending->completed);
            qemu_mutex_unlock(&client->state_lock);
            continue;
        }
        if (!cxl_memsim_v2_is_snoop(frame.type) || frame.src_host != CXL_MEMSIM_V2_SERVER_ENDPOINT ||
            frame.dst_host != client->endpoint || frame.session_id != client->session_id || !frame.snoop_id ||
            !cxl_memsim_v2_send_snoop_ack(client, &frame)) {
            cxl_memsim_v2_fail_connection(client, "invalid CXLMemSim v2 snoop or ACK failure");
            break;
        }
    }

    qemu_mutex_lock(&client->state_lock);
    client->running = false;
    qemu_mutex_unlock(&client->state_lock);
    return NULL;
}

CxlMemsimV2Client *cxl_memsim_v2_client_new(uint16_t endpoint, CxlMemsimV2SnoopHandler snoop_handler,
                                            void *snoop_opaque) {
    CxlMemsimV2Client *client;

    if (endpoint >= CXL_MEMSIM_V2_MAX_ENDPOINTS) {
        return NULL;
    }
    client = g_new0(CxlMemsimV2Client, 1);
    client->endpoint = endpoint;
    client->next_request_id = 1;
    client->write_policy = CXL_MEMSIM_V2_WRITE_BACK;
    client->snoop_handler = snoop_handler;
    client->snoop_opaque = snoop_opaque;
    client->pending = g_hash_table_new(g_int64_hash, g_int64_equal);
    client->completed_retries = g_hash_table_new_full(g_int64_hash, g_int64_equal, g_free, g_free);
    qemu_mutex_init(&client->state_lock);
    qemu_mutex_init(&client->send_lock);
    qemu_mutex_init(&client->operation_lock);
    qemu_mutex_init(&client->cache_lock);
    return client;
}

bool cxl_memsim_v2_client_set_write_policy(CxlMemsimV2Client *client, CxlMemsimV2WritePolicy policy, Error **errp) {
    if (!client || (policy != CXL_MEMSIM_V2_WRITE_BACK && policy != CXL_MEMSIM_V2_WRITE_THROUGH)) {
        error_setg(errp, "invalid CXLMemSim v2 write policy");
        return false;
    }
    qemu_mutex_lock(&client->state_lock);
    if (client->started) {
        qemu_mutex_unlock(&client->state_lock);
        error_setg(errp, "CXLMemSim v2 write policy is already active");
        return false;
    }
    client->write_policy = policy;
    qemu_mutex_unlock(&client->state_lock);
    return true;
}

static bool cxl_memsim_v2_transact_internal(CxlMemsimV2Client *client, CxlMemsimV2Frame *request,
                                            CxlMemsimV2Frame *response, int timeout_ms, bool registration,
                                            Error **errp) {
    CxlMemsimV2Pending pending = {
        .request = *request,
    };
    int64_t deadline_us;
    unsigned attempt;
    bool success = false;
    bool disconnect = false;

    if (!client || !request || !response || timeout_ms <= 0) {
        error_setg(errp, "invalid CXLMemSim v2 transaction arguments");
        return false;
    }
    qemu_cond_init(&pending.completed);
    qemu_mutex_lock(&client->state_lock);
    if (!client->connected) {
        error_setg(errp, "%s", client->connection_error ?: "CXLMemSim v2 client is disconnected");
        goto out_locked;
    }
    if (registration) {
        pending.request_id = 0;
    } else {
        if (!client->session_id || !client->next_request_id || client->next_request_id == UINT64_MAX) {
            error_setg(errp, "CXLMemSim v2 session or request ID unavailable");
            goto out_locked;
        }
        pending.request_id = client->next_request_id++;
        request->session_id = client->session_id;
    }
    request->magic = CXL_MEMSIM_V2_MAGIC;
    request->version = CXL_MEMSIM_V2_VERSION;
    request->src_host = client->endpoint;
    request->dst_host = CXL_MEMSIM_V2_SERVER_ENDPOINT;
    request->request_id = pending.request_id;
    pending.request = *request;
    if (g_hash_table_contains(client->pending, &pending.request_id)) {
        error_setg(errp, "duplicate CXLMemSim v2 pending request");
        goto out_locked;
    }
    g_hash_table_insert(client->pending, &pending.request_id, &pending);
    qemu_mutex_unlock(&client->state_lock);

    if (!cxl_memsim_v2_write_frame(client, request, errp)) {
        cxl_memsim_v2_fail_connection(client, "CXLMemSim v2 frame write failed");
        qemu_mutex_lock(&client->state_lock);
        goto remove_locked;
    }

    qemu_mutex_lock(&client->state_lock);
    for (attempt = 0; attempt < 2 && !pending.done; attempt++) {
        deadline_us = g_get_monotonic_time() + timeout_ms * G_TIME_SPAN_MILLISECOND;
        while (!pending.done) {
            int64_t remaining_us = deadline_us - g_get_monotonic_time();
            int remaining_ms;

            if (remaining_us <= 0) {
                break;
            }
            remaining_ms = DIV_ROUND_UP(remaining_us, G_TIME_SPAN_MILLISECOND);
            if (!qemu_cond_timedwait(&pending.completed, &client->state_lock, remaining_ms) && !pending.done) {
                break;
            }
        }
        if (!pending.done && attempt == 0) {
            Error *retry_err = NULL;
            bool retry_sent;

            pending.retried = true;
            qemu_mutex_unlock(&client->state_lock);
            retry_sent = cxl_memsim_v2_write_frame(client, &pending.request, &retry_err);
            qemu_mutex_lock(&client->state_lock);
            if (!retry_sent) {
                const char *message = retry_err ? error_get_pretty(retry_err) : "CXLMemSim v2 retry write failed";
                bool completed = !registration && pending.done && !pending.io_failed;

                cxl_memsim_v2_fail_connection_locked(client, message);
                if (completed) {
                    pending.io_failed = false;
                }
                error_free(retry_err);
                disconnect = true;
            }
        }
    }
    if (!pending.done) {
        cxl_memsim_v2_fail_connection_locked(client, "CXLMemSim v2 request timed out ambiguously");
        error_setg(errp, "CXLMemSim v2 request timed out ambiguously");
        disconnect = true;
    } else if (pending.io_failed) {
        error_setg(errp, "%s", client->connection_error ?: "CXLMemSim v2 connection failed");
    } else {
        *response = pending.response;
        success = true;
    }

remove_locked:
    if (success && pending.retried) {
        uint64_t *request_id = g_new(uint64_t, 1);
        CxlMemsimV2Frame *completed = g_new(CxlMemsimV2Frame, 1);

        *request_id = pending.request_id;
        *completed = pending.response;
        g_hash_table_replace(client->completed_retries, request_id, completed);
    }
    g_hash_table_remove(client->pending, &pending.request_id);
out_locked:
    qemu_mutex_unlock(&client->state_lock);
    qemu_cond_destroy(&pending.completed);
    if (disconnect) {
        qio_channel_shutdown(QIO_CHANNEL(client->socket), QIO_CHANNEL_SHUTDOWN_BOTH, NULL);
    }
    return success;
}

bool cxl_memsim_v2_client_start_fd(CxlMemsimV2Client *client, int fd, uint32_t cache_capacity, uint16_t cache_ways,
                                   int timeout_ms, Error **errp) {
    CxlMemsimV2Frame request;
    CxlMemsimV2Frame response;
    Error *local_err = NULL;

    if (!client || fd < 0 || cache_capacity < CXL_MEMSIM_V2_LINE_SIZE || cache_capacity % CXL_MEMSIM_V2_LINE_SIZE ||
        !cache_ways || (cache_capacity / CXL_MEMSIM_V2_LINE_SIZE) % cache_ways) {
        if (fd >= 0) {
            close(fd);
        }
        error_setg(errp, "invalid CXLMemSim v2 registration geometry");
        return false;
    }
    qemu_mutex_lock(&client->state_lock);
    if (client->started) {
        qemu_mutex_unlock(&client->state_lock);
        close(fd);
        error_setg(errp, "CXLMemSim v2 client is already started");
        return false;
    }
    client->socket = qio_channel_socket_new_fd(fd, &local_err);
    if (!client->socket) {
        qemu_mutex_unlock(&client->state_lock);
        error_propagate(errp, local_err);
        return false;
    }
    client->cache_line_count = cache_capacity / CXL_MEMSIM_V2_LINE_SIZE;
    client->cache_set_count = client->cache_line_count / cache_ways;
    client->cache_ways = cache_ways;
    client->timeout_ms = timeout_ms;
    client->cache = g_new0(CxlMemsimV2CacheLine, client->cache_line_count);
    client->started = true;
    client->running = true;
    client->connected = true;
    qemu_mutex_unlock(&client->state_lock);

    qemu_thread_create(&client->progress_thread, "cxl-memsim-v2", cxl_memsim_v2_progress, client, QEMU_THREAD_JOINABLE);
    qemu_mutex_lock(&client->state_lock);
    client->progress_joinable = true;
    qemu_mutex_unlock(&client->state_lock);

    cxl_memsim_v2_frame_init(&request, CXL_MEMSIM_V2_OP_REGISTER);
    request.capabilities = CXL_MEMSIM_V2_CAP_MODEL_SNOOP;
    request.expected = cache_ways;
    request.value = cache_capacity;
    request.size = CXL_MEMSIM_V2_LINE_SIZE;
    if (!cxl_memsim_v2_transact_internal(client, &request, &response, timeout_ms, true, &local_err)) {
        goto fail;
    }
    if (response.status != CXL_MEMSIM_V2_STATUS_OK || response.ack_strength != CXL_MEMSIM_V2_ACK_MODEL ||
        response.capabilities != CXL_MEMSIM_V2_CAP_MODEL_SNOOP || response.size != CXL_MEMSIM_V2_LINE_SIZE ||
        response.value != cache_capacity || response.expected != cache_ways || !response.old_value) {
        error_setg(&local_err, "CXLMemSim v2 registration response is invalid");
        goto fail;
    }
    qemu_mutex_lock(&client->state_lock);
    if (!client->connected) {
        error_setg(&local_err, "%s", client->connection_error ?: "CXLMemSim v2 registration disconnected");
        qemu_mutex_unlock(&client->state_lock);
        goto fail;
    }
    client->session_id = response.session_id;
    qemu_mutex_unlock(&client->state_lock);
    return true;

fail:
    qio_channel_shutdown(QIO_CHANNEL(client->socket), QIO_CHANNEL_SHUTDOWN_BOTH, NULL);
    qemu_thread_join(&client->progress_thread);
    qemu_mutex_lock(&client->state_lock);
    client->progress_joinable = false;
    client->running = false;
    client->connected = false;
    qemu_mutex_unlock(&client->state_lock);
    error_propagate(errp, local_err);
    return false;
}

bool cxl_memsim_v2_client_connect(CxlMemsimV2Client *client, const char *host, uint16_t port, uint32_t cache_capacity,
                                  uint16_t cache_ways, int timeout_ms, Error **errp) {
    SocketAddress address = {
        .type = SOCKET_ADDRESS_TYPE_INET,
    };
    QIOChannelSocket *socket;
    Error *local_err = NULL;
    int fd;

    if (!host || !host[0] || !port) {
        error_setg(errp, "invalid CXLMemSim v2 TCP address");
        return false;
    }
    address.u.inet.host = (char *)host;
    address.u.inet.port = g_strdup_printf("%u", port);
    socket = qio_channel_socket_new();
    if (qio_channel_socket_connect_sync(socket, &address, &local_err) < 0) {
        object_unref(OBJECT(socket));
        g_free(address.u.inet.port);
        error_propagate(errp, local_err);
        return false;
    }
    fd = dup(socket->fd);
    object_unref(OBJECT(socket));
    g_free(address.u.inet.port);
    if (fd < 0) {
        error_setg_errno(errp, errno, "cannot duplicate CXLMemSim v2 socket");
        return false;
    }
    return cxl_memsim_v2_client_start_fd(client, fd, cache_capacity, cache_ways, timeout_ms, errp);
}

bool cxl_memsim_v2_client_transact(CxlMemsimV2Client *client, CxlMemsimV2Frame *request, CxlMemsimV2Frame *response,
                                   int timeout_ms, Error **errp) {
    CxlMemsimV2Frame heartbeat;
    CxlMemsimV2Frame heartbeat_response;
    Error *heartbeat_err = NULL;
    uint64_t acknowledge_through = 0;
    bool success;

    if (!request || request->type == CXL_MEMSIM_V2_OP_REGISTER || request->type == CXL_MEMSIM_V2_OP_RESPONSE ||
        request->type == CXL_MEMSIM_V2_OP_SNOOP_ACK || cxl_memsim_v2_is_snoop(request->type)) {
        error_setg(errp, "invalid CXLMemSim v2 client request opcode");
        return false;
    }
    success = cxl_memsim_v2_transact_internal(client, request, response, timeout_ms, false, errp);
    if (!success || request->type == CXL_MEMSIM_V2_OP_HEARTBEAT) {
        return success;
    }

    qemu_mutex_lock(&client->state_lock);
    client->consumed_response_id = MAX(client->consumed_response_id, response->request_id);
    if (!client->response_ack_in_progress &&
        client->consumed_response_id - client->acknowledged_response_id >= CXL_MEMSIM_V2_RESPONSE_ACK_INTERVAL) {
        client->response_ack_in_progress = true;
        acknowledge_through = client->consumed_response_id;
    }
    qemu_mutex_unlock(&client->state_lock);
    if (!acknowledge_through) {
        return true;
    }

    cxl_memsim_v2_frame_init(&heartbeat, CXL_MEMSIM_V2_OP_HEARTBEAT);
    heartbeat.old_value = acknowledge_through;
    success =
        cxl_memsim_v2_transact_internal(client, &heartbeat, &heartbeat_response, timeout_ms, false, &heartbeat_err);
    if (success && heartbeat_response.status != CXL_MEMSIM_V2_STATUS_OK) {
        error_setg(&heartbeat_err, "CXLMemSim v2 heartbeat status %u", heartbeat_response.status);
        success = false;
    }
    if (success && (heartbeat_response.state != CXL_MEMSIM_V2_STATE_I || heartbeat_response.epoch ||
                    heartbeat_response.payload_len)) {
        error_setg(&heartbeat_err, "invalid CXLMemSim v2 heartbeat response");
        success = false;
    }

    if (!success) {
        const char *message = heartbeat_err ? error_get_pretty(heartbeat_err) : "CXLMemSim v2 heartbeat failed";

        cxl_memsim_v2_disconnect(client, message);
    }

    qemu_mutex_lock(&client->state_lock);
    if (success) {
        GHashTableIter iter;
        gpointer key;

        client->acknowledged_response_id = MAX(client->acknowledged_response_id, acknowledge_through);
        g_hash_table_iter_init(&iter, client->completed_retries);
        while (g_hash_table_iter_next(&iter, &key, NULL)) {
            if (*(uint64_t *)key <= acknowledge_through) {
                g_hash_table_iter_remove(&iter);
            }
        }
    }
    client->response_ack_in_progress = false;
    qemu_mutex_unlock(&client->state_lock);
    error_free(heartbeat_err);
    return true;
}

static bool cxl_memsim_v2_response_ok(const CxlMemsimV2Frame *response, Error **errp) {
    if (response->status == CXL_MEMSIM_V2_STATUS_OK) {
        return true;
    }
    error_setg(errp, "CXLMemSim v2 server status %u", response->status);
    return false;
}

static bool cxl_memsim_v2_acquire_line(CxlMemsimV2Client *client, uint64_t line_address, bool modified,
                                       CxlMemsimV2Frame *response, int timeout_ms, Error **errp) {
    CxlMemsimV2Frame request;

    cxl_memsim_v2_frame_init(&request, modified ? CXL_MEMSIM_V2_OP_GETM : CXL_MEMSIM_V2_OP_GETS);
    request.addr = line_address;
    request.state = CXL_MEMSIM_V2_STATE_I;
    if (!cxl_memsim_v2_client_transact(client, &request, response, timeout_ms, errp) ||
        !cxl_memsim_v2_response_ok(response, errp)) {
        return false;
    }
    if (response->payload_len != CXL_MEMSIM_V2_LINE_SIZE ||
        (modified ? response->state != CXL_MEMSIM_V2_STATE_M
                  : (response->state != CXL_MEMSIM_V2_STATE_S && response->state != CXL_MEMSIM_V2_STATE_E)) ||
        !response->epoch) {
        error_setg(errp, "invalid CXLMemSim v2 line grant");
        return false;
    }
    return true;
}

static bool cxl_memsim_v2_release_line(CxlMemsimV2Client *client, uint64_t line_address, const CxlMemsimV2Frame *grant,
                                       bool dirty, int timeout_ms, Error **errp) {
    CxlMemsimV2Frame request;
    CxlMemsimV2Frame response;

    cxl_memsim_v2_frame_init(&request, dirty ? CXL_MEMSIM_V2_OP_PUTM : CXL_MEMSIM_V2_OP_PUTS);
    request.addr = line_address;
    request.state = grant->state;
    request.epoch = grant->epoch;
    if (dirty) {
        request.payload_len = CXL_MEMSIM_V2_LINE_SIZE;
        memcpy(request.data, grant->data, sizeof(request.data));
    }
    if (!cxl_memsim_v2_client_transact(client, &request, &response, timeout_ms, errp) ||
        !cxl_memsim_v2_response_ok(&response, errp)) {
        return false;
    }
    if (response.state != CXL_MEMSIM_V2_STATE_I || response.epoch <= grant->epoch || response.payload_len) {
        error_setg(errp, "invalid CXLMemSim v2 line release");
        return false;
    }
    return true;
}

static bool cxl_memsim_v2_release_cached_line(CxlMemsimV2Client *client, const CxlMemsimV2CacheLine *line,
                                              int timeout_ms, Error **errp) {
    CxlMemsimV2Frame grant;

    cxl_memsim_v2_frame_init(&grant, CXL_MEMSIM_V2_OP_RESPONSE);
    grant.state = line->state;
    grant.epoch = line->epoch;
    memcpy(grant.data, line->data, sizeof(grant.data));
    return cxl_memsim_v2_release_line(client, line->address, &grant, line->state == CXL_MEMSIM_V2_STATE_M, timeout_ms,
                                      errp);
}

static bool cxl_memsim_v2_cache_evict_address(CxlMemsimV2Client *client, uint64_t line_address, int timeout_ms,
                                              Error **errp) {
    CxlMemsimV2CacheLine snapshot;
    CxlMemsimV2CacheLine *line;

    qemu_mutex_lock(&client->cache_lock);
    line = cxl_memsim_v2_cache_find_locked(client, line_address);
    if (!line) {
        qemu_mutex_unlock(&client->cache_lock);
        return true;
    }
    snapshot = *line;
    qemu_mutex_unlock(&client->cache_lock);

    if (!cxl_memsim_v2_release_cached_line(client, &snapshot, timeout_ms, errp)) {
        return false;
    }
    qemu_mutex_lock(&client->cache_lock);
    line = cxl_memsim_v2_cache_find_locked(client, line_address);
    if (line) {
        memset(line, 0, sizeof(*line));
    }
    qemu_mutex_unlock(&client->cache_lock);
    return true;
}

static bool cxl_memsim_v2_cache_make_room(CxlMemsimV2Client *client, uint64_t line_address, int timeout_ms,
                                          Error **errp) {
    uint64_t victim_address;
    CxlMemsimV2CacheLine *victim;

    qemu_mutex_lock(&client->cache_lock);
    victim = cxl_memsim_v2_cache_victim_locked(client, line_address);
    if (!victim->valid) {
        qemu_mutex_unlock(&client->cache_lock);
        return true;
    }
    victim_address = victim->address;
    qemu_mutex_unlock(&client->cache_lock);
    return cxl_memsim_v2_cache_evict_address(client, victim_address, timeout_ms, errp);
}

static bool cxl_memsim_v2_upgrade_line(CxlMemsimV2Client *client, const CxlMemsimV2CacheLine *line,
                                       CxlMemsimV2Frame *response, int timeout_ms, Error **errp) {
    CxlMemsimV2Frame request;

    cxl_memsim_v2_frame_init(&request, CXL_MEMSIM_V2_OP_UPGRADE);
    request.addr = line->address;
    request.state = line->state;
    request.epoch = line->epoch;
    if (!cxl_memsim_v2_client_transact(client, &request, response, timeout_ms, errp) ||
        !cxl_memsim_v2_response_ok(response, errp)) {
        return false;
    }
    if (response->state != CXL_MEMSIM_V2_STATE_M || response->epoch <= line->epoch || response->payload_len) {
        error_setg(errp, "invalid CXLMemSim v2 upgrade grant");
        return false;
    }
    return true;
}

static bool cxl_memsim_v2_cache_ensure(CxlMemsimV2Client *client, uint64_t line_address, bool modified, int timeout_ms,
                                       Error **errp) {
    for (;;) {
        CxlMemsimV2CacheLine snapshot;
        CxlMemsimV2CacheLine *line;
        CxlMemsimV2Frame grant;

        qemu_mutex_lock(&client->cache_lock);
        line = cxl_memsim_v2_cache_find_locked(client, line_address);
        if (line && (!modified || line->state == CXL_MEMSIM_V2_STATE_M)) {
            cxl_memsim_v2_cache_touch_locked(client, line);
            qemu_mutex_unlock(&client->cache_lock);
            return true;
        }
        if (line) {
            snapshot = *line;
            qemu_mutex_unlock(&client->cache_lock);
            if (!cxl_memsim_v2_upgrade_line(client, &snapshot, &grant, timeout_ms, errp)) {
                return false;
            }
            continue;
        }
        qemu_mutex_unlock(&client->cache_lock);

        if (!cxl_memsim_v2_cache_make_room(client, line_address, timeout_ms, errp) ||
            !cxl_memsim_v2_acquire_line(client, line_address, modified, &grant, timeout_ms, errp)) {
            return false;
        }
    }
}

static bool cxl_memsim_v2_access_size_valid(uint64_t address, unsigned size) {
    return (size == 1 || size == 2 || size == 4 || size == 8) && address <= UINT64_MAX - size;
}

bool cxl_memsim_v2_load(CxlMemsimV2Client *client, uint64_t address, unsigned size, uint64_t *value, int timeout_ms,
                        Error **errp) {
    uint8_t bytes[sizeof(*value)] = {0};
    uint64_t cursor = address;
    unsigned copied = 0;
    bool success = false;

    if (!client || !value || !cxl_memsim_v2_access_size_valid(address, size)) {
        error_setg(errp, "invalid CXLMemSim v2 load");
        return false;
    }
    qemu_mutex_lock(&client->operation_lock);
    while (copied < size) {
        CxlMemsimV2CacheLine *line;
        uint64_t line_address = cursor & ~(uint64_t)(CXL_MEMSIM_V2_LINE_SIZE - 1);
        unsigned line_offset = cursor & (CXL_MEMSIM_V2_LINE_SIZE - 1);
        unsigned chunk = MIN(size - copied, CXL_MEMSIM_V2_LINE_SIZE - line_offset);

        if (!cxl_memsim_v2_cache_ensure(client, line_address, false, timeout_ms, errp)) {
            goto out;
        }
        qemu_mutex_lock(&client->state_lock);
        if (!client->connected) {
            error_setg(errp, "%s", client->connection_error ?: "CXLMemSim v2 client is disconnected");
            qemu_mutex_unlock(&client->state_lock);
            goto out;
        }
        qemu_mutex_lock(&client->cache_lock);
        line = cxl_memsim_v2_cache_find_locked(client, line_address);
        if (!line) {
            qemu_mutex_unlock(&client->cache_lock);
            qemu_mutex_unlock(&client->state_lock);
            error_setg(errp, "CXLMemSim v2 load grant was invalidated");
            goto out;
        }
        memcpy(bytes + copied, line->data + line_offset, chunk);
        cxl_memsim_v2_cache_touch_locked(client, line);
        qemu_mutex_unlock(&client->cache_lock);
        qemu_mutex_unlock(&client->state_lock);
        cursor += chunk;
        copied += chunk;
    }
    *value = ldn_le_p(bytes, size);
    success = true;

out:
    qemu_mutex_unlock(&client->operation_lock);
    return success;
}

bool cxl_memsim_v2_store(CxlMemsimV2Client *client, uint64_t address, unsigned size, uint64_t value, int timeout_ms,
                         Error **errp) {
    uint8_t bytes[sizeof(value)] = {0};
    uint64_t cursor = address;
    unsigned copied = 0;
    bool success = false;

    if (!client || !cxl_memsim_v2_access_size_valid(address, size)) {
        error_setg(errp, "invalid CXLMemSim v2 store");
        return false;
    }
    stn_le_p(bytes, size, value);
    qemu_mutex_lock(&client->operation_lock);
    while (copied < size) {
        CxlMemsimV2CacheLine *line;
        uint64_t line_address = cursor & ~(uint64_t)(CXL_MEMSIM_V2_LINE_SIZE - 1);
        unsigned line_offset = cursor & (CXL_MEMSIM_V2_LINE_SIZE - 1);
        unsigned chunk = MIN(size - copied, CXL_MEMSIM_V2_LINE_SIZE - line_offset);

        if (!cxl_memsim_v2_cache_ensure(client, line_address, true, timeout_ms, errp)) {
            goto out;
        }
        qemu_mutex_lock(&client->state_lock);
        if (!client->connected) {
            error_setg(errp, "%s", client->connection_error ?: "CXLMemSim v2 client is disconnected");
            qemu_mutex_unlock(&client->state_lock);
            goto out;
        }
        qemu_mutex_lock(&client->cache_lock);
        line = cxl_memsim_v2_cache_find_locked(client, line_address);
        if (!line || line->state != CXL_MEMSIM_V2_STATE_M) {
            qemu_mutex_unlock(&client->cache_lock);
            qemu_mutex_unlock(&client->state_lock);
            error_setg(errp, "CXLMemSim v2 store grant was invalidated");
            goto out;
        }
        memcpy(line->data + line_offset, bytes + copied, chunk);
        line->dirty = true;
        cxl_memsim_v2_cache_touch_locked(client, line);
        qemu_mutex_unlock(&client->cache_lock);
        qemu_mutex_unlock(&client->state_lock);
        if (client->write_policy == CXL_MEMSIM_V2_WRITE_THROUGH &&
            !cxl_memsim_v2_cache_evict_address(client, line_address, timeout_ms, errp)) {
            goto out;
        }
        cursor += chunk;
        copied += chunk;
    }
    success = true;

out:
    qemu_mutex_unlock(&client->operation_lock);
    return success;
}

static bool cxl_memsim_v2_atomic(CxlMemsimV2Client *client, CxlMemsimV2Opcode opcode, uint64_t address,
                                 uint64_t expected, uint64_t operand, uint64_t *old_value, uint64_t *new_value,
                                 int timeout_ms, Error **errp) {
    CxlMemsimV2Frame request;
    CxlMemsimV2Frame response;
    Error *release_err = NULL;
    uint64_t line_address;
    unsigned line_offset;
    bool success = false;

    if (!client || !old_value || !new_value || address % 8 ||
        (address & (CXL_MEMSIM_V2_LINE_SIZE - 1)) > CXL_MEMSIM_V2_LINE_SIZE - sizeof(uint64_t)) {
        error_setg(errp, "invalid CXLMemSim v2 atomic address");
        return false;
    }
    qemu_mutex_lock(&client->operation_lock);
    line_address = address & ~(uint64_t)(CXL_MEMSIM_V2_LINE_SIZE - 1);
    if (!cxl_memsim_v2_cache_evict_address(client, line_address, timeout_ms, errp) ||
        !cxl_memsim_v2_cache_make_room(client, line_address, timeout_ms, errp)) {
        goto out;
    }
    cxl_memsim_v2_frame_init(&request, opcode);
    request.addr = address;
    request.state = CXL_MEMSIM_V2_STATE_I;
    request.expected = expected;
    request.value = operand;
    request.size = sizeof(uint64_t);
    if (!cxl_memsim_v2_client_transact(client, &request, &response, timeout_ms, errp) ||
        !cxl_memsim_v2_response_ok(&response, errp)) {
        goto out;
    }
    if (response.state != CXL_MEMSIM_V2_STATE_M || response.payload_len != CXL_MEMSIM_V2_LINE_SIZE || !response.epoch) {
        error_setg(errp, "invalid CXLMemSim v2 atomic grant");
        goto out;
    }
    line_offset = address & (CXL_MEMSIM_V2_LINE_SIZE - 1);
    *old_value = response.old_value;
    *new_value = ldq_le_p(response.data + line_offset);
    if (!cxl_memsim_v2_cache_evict_address(client, line_address, timeout_ms, &release_err)) {
        const char *message = release_err ? error_get_pretty(release_err) : "CXLMemSim v2 atomic grant release failed";
        CxlMemsimV2CacheLine *line;

        cxl_memsim_v2_disconnect(client, message);
        qemu_mutex_lock(&client->cache_lock);
        line = cxl_memsim_v2_cache_find_locked(client, line_address);
        if (line) {
            memset(line, 0, sizeof(*line));
        }
        qemu_mutex_unlock(&client->cache_lock);
        error_free(release_err);
    }
    success = true;

out:
    qemu_mutex_unlock(&client->operation_lock);
    return success;
}

bool cxl_memsim_v2_fetch_add(CxlMemsimV2Client *client, uint64_t address, uint64_t addend, uint64_t *old_value,
                             uint64_t *new_value, int timeout_ms, Error **errp) {
    return cxl_memsim_v2_atomic(client, CXL_MEMSIM_V2_OP_ATOMIC_FAA, address, 0, addend, old_value, new_value,
                                timeout_ms, errp);
}

bool cxl_memsim_v2_compare_exchange(CxlMemsimV2Client *client, uint64_t address, uint64_t expected, uint64_t desired,
                                    uint64_t *old_value, uint64_t *new_value, int timeout_ms, Error **errp) {
    return cxl_memsim_v2_atomic(client, CXL_MEMSIM_V2_OP_ATOMIC_CAS, address, expected, desired, old_value, new_value,
                                timeout_ms, errp);
}

bool cxl_memsim_v2_fence(CxlMemsimV2Client *client, int timeout_ms, Error **errp) {
    CxlMemsimV2Frame request;
    CxlMemsimV2Frame response;
    bool success;

    if (!client) {
        error_setg(errp, "invalid CXLMemSim v2 fence client");
        return false;
    }
    qemu_mutex_lock(&client->operation_lock);
    for (;;) {
        uint64_t dirty_address = 0;
        bool found = false;
        size_t index;

        qemu_mutex_lock(&client->cache_lock);
        for (index = 0; index < client->cache_line_count; index++) {
            if (client->cache[index].valid && client->cache[index].state == CXL_MEMSIM_V2_STATE_M) {
                dirty_address = client->cache[index].address;
                found = true;
                break;
            }
        }
        qemu_mutex_unlock(&client->cache_lock);
        if (!found) {
            break;
        }
        if (!cxl_memsim_v2_cache_evict_address(client, dirty_address, timeout_ms, errp)) {
            qemu_mutex_unlock(&client->operation_lock);
            return false;
        }
    }
    cxl_memsim_v2_frame_init(&request, CXL_MEMSIM_V2_OP_FENCE);
    success = cxl_memsim_v2_client_transact(client, &request, &response, timeout_ms, errp) &&
              cxl_memsim_v2_response_ok(&response, errp) && response.state == CXL_MEMSIM_V2_STATE_I &&
              response.epoch == 0 && response.payload_len == 0;
    if (!success && errp && !*errp) {
        error_setg(errp, "invalid CXLMemSim v2 fence response");
    }
    qemu_mutex_unlock(&client->operation_lock);
    return success;
}

void cxl_memsim_v2_client_free(CxlMemsimV2Client *client) {
    if (!client) {
        return;
    }
    if (client->started) {
        CxlMemsimV2Frame request;
        CxlMemsimV2Frame response;
        Error *local_err = NULL;
        bool connected;
        bool fence_ok = false;
        bool progress_joinable;

        qemu_mutex_lock(&client->state_lock);
        connected = client->connected && client->session_id;
        qemu_mutex_unlock(&client->state_lock);
        if (connected) {
            fence_ok = cxl_memsim_v2_fence(client, client->timeout_ms, &local_err);
            error_free(local_err);
            local_err = NULL;

            qemu_mutex_lock(&client->state_lock);
            connected = fence_ok && client->connected;
            qemu_mutex_unlock(&client->state_lock);
            if (connected) {
                cxl_memsim_v2_frame_init(&request, CXL_MEMSIM_V2_OP_UNREGISTER);
                if (!cxl_memsim_v2_transact_internal(client, &request, &response, client->timeout_ms, false,
                                                     &local_err) ||
                    response.status != CXL_MEMSIM_V2_STATUS_OK || response.state != CXL_MEMSIM_V2_STATE_I ||
                    response.epoch || response.payload_len) {
                    error_free(local_err);
                }
            }
        }
        qio_channel_shutdown(QIO_CHANNEL(client->socket), QIO_CHANNEL_SHUTDOWN_BOTH, NULL);
        qemu_mutex_lock(&client->state_lock);
        progress_joinable = client->progress_joinable;
        qemu_mutex_unlock(&client->state_lock);
        if (progress_joinable) {
            qemu_thread_join(&client->progress_thread);
            qemu_mutex_lock(&client->state_lock);
            client->progress_joinable = false;
            qemu_mutex_unlock(&client->state_lock);
        }
        qio_channel_close(QIO_CHANNEL(client->socket), NULL);
        object_unref(OBJECT(client->socket));
    }
    g_hash_table_destroy(client->pending);
    g_hash_table_destroy(client->completed_retries);
    g_free(client->cache);
    g_free(client->connection_error);
    qemu_mutex_destroy(&client->cache_lock);
    qemu_mutex_destroy(&client->operation_lock);
    qemu_mutex_destroy(&client->send_lock);
    qemu_mutex_destroy(&client->state_lock);
    g_free(client);
}

uint16_t cxl_memsim_v2_client_endpoint(CxlMemsimV2Client *client) {
    return client ? client->endpoint : CXL_MEMSIM_V2_SERVER_ENDPOINT;
}

uint64_t cxl_memsim_v2_client_session(CxlMemsimV2Client *client) {
    uint64_t session = 0;

    if (client) {
        qemu_mutex_lock(&client->state_lock);
        session = client->session_id;
        qemu_mutex_unlock(&client->state_lock);
    }
    return session;
}

unsigned cxl_memsim_v2_client_progress_starts(CxlMemsimV2Client *client) {
    unsigned starts = 0;

    if (client) {
        qemu_mutex_lock(&client->state_lock);
        starts = client->progress_starts;
        qemu_mutex_unlock(&client->state_lock);
    }
    return starts;
}

CxlMemsimV2Client *cxl_memsim_v2_path_client(CxlMemsimV2EndpointPair *endpoints, CxlMemsimV2Path path) {
    if (!endpoints) {
        return NULL;
    }
    switch (path) {
    case CXL_MEMSIM_V2_PATH_CFMWS_HOST:
        return endpoints->host;
    case CXL_MEMSIM_V2_PATH_BAR2_DEVICE:
        return endpoints->device;
    default:
        return NULL;
    }
}
