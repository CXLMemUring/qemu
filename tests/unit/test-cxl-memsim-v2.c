#include "qemu/osdep.h"

#include "hw/cxl/cxl_memsim_v2.h"
#include "hw/cxl/cxl_type2_gpu_cmd.h"
#include "qapi/error.h"
#include "qemu/module.h"

#include <poll.h>

#define TEST_TIMEOUT_MS 2000
#define TEST_RESPONSE_ACK_INTERVAL 128
#define TEST_LINE_A UINT64_C(0x1000)
#define TEST_LINE_B UINT64_C(0x2000)

typedef struct FakePeer {
    int fd;
    uint64_t session_id;
    uint16_t registered_endpoint;
    uint16_t request_endpoint;
    uint64_t request_session;
    bool duplex;
    bool saw_snoop_ack;
    int error_code;
} FakePeer;

typedef struct SnoopObservation {
    GThread *request_thread;
    GThread *handler_thread;
    unsigned calls;
} SnoopObservation;

typedef struct OperationPeer {
    int fd;
    uint64_t session_id;
    uint64_t stored_value;
    int error_code;
} OperationPeer;

typedef struct WatermarkPeer {
    int fd;
    uint64_t session_id;
    uint64_t acknowledged_request;
    int error_code;
} WatermarkPeer;

typedef struct RetryPeer {
    int fd;
    uint64_t session_id;
    uint64_t first_request_id;
    uint64_t second_request_id;
    bool frames_identical;
    int error_code;
} RetryPeer;

typedef struct HeartbeatFailurePeer {
    int fd;
    uint64_t session_id;
    unsigned ordinary_responses;
    bool saw_heartbeat;
    int error_code;
} HeartbeatFailurePeer;

typedef struct RegistrationFailurePeer {
    int fd;
    int error_code;
} RegistrationFailurePeer;

static void put_le16(uint8_t *bytes, size_t offset, uint16_t value) {
    bytes[offset] = value;
    bytes[offset + 1] = value >> 8;
}

static void put_le32(uint8_t *bytes, size_t offset, uint32_t value) {
    size_t i;

    for (i = 0; i < sizeof(value); i++) {
        bytes[offset + i] = value >> (i * 8);
    }
}

static void put_le64(uint8_t *bytes, size_t offset, uint64_t value) {
    size_t i;

    for (i = 0; i < sizeof(value); i++) {
        bytes[offset + i] = value >> (i * 8);
    }
}

static uint16_t get_le16(const uint8_t *bytes, size_t offset) {
    return bytes[offset] | (uint16_t)bytes[offset + 1] << 8;
}

static uint32_t get_le32(const uint8_t *bytes, size_t offset) {
    uint32_t value = 0;
    size_t i;

    for (i = 0; i < sizeof(value); i++) {
        value |= (uint32_t)bytes[offset + i] << (i * 8);
    }
    return value;
}

static uint64_t get_le64(const uint8_t *bytes, size_t offset) {
    uint64_t value = 0;
    size_t i;

    for (i = 0; i < sizeof(value); i++) {
        value |= (uint64_t)bytes[offset + i] << (i * 8);
    }
    return value;
}

static bool read_all_timeout(int fd, uint8_t *bytes, size_t length) {
    size_t offset = 0;

    while (offset < length) {
        struct pollfd pfd = {
            .fd = fd,
            .events = POLLIN,
        };
        ssize_t received;

        if (poll(&pfd, 1, TEST_TIMEOUT_MS) != 1) {
            return false;
        }
        do {
            received = recv(fd, bytes + offset, length - offset, 0);
        } while (received < 0 && errno == EINTR);
        if (received <= 0) {
            return false;
        }
        offset += received;
    }
    return true;
}

static bool write_all(int fd, const uint8_t *bytes, size_t length) {
    size_t offset = 0;

    while (offset < length) {
        ssize_t sent;

        do {
            sent = send(fd, bytes + offset, length - offset, MSG_NOSIGNAL);
        } while (sent < 0 && errno == EINTR);
        if (sent <= 0) {
            return false;
        }
        offset += sent;
    }
    return true;
}

static void init_wire_frame(uint8_t frame[CXL_MEMSIM_V2_FRAME_SIZE], uint16_t opcode) {
    memset(frame, 0, CXL_MEMSIM_V2_FRAME_SIZE);
    put_le32(frame, 0, CXL_MEMSIM_V2_MAGIC);
    put_le16(frame, 4, CXL_MEMSIM_V2_VERSION);
    put_le16(frame, 6, opcode);
}

static void send_registration_response(FakePeer *peer, const uint8_t *request) {
    uint8_t response[CXL_MEMSIM_V2_FRAME_SIZE];

    init_wire_frame(response, CXL_MEMSIM_V2_OP_RESPONSE);
    response[14] = CXL_MEMSIM_V2_ACK_MODEL;
    put_le16(response, 16, CXL_MEMSIM_V2_SERVER_ENDPOINT);
    put_le16(response, 18, get_le16(request, 16));
    put_le64(response, 40, peer->session_id);
    put_le64(response, 64, CXL_MEMSIM_V2_CAP_MODEL_SNOOP);
    put_le64(response, 72, get_le64(request, 72));
    put_le64(response, 80, get_le64(request, 80));
    put_le64(response, 88, 1);
    put_le32(response, 96, CXL_MEMSIM_V2_LINE_SIZE);
    if (!write_all(peer->fd, response, sizeof(response))) {
        peer->error_code = EIO;
    }
}

static void send_gets_response(FakePeer *peer, const uint8_t *request, uint8_t fill) {
    uint8_t response[CXL_MEMSIM_V2_FRAME_SIZE];

    init_wire_frame(response, CXL_MEMSIM_V2_OP_RESPONSE);
    response[15] = CXL_MEMSIM_V2_STATE_E;
    put_le16(response, 16, CXL_MEMSIM_V2_SERVER_ENDPOINT);
    put_le16(response, 18, get_le16(request, 16));
    put_le16(response, 20, CXL_MEMSIM_V2_LINE_SIZE);
    put_le64(response, 24, get_le64(request, 24));
    put_le64(response, 40, get_le64(request, 40));
    put_le64(response, 48, get_le64(request, 48));
    put_le64(response, 56, 1);
    memset(response + 104, fill, CXL_MEMSIM_V2_LINE_SIZE);
    if (!write_all(peer->fd, response, sizeof(response))) {
        peer->error_code = EIO;
    }
}

static bool send_operation_response(int fd, const uint8_t *request, uint8_t state, uint64_t epoch, const uint8_t *line,
                                    uint64_t old_value) {
    uint8_t response[CXL_MEMSIM_V2_FRAME_SIZE];

    init_wire_frame(response, CXL_MEMSIM_V2_OP_RESPONSE);
    response[15] = state;
    put_le16(response, 16, CXL_MEMSIM_V2_SERVER_ENDPOINT);
    put_le16(response, 18, get_le16(request, 16));
    put_le64(response, 24, get_le64(request, 24));
    put_le64(response, 40, get_le64(request, 40));
    put_le64(response, 48, get_le64(request, 48));
    put_le64(response, 56, epoch);
    put_le64(response, 88, old_value);
    if (line) {
        put_le16(response, 20, CXL_MEMSIM_V2_LINE_SIZE);
        memcpy(response + 104, line, CXL_MEMSIM_V2_LINE_SIZE);
    }
    return write_all(fd, response, sizeof(response));
}

static void send_snoop(FakePeer *peer) {
    uint8_t snoop[CXL_MEMSIM_V2_FRAME_SIZE];

    init_wire_frame(snoop, CXL_MEMSIM_V2_OP_SNP_INV);
    put_le16(snoop, 16, CXL_MEMSIM_V2_SERVER_ENDPOINT);
    put_le16(snoop, 18, peer->registered_endpoint);
    put_le64(snoop, 32, 77);
    put_le64(snoop, 40, peer->session_id);
    put_le64(snoop, 48, TEST_LINE_A);
    put_le64(snoop, 56, 1);
    if (!write_all(peer->fd, snoop, sizeof(snoop))) {
        peer->error_code = EIO;
    }
}

static gpointer fake_peer_thread(gpointer opaque) {
    FakePeer *peer = opaque;
    uint8_t frame[CXL_MEMSIM_V2_FRAME_SIZE];
    uint8_t second_request[CXL_MEMSIM_V2_FRAME_SIZE];

    if (!read_all_timeout(peer->fd, frame, sizeof(frame)) || get_le32(frame, 0) != CXL_MEMSIM_V2_MAGIC ||
        get_le16(frame, 6) != CXL_MEMSIM_V2_OP_REGISTER) {
        peer->error_code = EPROTO;
        goto out;
    }
    peer->registered_endpoint = get_le16(frame, 16);
    send_registration_response(peer, frame);
    if (peer->error_code) {
        goto out;
    }

    if (!read_all_timeout(peer->fd, frame, sizeof(frame)) || get_le16(frame, 6) != CXL_MEMSIM_V2_OP_GETS) {
        peer->error_code = EPROTO;
        goto out;
    }
    peer->request_endpoint = get_le16(frame, 16);
    peer->request_session = get_le64(frame, 40);

    if (!peer->duplex) {
        send_gets_response(peer, frame, (uint8_t)(0x40 + peer->registered_endpoint));
        goto wait_for_close;
    }

    send_gets_response(peer, frame, 0x51);
    if (peer->error_code || !read_all_timeout(peer->fd, frame, sizeof(frame)) ||
        get_le16(frame, 6) != CXL_MEMSIM_V2_OP_GETS) {
        peer->error_code = EPROTO;
        goto out;
    }
    memcpy(second_request, frame, sizeof(second_request));

    send_snoop(peer);
    if (peer->error_code || !read_all_timeout(peer->fd, frame, sizeof(frame)) ||
        get_le16(frame, 6) != CXL_MEMSIM_V2_OP_SNOOP_ACK || get_le64(frame, 32) != 77 ||
        get_le16(frame, 16) != peer->registered_endpoint || get_le64(frame, 40) != peer->session_id) {
        peer->error_code = EPROTO;
        goto out;
    }
    peer->saw_snoop_ack = true;
    send_gets_response(peer, second_request, 0x62);

wait_for_close:
    while (read_all_timeout(peer->fd, frame, sizeof(frame))) {
        /* Client shutdown is the normal terminator for this fake server. */
    }

out:
    close(peer->fd);
    return NULL;
}

static bool session_frame_expect(int fd, uint64_t session_id, uint16_t opcode,
                                 uint8_t frame[CXL_MEMSIM_V2_FRAME_SIZE]) {
    return read_all_timeout(fd, frame, CXL_MEMSIM_V2_FRAME_SIZE) && get_le16(frame, 6) == opcode &&
           get_le64(frame, 40) == session_id;
}

static bool operation_peer_expect(OperationPeer *peer, uint16_t opcode, uint8_t frame[CXL_MEMSIM_V2_FRAME_SIZE]) {
    if (!session_frame_expect(peer->fd, peer->session_id, opcode, frame)) {
        peer->error_code = EPROTO;
        return false;
    }
    return true;
}

static gpointer operation_peer_thread(gpointer opaque) {
    OperationPeer *peer = opaque;
    FakePeer registration_peer = {
        .fd = peer->fd,
        .session_id = peer->session_id,
    };
    uint8_t frame[CXL_MEMSIM_V2_FRAME_SIZE];
    uint8_t line[CXL_MEMSIM_V2_LINE_SIZE] = {0};

    if (!read_all_timeout(peer->fd, frame, sizeof(frame)) || get_le16(frame, 6) != CXL_MEMSIM_V2_OP_REGISTER) {
        peer->error_code = EPROTO;
        goto out;
    }
    send_registration_response(&registration_peer, frame);
    if (registration_peer.error_code) {
        peer->error_code = registration_peer.error_code;
        goto out;
    }

    put_le64(line, 0, UINT64_C(0x1122334455667788));
    if (!operation_peer_expect(peer, CXL_MEMSIM_V2_OP_GETS, frame) || get_le64(frame, 48) != TEST_LINE_A ||
        !send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_E, 1, line, 0)) {
        peer->error_code = EPROTO;
        goto out;
    }

    memset(line, 0, sizeof(line));
    if (!operation_peer_expect(peer, CXL_MEMSIM_V2_OP_GETM, frame) || get_le64(frame, 48) != TEST_LINE_B ||
        !send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_M, 1, line, 0)) {
        peer->error_code = EPROTO;
        goto out;
    }

    memset(line, 0, sizeof(line));
    put_le64(line, 16, 15);
    if (!operation_peer_expect(peer, CXL_MEMSIM_V2_OP_PUTS, frame) || get_le64(frame, 48) != TEST_LINE_A ||
        !send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_I, 2, NULL, 0) ||
        !operation_peer_expect(peer, CXL_MEMSIM_V2_OP_ATOMIC_FAA, frame) || get_le64(frame, 48) != TEST_LINE_A + 16 ||
        get_le64(frame, 80) != 5 || !send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_M, 3, line, 10) ||
        !operation_peer_expect(peer, CXL_MEMSIM_V2_OP_PUTM, frame) ||
        !send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_I, 4, NULL, 0)) {
        peer->error_code = EPROTO;
        goto out;
    }

    memset(line, 0, sizeof(line));
    put_le64(line, 16, 42);
    if (!operation_peer_expect(peer, CXL_MEMSIM_V2_OP_ATOMIC_CAS, frame) || get_le64(frame, 48) != TEST_LINE_A + 16 ||
        get_le64(frame, 72) != 15 || get_le64(frame, 80) != 42 ||
        !send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_M, 5, line, 15) ||
        !operation_peer_expect(peer, CXL_MEMSIM_V2_OP_PUTM, frame) ||
        !send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_I, 6, NULL, 0) ||
        !operation_peer_expect(peer, CXL_MEMSIM_V2_OP_PUTM, frame)) {
        peer->error_code = EPROTO;
        goto out;
    }
    peer->stored_value = get_le64(frame, 104 + 8);
    if (!send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_I, 2, NULL, 0) ||
        !operation_peer_expect(peer, CXL_MEMSIM_V2_OP_FENCE, frame) ||
        !send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_I, 0, NULL, 0)) {
        peer->error_code = EPROTO;
        goto out;
    }

    while (read_all_timeout(peer->fd, frame, sizeof(frame))) {
        /* Client shutdown is the normal terminator for this fake server. */
    }

out:
    close(peer->fd);
    return NULL;
}

static gpointer watermark_peer_thread(gpointer opaque) {
    WatermarkPeer *peer = opaque;
    FakePeer registration_peer = {
        .fd = peer->fd,
        .session_id = peer->session_id,
    };
    uint8_t frame[CXL_MEMSIM_V2_FRAME_SIZE];
    unsigned index;

    if (!read_all_timeout(peer->fd, frame, sizeof(frame)) || get_le16(frame, 6) != CXL_MEMSIM_V2_OP_REGISTER) {
        peer->error_code = EPROTO;
        goto out;
    }
    send_registration_response(&registration_peer, frame);
    if (registration_peer.error_code) {
        peer->error_code = registration_peer.error_code;
        goto out;
    }

    for (index = 1; index <= TEST_RESPONSE_ACK_INTERVAL; index++) {
        if (!session_frame_expect(peer->fd, peer->session_id, CXL_MEMSIM_V2_OP_FENCE, frame) ||
            get_le64(frame, 24) != index ||
            !send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_I, 0, NULL, 0)) {
            peer->error_code = EPROTO;
            goto out;
        }
    }

    if (!session_frame_expect(peer->fd, peer->session_id, CXL_MEMSIM_V2_OP_HEARTBEAT, frame) ||
        get_le64(frame, 24) != TEST_RESPONSE_ACK_INTERVAL + 1 || get_le64(frame, 88) != TEST_RESPONSE_ACK_INTERVAL) {
        peer->error_code = EPROTO;
        goto out;
    }
    peer->acknowledged_request = get_le64(frame, 88);
    if (!send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_I, 0, NULL, 0)) {
        peer->error_code = EIO;
        goto out;
    }

    while (read_all_timeout(peer->fd, frame, sizeof(frame))) {
        /* Client shutdown is the normal terminator for this fake server. */
    }

out:
    close(peer->fd);
    return NULL;
}

static gpointer retry_peer_thread(gpointer opaque) {
    RetryPeer *peer = opaque;
    FakePeer registration_peer = {
        .fd = peer->fd,
        .session_id = peer->session_id,
    };
    uint8_t first[CXL_MEMSIM_V2_FRAME_SIZE];
    uint8_t second[CXL_MEMSIM_V2_FRAME_SIZE];
    uint8_t line[CXL_MEMSIM_V2_LINE_SIZE] = {0};

    if (!read_all_timeout(peer->fd, first, sizeof(first)) || get_le16(first, 6) != CXL_MEMSIM_V2_OP_REGISTER) {
        peer->error_code = EPROTO;
        goto out;
    }
    send_registration_response(&registration_peer, first);
    if (registration_peer.error_code ||
        !session_frame_expect(peer->fd, peer->session_id, CXL_MEMSIM_V2_OP_ATOMIC_FAA, first) ||
        !read_all_timeout(peer->fd, second, sizeof(second))) {
        peer->error_code = EPROTO;
        goto out;
    }

    peer->first_request_id = get_le64(first, 24);
    peer->second_request_id = get_le64(second, 24);
    peer->frames_identical = memcmp(first, second, sizeof(first)) == 0;
    put_le64(line, 0, 15);
    if (get_le16(second, 6) != CXL_MEMSIM_V2_OP_ATOMIC_FAA ||
        !send_operation_response(peer->fd, second, CXL_MEMSIM_V2_STATE_M, 1, line, 10) ||
        !session_frame_expect(peer->fd, peer->session_id, CXL_MEMSIM_V2_OP_PUTM, first) ||
        !send_operation_response(peer->fd, first, CXL_MEMSIM_V2_STATE_I, 2, NULL, 0) ||
        !send_operation_response(peer->fd, second, CXL_MEMSIM_V2_STATE_M, 1, line, 10)) {
        peer->error_code = EPROTO;
        goto out;
    }

    while (read_all_timeout(peer->fd, first, sizeof(first))) {
        uint16_t opcode = get_le16(first, 6);

        if (opcode != CXL_MEMSIM_V2_OP_FENCE && opcode != CXL_MEMSIM_V2_OP_UNREGISTER) {
            peer->error_code = EPROTO;
            break;
        }
        if (!send_operation_response(peer->fd, first, CXL_MEMSIM_V2_STATE_I, 0, NULL, 0) ||
            opcode == CXL_MEMSIM_V2_OP_UNREGISTER) {
            break;
        }
    }

out:
    close(peer->fd);
    return NULL;
}

static gpointer heartbeat_store_failure_peer_thread(gpointer opaque) {
    HeartbeatFailurePeer *peer = opaque;
    FakePeer registration_peer = {
        .fd = peer->fd,
        .session_id = peer->session_id,
    };
    uint8_t frame[CXL_MEMSIM_V2_FRAME_SIZE];
    uint8_t response[CXL_MEMSIM_V2_FRAME_SIZE];
    uint8_t line[CXL_MEMSIM_V2_LINE_SIZE] = {0};
    unsigned index;

    if (!read_all_timeout(peer->fd, frame, sizeof(frame)) || get_le16(frame, 6) != CXL_MEMSIM_V2_OP_REGISTER) {
        peer->error_code = EPROTO;
        goto out;
    }
    send_registration_response(&registration_peer, frame);
    if (registration_peer.error_code) {
        peer->error_code = registration_peer.error_code;
        goto out;
    }
    for (index = 1; index < TEST_RESPONSE_ACK_INTERVAL; index++) {
        if (!session_frame_expect(peer->fd, peer->session_id, CXL_MEMSIM_V2_OP_FENCE, frame) ||
            !send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_I, 0, NULL, 0)) {
            peer->error_code = EPROTO;
            goto out;
        }
        peer->ordinary_responses++;
    }
    if (!session_frame_expect(peer->fd, peer->session_id, CXL_MEMSIM_V2_OP_GETM, frame) ||
        !send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_M, 1, line, 0) ||
        !session_frame_expect(peer->fd, peer->session_id, CXL_MEMSIM_V2_OP_HEARTBEAT, frame) ||
        get_le64(frame, 88) != TEST_RESPONSE_ACK_INTERVAL) {
        peer->error_code = EPROTO;
        goto out;
    }
    peer->ordinary_responses++;
    peer->saw_heartbeat = true;
    init_wire_frame(response, CXL_MEMSIM_V2_OP_RESPONSE);
    put_le16(response, 12, CXL_MEMSIM_V2_STATUS_IO_ERROR);
    put_le16(response, 16, CXL_MEMSIM_V2_SERVER_ENDPOINT);
    put_le16(response, 18, get_le16(frame, 16));
    put_le64(response, 24, get_le64(frame, 24));
    put_le64(response, 40, get_le64(frame, 40));
    if (!write_all(peer->fd, response, sizeof(response))) {
        peer->error_code = EIO;
        goto out;
    }
    while (read_all_timeout(peer->fd, frame, sizeof(frame))) {
        peer->error_code = EPROTO;
        break;
    }

out:
    close(peer->fd);
    return NULL;
}

static gpointer heartbeat_atomic_failure_peer_thread(gpointer opaque) {
    HeartbeatFailurePeer *peer = opaque;
    FakePeer registration_peer = {
        .fd = peer->fd,
        .session_id = peer->session_id,
    };
    uint8_t frame[CXL_MEMSIM_V2_FRAME_SIZE];
    uint8_t response[CXL_MEMSIM_V2_FRAME_SIZE];
    uint8_t line[CXL_MEMSIM_V2_LINE_SIZE] = {0};
    unsigned index;

    if (!read_all_timeout(peer->fd, frame, sizeof(frame)) || get_le16(frame, 6) != CXL_MEMSIM_V2_OP_REGISTER) {
        peer->error_code = EPROTO;
        goto out;
    }
    send_registration_response(&registration_peer, frame);
    if (registration_peer.error_code) {
        peer->error_code = registration_peer.error_code;
        goto out;
    }
    for (index = 1; index < TEST_RESPONSE_ACK_INTERVAL; index++) {
        if (!session_frame_expect(peer->fd, peer->session_id, CXL_MEMSIM_V2_OP_FENCE, frame) ||
            !send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_I, 0, NULL, 0)) {
            peer->error_code = EPROTO;
            goto out;
        }
        peer->ordinary_responses++;
    }
    put_le64(line, 0, 15);
    if (!session_frame_expect(peer->fd, peer->session_id, CXL_MEMSIM_V2_OP_ATOMIC_FAA, frame) ||
        !send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_M, 1, line, 10) ||
        !session_frame_expect(peer->fd, peer->session_id, CXL_MEMSIM_V2_OP_HEARTBEAT, frame) ||
        get_le64(frame, 88) != TEST_RESPONSE_ACK_INTERVAL) {
        peer->error_code = EPROTO;
        goto out;
    }
    peer->ordinary_responses++;
    peer->saw_heartbeat = true;
    init_wire_frame(response, CXL_MEMSIM_V2_OP_RESPONSE);
    put_le16(response, 12, CXL_MEMSIM_V2_STATUS_IO_ERROR);
    put_le16(response, 16, CXL_MEMSIM_V2_SERVER_ENDPOINT);
    put_le16(response, 18, get_le16(frame, 16));
    put_le64(response, 24, get_le64(frame, 24));
    put_le64(response, 40, get_le64(frame, 40));
    if (!write_all(peer->fd, response, sizeof(response))) {
        peer->error_code = EIO;
        goto out;
    }
    while (read_all_timeout(peer->fd, frame, sizeof(frame))) {
        peer->error_code = EPROTO;
        break;
    }

out:
    close(peer->fd);
    return NULL;
}

static gpointer cached_load_disconnect_peer_thread(gpointer opaque) {
    HeartbeatFailurePeer *peer = opaque;
    FakePeer registration_peer = {
        .fd = peer->fd,
        .session_id = peer->session_id,
    };
    uint8_t frame[CXL_MEMSIM_V2_FRAME_SIZE];
    uint8_t line[CXL_MEMSIM_V2_LINE_SIZE] = {0};

    if (!read_all_timeout(peer->fd, frame, sizeof(frame)) || get_le16(frame, 6) != CXL_MEMSIM_V2_OP_REGISTER) {
        peer->error_code = EPROTO;
        goto out;
    }
    send_registration_response(&registration_peer, frame);
    put_le64(line, 0, UINT64_C(0x1122334455667788));
    if (registration_peer.error_code ||
        !session_frame_expect(peer->fd, peer->session_id, CXL_MEMSIM_V2_OP_GETS, frame) ||
        !send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_S, 1, line, 0) ||
        !session_frame_expect(peer->fd, peer->session_id, CXL_MEMSIM_V2_OP_FENCE, frame)) {
        peer->error_code = EPROTO;
    }

out:
    close(peer->fd);
    return NULL;
}

static gpointer registration_failure_peer_thread(gpointer opaque) {
    RegistrationFailurePeer *peer = opaque;
    uint8_t request[CXL_MEMSIM_V2_FRAME_SIZE];
    uint8_t response[CXL_MEMSIM_V2_FRAME_SIZE];

    if (!read_all_timeout(peer->fd, request, sizeof(request)) || get_le16(request, 6) != CXL_MEMSIM_V2_OP_REGISTER) {
        peer->error_code = EPROTO;
        goto out;
    }
    init_wire_frame(response, CXL_MEMSIM_V2_OP_RESPONSE);
    put_le16(response, 12, CXL_MEMSIM_V2_STATUS_NO_CAPABILITY);
    put_le16(response, 16, CXL_MEMSIM_V2_SERVER_ENDPOINT);
    put_le16(response, 18, get_le16(request, 16));
    if (!write_all(peer->fd, response, sizeof(response))) {
        peer->error_code = EIO;
    }

out:
    while (read_all_timeout(peer->fd, request, sizeof(request))) {
        peer->error_code = EPROTO;
        break;
    }
    close(peer->fd);
    return NULL;
}

static gpointer heartbeat_failure_peer_thread(gpointer opaque) {
    HeartbeatFailurePeer *peer = opaque;
    FakePeer registration_peer = {
        .fd = peer->fd,
        .session_id = peer->session_id,
    };
    uint8_t frame[CXL_MEMSIM_V2_FRAME_SIZE];
    uint8_t response[CXL_MEMSIM_V2_FRAME_SIZE];
    unsigned index;

    if (!read_all_timeout(peer->fd, frame, sizeof(frame)) || get_le16(frame, 6) != CXL_MEMSIM_V2_OP_REGISTER) {
        peer->error_code = EPROTO;
        goto out;
    }
    send_registration_response(&registration_peer, frame);
    if (registration_peer.error_code) {
        peer->error_code = registration_peer.error_code;
        goto out;
    }

    for (index = 0; index < TEST_RESPONSE_ACK_INTERVAL; index++) {
        if (!session_frame_expect(peer->fd, peer->session_id, CXL_MEMSIM_V2_OP_FENCE, frame) ||
            !send_operation_response(peer->fd, frame, CXL_MEMSIM_V2_STATE_I, 0, NULL, 0)) {
            peer->error_code = EPROTO;
            goto out;
        }
        peer->ordinary_responses++;
    }
    if (!session_frame_expect(peer->fd, peer->session_id, CXL_MEMSIM_V2_OP_HEARTBEAT, frame)) {
        peer->error_code = EPROTO;
        goto out;
    }
    peer->saw_heartbeat = true;
    init_wire_frame(response, CXL_MEMSIM_V2_OP_RESPONSE);
    put_le16(response, 12, CXL_MEMSIM_V2_STATUS_IO_ERROR);
    put_le16(response, 16, CXL_MEMSIM_V2_SERVER_ENDPOINT);
    put_le16(response, 18, get_le16(frame, 16));
    put_le64(response, 24, get_le64(frame, 24));
    put_le64(response, 40, get_le64(frame, 40));
    if (!write_all(peer->fd, response, sizeof(response))) {
        peer->error_code = EIO;
        goto out;
    }
    while (read_all_timeout(peer->fd, frame, sizeof(frame))) {
        peer->error_code = EPROTO;
        break;
    }

out:
    close(peer->fd);
    return NULL;
}

static bool snoop_handler(void *opaque, const CxlMemsimV2Frame *snoop, CxlMemsimV2Frame *ack, Error **errp) {
    SnoopObservation *observation = opaque;

    g_assert_cmpuint(snoop->type, ==, CXL_MEMSIM_V2_OP_SNP_INV);
    g_assert_cmpuint(snoop->snoop_id, ==, 77);
    observation->calls++;
    observation->handler_thread = g_thread_self();
    ack->status = CXL_MEMSIM_V2_STATUS_OK;
    ack->state = CXL_MEMSIM_V2_STATE_I;
    return true;
}

static CxlMemsimV2Client *start_client(int fd, uint16_t endpoint, CxlMemsimV2SnoopHandler handler,
                                       void *handler_opaque) {
    CxlMemsimV2Client *client;
    Error *err = NULL;
    bool started;

    client = cxl_memsim_v2_client_new(endpoint, handler, handler_opaque);
    g_assert_nonnull(client);
    started = cxl_memsim_v2_client_start_fd(client, fd, 256 * 1024, 4, TEST_TIMEOUT_MS, &err);
    if (!started && err) {
        g_test_message("registration failed: %s", error_get_pretty(err));
    }
    g_assert_true(started);
    g_assert_null(err);
    return client;
}

static void issue_gets(CxlMemsimV2Client *client, uint64_t address, uint8_t expected_fill) {
    CxlMemsimV2Frame request;
    CxlMemsimV2Frame response;
    Error *err = NULL;

    cxl_memsim_v2_frame_init(&request, CXL_MEMSIM_V2_OP_GETS);
    request.addr = address;
    request.state = CXL_MEMSIM_V2_STATE_I;
    g_assert_true(cxl_memsim_v2_client_transact(client, &request, &response, TEST_TIMEOUT_MS, &err));
    g_assert_null(err);
    g_assert_cmpuint(response.status, ==, CXL_MEMSIM_V2_STATUS_OK);
    g_assert_cmpuint(response.payload_len, ==, CXL_MEMSIM_V2_LINE_SIZE);
    g_assert_cmpuint(response.data[0], ==, expected_fill);
}

static void test_distinct_host_and_device_sessions(void) {
    int host_sockets[2];
    int device_sockets[2];
    FakePeer host_peer = {
        .session_id = UINT64_C(0x1001),
    };
    FakePeer device_peer = {
        .session_id = UINT64_C(0x2001),
    };
    GThread *host_thread;
    GThread *device_thread;
    CxlMemsimV2Client *host;
    CxlMemsimV2Client *device;
    CxlMemsimV2EndpointPair endpoints;

    g_assert_cmpint(socketpair(AF_UNIX, SOCK_STREAM, 0, host_sockets), ==, 0);
    g_assert_cmpint(socketpair(AF_UNIX, SOCK_STREAM, 0, device_sockets), ==, 0);
    host_peer.fd = host_sockets[1];
    device_peer.fd = device_sockets[1];
    host_thread = g_thread_new("memsim-v2-host-peer", fake_peer_thread, &host_peer);
    device_thread = g_thread_new("memsim-v2-device-peer", fake_peer_thread, &device_peer);

    host = start_client(host_sockets[0], CXL_MEMSIM_V2_HOST_ENDPOINT, NULL, NULL);
    device = start_client(device_sockets[0], CXL_MEMSIM_V2_DEVICE_ENDPOINT, NULL, NULL);
    endpoints = (CxlMemsimV2EndpointPair){
        .host = host,
        .device = device,
    };
    issue_gets(cxl_memsim_v2_path_client(&endpoints, CXL_MEMSIM_V2_PATH_CFMWS_HOST), TEST_LINE_A, 0x40);
    issue_gets(cxl_memsim_v2_path_client(&endpoints, CXL_MEMSIM_V2_PATH_BAR2_DEVICE), TEST_LINE_A, 0x41);

    g_assert_cmpuint(cxl_memsim_v2_client_endpoint(host), ==, CXL_MEMSIM_V2_HOST_ENDPOINT);
    g_assert_cmpuint(cxl_memsim_v2_client_endpoint(device), ==, CXL_MEMSIM_V2_DEVICE_ENDPOINT);
    g_assert_cmpuint(cxl_memsim_v2_client_session(host), ==, host_peer.session_id);
    g_assert_cmpuint(cxl_memsim_v2_client_session(device), ==, device_peer.session_id);
    g_assert_cmpuint(cxl_memsim_v2_client_session(host), !=, cxl_memsim_v2_client_session(device));
    g_assert_cmpuint(cxl_memsim_v2_client_progress_starts(host), ==, 1);
    g_assert_cmpuint(cxl_memsim_v2_client_progress_starts(device), ==, 1);

    cxl_memsim_v2_client_free(host);
    cxl_memsim_v2_client_free(device);
    g_thread_join(host_thread);
    g_thread_join(device_thread);
    g_assert_cmpint(host_peer.error_code, ==, 0);
    g_assert_cmpint(device_peer.error_code, ==, 0);
    g_assert_cmpuint(host_peer.registered_endpoint, ==, CXL_MEMSIM_V2_HOST_ENDPOINT);
    g_assert_cmpuint(device_peer.registered_endpoint, ==, CXL_MEMSIM_V2_DEVICE_ENDPOINT);
    g_assert_cmpuint(host_peer.request_endpoint, ==, CXL_MEMSIM_V2_HOST_ENDPOINT);
    g_assert_cmpuint(device_peer.request_endpoint, ==, CXL_MEMSIM_V2_DEVICE_ENDPOINT);
    g_assert_cmpuint(host_peer.request_session, ==, host_peer.session_id);
    g_assert_cmpuint(device_peer.request_session, ==, device_peer.session_id);
}

static void test_bar2_coherent_command_numbers(void) {
    g_assert_cmpuint(CXL_GPU_CMD_COHERENT_LOAD, ==, 0x83);
    g_assert_cmpuint(CXL_GPU_CMD_COHERENT_STORE, ==, 0x84);
    g_assert_cmpuint(CXL_GPU_CMD_COHERENT_FAA, ==, 0x85);
    g_assert_cmpuint(CXL_GPU_CMD_COHERENT_CAS, ==, 0x86);
    g_assert_cmpuint(CXL_GPU_CMD_COHERENT_FENCE, ==, 0xA3);
    g_assert_cmpuint(CXL_GPU_CMD_COH_ACQUIRE_RANGE, ==, 0xB2);
    g_assert_cmpuint(CXL_GPU_CMD_COH_RELEASE_RANGE, ==, 0xB3);
    g_assert_cmpuint(CXL_GPU_ERROR_COHERENCY, ==, 1000);
}

static void test_endpoint_memory_and_atomic_operations(void) {
    int sockets[2];
    OperationPeer peer = {
        .session_id = UINT64_C(0x4001),
    };
    GThread *peer_thread;
    CxlMemsimV2Client *client;
    Error *err = NULL;
    uint64_t value = 0;
    uint64_t old_value = 0;
    uint64_t new_value = 0;

    g_assert_cmpint(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), ==, 0);
    peer.fd = sockets[1];
    peer_thread = g_thread_new("memsim-v2-operation-peer", operation_peer_thread, &peer);
    client = start_client(sockets[0], CXL_MEMSIM_V2_DEVICE_ENDPOINT, NULL, NULL);

    g_assert_true(cxl_memsim_v2_load(client, TEST_LINE_A, 8, &value, TEST_TIMEOUT_MS, &err));
    g_assert_null(err);
    g_assert_cmphex(value, ==, UINT64_C(0x1122334455667788));

    g_assert_true(cxl_memsim_v2_store(client, TEST_LINE_B + 8, 8, UINT64_C(0xaabbccddeeff0011), TEST_TIMEOUT_MS, &err));
    g_assert_null(err);

    g_assert_true(cxl_memsim_v2_fetch_add(client, TEST_LINE_A + 16, 5, &old_value, &new_value, TEST_TIMEOUT_MS, &err));
    g_assert_null(err);
    g_assert_cmpuint(old_value, ==, 10);
    g_assert_cmpuint(new_value, ==, 15);

    g_assert_true(cxl_memsim_v2_compare_exchange(client, TEST_LINE_A + 16, 15, 42, &old_value, &new_value,
                                                 TEST_TIMEOUT_MS, &err));
    g_assert_null(err);
    g_assert_cmpuint(old_value, ==, 15);
    g_assert_cmpuint(new_value, ==, 42);

    g_assert_true(cxl_memsim_v2_fence(client, TEST_TIMEOUT_MS, &err));
    g_assert_null(err);

    cxl_memsim_v2_client_free(client);
    g_thread_join(peer_thread);
    g_assert_cmpint(peer.error_code, ==, 0);
    g_assert_cmphex(peer.stored_value, ==, UINT64_C(0xaabbccddeeff0011));
}

static void test_progress_reader_multiplexes_snoop_and_response(void) {
    int sockets[2];
    FakePeer peer = {
        .session_id = UINT64_C(0x3001),
        .duplex = true,
    };
    SnoopObservation observation = {
        .request_thread = g_thread_self(),
    };
    GThread *peer_thread;
    CxlMemsimV2Client *client;

    g_assert_cmpint(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), ==, 0);
    peer.fd = sockets[1];
    peer_thread = g_thread_new("memsim-v2-duplex-peer", fake_peer_thread, &peer);
    client = start_client(sockets[0], CXL_MEMSIM_V2_HOST_ENDPOINT, snoop_handler, &observation);

    issue_gets(client, TEST_LINE_A, 0x51);
    issue_gets(client, TEST_LINE_B, 0x62);

    g_assert_cmpuint(observation.calls, ==, 1);
    g_assert_nonnull(observation.handler_thread);
    g_assert_true(observation.handler_thread != observation.request_thread);
    g_assert_cmpuint(cxl_memsim_v2_client_progress_starts(client), ==, 1);

    cxl_memsim_v2_client_free(client);
    g_thread_join(peer_thread);
    g_assert_cmpint(peer.error_code, ==, 0);
    g_assert_true(peer.saw_snoop_ack);
}

static void test_periodic_heartbeat_acknowledges_consumed_responses(void) {
    int sockets[2];
    WatermarkPeer peer = {
        .session_id = UINT64_C(0x5001),
    };
    GThread *peer_thread;
    CxlMemsimV2Client *client;
    Error *err = NULL;
    unsigned index;

    g_assert_cmpint(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), ==, 0);
    peer.fd = sockets[1];
    peer_thread = g_thread_new("memsim-v2-watermark-peer", watermark_peer_thread, &peer);
    client = start_client(sockets[0], CXL_MEMSIM_V2_DEVICE_ENDPOINT, NULL, NULL);

    for (index = 0; index < TEST_RESPONSE_ACK_INTERVAL; index++) {
        g_assert_true(cxl_memsim_v2_fence(client, TEST_TIMEOUT_MS, &err));
        g_assert_null(err);
    }

    cxl_memsim_v2_client_free(client);
    g_thread_join(peer_thread);
    g_assert_cmpint(peer.error_code, ==, 0);
    g_assert_cmpuint(peer.acknowledged_request, ==, TEST_RESPONSE_ACK_INTERVAL);
}

static void test_timeout_retransmits_exact_atomic_frame(void) {
    int sockets[2];
    RetryPeer peer = {
        .session_id = UINT64_C(0x6001),
    };
    GThread *peer_thread;
    CxlMemsimV2Client *client;
    Error *err = NULL;
    uint64_t old_value = 0;
    uint64_t new_value = 0;

    g_assert_cmpint(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), ==, 0);
    peer.fd = sockets[1];
    peer_thread = g_thread_new("memsim-v2-retry-peer", retry_peer_thread, &peer);
    client = start_client(sockets[0], CXL_MEMSIM_V2_DEVICE_ENDPOINT, NULL, NULL);

    g_assert_true(cxl_memsim_v2_fetch_add(client, TEST_LINE_A, 5, &old_value, &new_value, 50, &err));
    g_assert_null(err);
    g_assert_cmpuint(old_value, ==, 10);
    g_assert_cmpuint(new_value, ==, 15);
    g_assert_true(cxl_memsim_v2_fence(client, TEST_TIMEOUT_MS, &err));
    g_assert_null(err);

    cxl_memsim_v2_client_free(client);
    g_thread_join(peer_thread);
    g_assert_cmpint(peer.error_code, ==, 0);
    g_assert_cmpuint(peer.first_request_id, !=, 0);
    g_assert_cmpuint(peer.second_request_id, ==, peer.first_request_id);
    g_assert_true(peer.frames_identical);
}

static void test_heartbeat_failure_after_getm_aborts_store(void) {
    int sockets[2];
    HeartbeatFailurePeer peer = {
        .session_id = UINT64_C(0x7101),
    };
    GThread *peer_thread;
    CxlMemsimV2Client *client;
    Error *err = NULL;
    unsigned index;

    g_assert_cmpint(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), ==, 0);
    peer.fd = sockets[1];
    peer_thread = g_thread_new("memsim-v2-heartbeat-store-failure-peer", heartbeat_store_failure_peer_thread, &peer);
    client = start_client(sockets[0], CXL_MEMSIM_V2_DEVICE_ENDPOINT, NULL, NULL);

    for (index = 1; index < TEST_RESPONSE_ACK_INTERVAL; index++) {
        g_assert_true(cxl_memsim_v2_fence(client, TEST_TIMEOUT_MS, &err));
        g_assert_null(err);
    }
    g_assert_false(cxl_memsim_v2_store(client, TEST_LINE_A, 8, 42, TEST_TIMEOUT_MS, &err));
    g_assert_nonnull(err);
    error_free(err);

    cxl_memsim_v2_client_free(client);
    g_thread_join(peer_thread);
    g_assert_cmpint(peer.error_code, ==, 0);
    g_assert_cmpuint(peer.ordinary_responses, ==, TEST_RESPONSE_ACK_INTERVAL);
    g_assert_true(peer.saw_heartbeat);
}

static void test_heartbeat_failure_preserves_committed_atomic(void) {
    int sockets[2];
    HeartbeatFailurePeer peer = {
        .session_id = UINT64_C(0x7201),
    };
    GThread *peer_thread;
    CxlMemsimV2Client *client;
    Error *err = NULL;
    uint64_t old_value = 0;
    uint64_t new_value = 0;
    unsigned index;

    g_assert_cmpint(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), ==, 0);
    peer.fd = sockets[1];
    peer_thread = g_thread_new("memsim-v2-heartbeat-atomic-failure-peer", heartbeat_atomic_failure_peer_thread, &peer);
    client = start_client(sockets[0], CXL_MEMSIM_V2_DEVICE_ENDPOINT, NULL, NULL);

    for (index = 1; index < TEST_RESPONSE_ACK_INTERVAL; index++) {
        g_assert_true(cxl_memsim_v2_fence(client, TEST_TIMEOUT_MS, &err));
        g_assert_null(err);
    }
    g_assert_true(cxl_memsim_v2_fetch_add(client, TEST_LINE_A, 5, &old_value, &new_value, TEST_TIMEOUT_MS, &err));
    g_assert_null(err);
    g_assert_cmpuint(old_value, ==, 10);
    g_assert_cmpuint(new_value, ==, 15);

    cxl_memsim_v2_client_free(client);
    g_thread_join(peer_thread);
    g_assert_cmpint(peer.error_code, ==, 0);
    g_assert_cmpuint(peer.ordinary_responses, ==, TEST_RESPONSE_ACK_INTERVAL);
    g_assert_true(peer.saw_heartbeat);
}

static void test_disconnected_client_rejects_cached_load(void) {
    int sockets[2];
    HeartbeatFailurePeer peer = {
        .session_id = UINT64_C(0x7301),
    };
    GThread *peer_thread;
    CxlMemsimV2Client *client;
    Error *err = NULL;
    uint64_t value = 0;

    g_assert_cmpint(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), ==, 0);
    peer.fd = sockets[1];
    peer_thread = g_thread_new("memsim-v2-cached-load-disconnect-peer", cached_load_disconnect_peer_thread, &peer);
    client = start_client(sockets[0], CXL_MEMSIM_V2_DEVICE_ENDPOINT, NULL, NULL);

    g_assert_true(cxl_memsim_v2_load(client, TEST_LINE_A, 8, &value, TEST_TIMEOUT_MS, &err));
    g_assert_null(err);
    g_assert_cmphex(value, ==, UINT64_C(0x1122334455667788));
    g_assert_false(cxl_memsim_v2_fence(client, TEST_TIMEOUT_MS, &err));
    g_assert_nonnull(err);
    error_free(err);
    err = NULL;
    g_assert_false(cxl_memsim_v2_load(client, TEST_LINE_A, 8, &value, TEST_TIMEOUT_MS, &err));
    g_assert_nonnull(err);
    error_free(err);

    cxl_memsim_v2_client_free(client);
    g_thread_join(peer_thread);
    g_assert_cmpint(peer.error_code, ==, 0);
}

static void test_registration_failure_can_be_freed(void) {
    int sockets[2];
    RegistrationFailurePeer peer = {0};
    GThread *peer_thread;
    CxlMemsimV2Client *client;
    Error *err = NULL;

    g_assert_cmpint(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), ==, 0);
    peer.fd = sockets[1];
    peer_thread = g_thread_new("memsim-v2-registration-failure-peer", registration_failure_peer_thread, &peer);
    client = cxl_memsim_v2_client_new(CXL_MEMSIM_V2_DEVICE_ENDPOINT, NULL, NULL);
    g_assert_nonnull(client);
    g_assert_false(cxl_memsim_v2_client_start_fd(client, sockets[0], 256 * 1024, 4, TEST_TIMEOUT_MS, &err));
    g_assert_nonnull(err);
    error_free(err);
    cxl_memsim_v2_client_free(client);
    g_thread_join(peer_thread);
    g_assert_cmpint(peer.error_code, ==, 0);
}

static void test_heartbeat_failure_preserves_completed_operation(void) {
    int sockets[2];
    HeartbeatFailurePeer peer = {
        .session_id = UINT64_C(0x7001),
    };
    GThread *peer_thread;
    CxlMemsimV2Client *client;
    Error *err = NULL;
    unsigned index;

    g_assert_cmpint(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), ==, 0);
    peer.fd = sockets[1];
    peer_thread = g_thread_new("memsim-v2-heartbeat-failure-peer", heartbeat_failure_peer_thread, &peer);
    client = start_client(sockets[0], CXL_MEMSIM_V2_DEVICE_ENDPOINT, NULL, NULL);

    for (index = 0; index < TEST_RESPONSE_ACK_INTERVAL; index++) {
        g_assert_true(cxl_memsim_v2_fence(client, TEST_TIMEOUT_MS, &err));
        g_assert_null(err);
    }
    g_assert_false(cxl_memsim_v2_fence(client, TEST_TIMEOUT_MS, &err));
    g_assert_nonnull(err);
    error_free(err);

    cxl_memsim_v2_client_free(client);
    g_thread_join(peer_thread);
    g_assert_cmpint(peer.error_code, ==, 0);
    g_assert_cmpuint(peer.ordinary_responses, ==, TEST_RESPONSE_ACK_INTERVAL);
    g_assert_true(peer.saw_heartbeat);
}

int main(int argc, char **argv) {
    module_call_init(MODULE_INIT_QOM);
    g_test_init(&argc, &argv, NULL);

    g_test_add_func("/cxl/type2/memsim-v2/distinct-endpoint-sessions", test_distinct_host_and_device_sessions);
    g_test_add_func("/cxl/type2/memsim-v2/duplex-progress-reader", test_progress_reader_multiplexes_snoop_and_response);
    g_test_add_func("/cxl/type2/memsim-v2/bar2-command-numbers", test_bar2_coherent_command_numbers);
    g_test_add_func("/cxl/type2/memsim-v2/endpoint-operations", test_endpoint_memory_and_atomic_operations);
    g_test_add_func("/cxl/type2/memsim-v2/periodic-heartbeat-watermark",
                    test_periodic_heartbeat_acknowledges_consumed_responses);
    g_test_add_func("/cxl/type2/memsim-v2/exact-timeout-retry", test_timeout_retransmits_exact_atomic_frame);
    g_test_add_func("/cxl/type2/memsim-v2/heartbeat-failure-after-response",
                    test_heartbeat_failure_preserves_completed_operation);
    g_test_add_func("/cxl/type2/memsim-v2/heartbeat-failure-aborts-store",
                    test_heartbeat_failure_after_getm_aborts_store);
    g_test_add_func("/cxl/type2/memsim-v2/heartbeat-failure-preserves-atomic",
                    test_heartbeat_failure_preserves_committed_atomic);
    g_test_add_func("/cxl/type2/memsim-v2/disconnected-cached-load", test_disconnected_client_rejects_cached_load);
    g_test_add_func("/cxl/type2/memsim-v2/registration-failure-free", test_registration_failure_can_be_freed);

    return g_test_run();
}
