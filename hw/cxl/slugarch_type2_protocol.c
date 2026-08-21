#include "qemu/osdep.h"

#include "hw/cxl/slugarch_type2_protocol.h"

#include "qemu/bswap.h"
#include "qemu/crc32c.h"
#include "qemu/processor.h"
#include "qemu/timer.h"

enum {
    SLUGARCH_T2_VERSION_OFFSET = 4,
    SLUGARCH_T2_TYPE_OFFSET = 6,
    SLUGARCH_T2_LENGTH_OFFSET = 8,
    SLUGARCH_T2_FLAGS_OFFSET = 12,
    SLUGARCH_T2_REQUEST_ID_OFFSET = 16,
    SLUGARCH_T2_CLIENT_ID_OFFSET = 24,
    SLUGARCH_T2_CHECKSUM_OFFSET = 32,
    SLUGARCH_T2_RESERVED_OFFSET = 36,
};

static const uint8_t slugarch_t2_magic[] = { 'S', 'L', 'T', '2' };

static uint32_t slugarch_t2_expected_length(SlugArchT2FrameType type)
{
    switch (type) {
    case SLUGARCH_T2_FRAME_HELLO:
        return 72;
    case SLUGARCH_T2_FRAME_ACK:
        return 88;
    case SLUGARCH_T2_FRAME_READ:
    case SLUGARCH_T2_FRAME_WRITE:
    case SLUGARCH_T2_FRAME_MEMORY_RESPONSE:
        return 128;
    case SLUGARCH_T2_FRAME_COUNTER_SNAPSHOT:
        return 48;
    case SLUGARCH_T2_FRAME_COUNTER_RESPONSE:
        return 112;
    case SLUGARCH_T2_FRAME_ERROR:
        return 56;
    default:
        return 0;
    }
}

static bool slugarch_t2_valid_status(uint32_t status)
{
    return status <= SLUGARCH_T2_STATUS_UNSUPPORTED;
}

static bool slugarch_t2_begin_frame(SlugArchT2FrameType type,
                                    uint64_t request_id,
                                    uint64_t client_id,
                                    SlugArchT2Frame *frame)
{
    uint32_t length = slugarch_t2_expected_length(type);

    if (!frame || !length || !request_id ||
        (type == SLUGARCH_T2_FRAME_HELLO ?
         client_id != 0 : client_id == 0)) {
        return false;
    }

    memset(frame, 0, sizeof(*frame));
    frame->length = length;
    memcpy(frame->bytes, slugarch_t2_magic, sizeof(slugarch_t2_magic));
    stw_le_p(frame->bytes + SLUGARCH_T2_VERSION_OFFSET,
             SLUGARCH_T2_VERSION);
    stw_le_p(frame->bytes + SLUGARCH_T2_TYPE_OFFSET, type);
    stl_le_p(frame->bytes + SLUGARCH_T2_LENGTH_OFFSET, length);
    stl_le_p(frame->bytes + SLUGARCH_T2_FLAGS_OFFSET, 0);
    stq_le_p(frame->bytes + SLUGARCH_T2_REQUEST_ID_OFFSET, request_id);
    stq_le_p(frame->bytes + SLUGARCH_T2_CLIENT_ID_OFFSET, client_id);
    stl_le_p(frame->bytes + SLUGARCH_T2_CHECKSUM_OFFSET, 0);
    stl_le_p(frame->bytes + SLUGARCH_T2_RESERVED_OFFSET, 0);
    return true;
}

static void slugarch_t2_finish_frame(SlugArchT2Frame *frame)
{
    uint32_t checksum;

    stl_le_p(frame->bytes + SLUGARCH_T2_CHECKSUM_OFFSET, 0);
    checksum = slugarch_t2_crc32c(frame->bytes, frame->length);
    stl_le_p(frame->bytes + SLUGARCH_T2_CHECKSUM_OFFSET, checksum);
}

static bool slugarch_t2_decode_header_bytes(const uint8_t *bytes,
                                            size_t available,
                                            SlugArchT2Header *header,
                                            Error **errp)
{
    uint16_t raw_type;
    uint32_t frame_length;
    uint64_t request_id;
    uint64_t client_id;
    SlugArchT2FrameType type;

    if (!bytes || !header) {
        error_setg(errp, "SlugArch Type-2 header output is null");
        return false;
    }
    if (available < SLUGARCH_T2_HEADER_BYTES) {
        error_setg(errp, "SlugArch Type-2 frame is shorter than 40 bytes");
        return false;
    }
    if (memcmp(bytes, slugarch_t2_magic, sizeof(slugarch_t2_magic))) {
        error_setg(errp, "SlugArch Type-2 magic is invalid");
        return false;
    }
    if (lduw_le_p(bytes + SLUGARCH_T2_VERSION_OFFSET) !=
        SLUGARCH_T2_VERSION) {
        error_setg(errp, "SlugArch Type-2 version is not 1");
        return false;
    }

    raw_type = lduw_le_p(bytes + SLUGARCH_T2_TYPE_OFFSET);
    type = raw_type;
    frame_length = ldl_le_p(bytes + SLUGARCH_T2_LENGTH_OFFSET);
    if (!slugarch_t2_expected_length(type)) {
        error_setg(errp, "SlugArch Type-2 frame type %u is invalid",
                   raw_type);
        return false;
    }
    if (frame_length != slugarch_t2_expected_length(type) ||
        frame_length < SLUGARCH_T2_HEADER_BYTES ||
        frame_length > SLUGARCH_T2_MAX_FRAME) {
        error_setg(errp,
                   "SlugArch Type-2 frame length %u is invalid for type %u",
                   frame_length, raw_type);
        return false;
    }
    if (ldl_le_p(bytes + SLUGARCH_T2_FLAGS_OFFSET) != 0) {
        error_setg(errp, "SlugArch Type-2 flags must be zero");
        return false;
    }
    if (ldl_le_p(bytes + SLUGARCH_T2_RESERVED_OFFSET) != 0) {
        error_setg(errp, "SlugArch Type-2 header reserved field is nonzero");
        return false;
    }

    request_id = ldq_le_p(bytes + SLUGARCH_T2_REQUEST_ID_OFFSET);
    client_id = ldq_le_p(bytes + SLUGARCH_T2_CLIENT_ID_OFFSET);
    if (!request_id) {
        error_setg(errp, "SlugArch Type-2 request ID is zero");
        return false;
    }
    if ((type == SLUGARCH_T2_FRAME_HELLO && client_id != 0) ||
        (type != SLUGARCH_T2_FRAME_HELLO && client_id == 0)) {
        error_setg(errp, "SlugArch Type-2 client ID is invalid");
        return false;
    }

    *header = (SlugArchT2Header) {
        .type = type,
        .frame_length = frame_length,
        .request_id = request_id,
        .client_id = client_id,
    };
    return true;
}

static bool slugarch_t2_verify_frame(const SlugArchT2Frame *frame,
                                     SlugArchT2FrameType expected_type,
                                     SlugArchT2Header *header,
                                     Error **errp)
{
    uint8_t checksum_bytes[SLUGARCH_T2_MAX_FRAME];
    uint32_t received_checksum;
    uint32_t computed_checksum;

    if (!frame || frame->length < SLUGARCH_T2_HEADER_BYTES ||
        frame->length > SLUGARCH_T2_MAX_FRAME) {
        error_setg(errp, "SlugArch Type-2 frame length is outside [40, 128]");
        return false;
    }
    if (!slugarch_t2_decode_header_bytes(frame->bytes, frame->length,
                                         header, errp)) {
        return false;
    }
    if (frame->length != header->frame_length) {
        error_setg(errp,
                   "SlugArch Type-2 received length %u differs from header %u",
                   frame->length, header->frame_length);
        return false;
    }
    if (expected_type && header->type != expected_type) {
        error_setg(errp,
                   "SlugArch Type-2 response type %u differs from expected %u",
                   header->type, expected_type);
        return false;
    }

    memcpy(checksum_bytes, frame->bytes, frame->length);
    received_checksum =
        ldl_le_p(checksum_bytes + SLUGARCH_T2_CHECKSUM_OFFSET);
    stl_le_p(checksum_bytes + SLUGARCH_T2_CHECKSUM_OFFSET, 0);
    computed_checksum = slugarch_t2_crc32c(checksum_bytes, frame->length);
    if (received_checksum != computed_checksum) {
        error_setg(errp,
                   "SlugArch Type-2 CRC32C mismatch: received 0x%08x, "
                   "computed 0x%08x",
                   received_checksum, computed_checksum);
        return false;
    }
    return true;
}

uint32_t slugarch_t2_crc32c(const uint8_t *data, size_t length)
{
    g_assert(length <= UINT_MAX);
    return crc32c(0xffffffffU, data, length);
}

uint64_t slugarch_t2_monotonic_ns(void)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
        abort();
    }
    return (uint64_t)now.tv_sec * 1000000000ULL + now.tv_nsec;
}

bool slugarch_t2_encode_hello(const SlugArchT2Hello *hello,
                              SlugArchT2Frame *frame)
{
    if (!hello ||
        (hello->role != SLUGARCH_T2_ROLE_QEMU &&
         hello->role != SLUGARCH_T2_ROLE_ORACLE) ||
        !slugarch_t2_begin_frame(SLUGARCH_T2_FRAME_HELLO,
                                 hello->request_id, 0, frame)) {
        return false;
    }

    stl_le_p(frame->bytes + 40, hello->role);
    stl_le_p(frame->bytes + 44, 0);
    memcpy(frame->bytes + 48, hello->client_nonce,
           sizeof(hello->client_nonce));
    stl_le_p(frame->bytes + 64, SLUGARCH_T2_MAX_FRAME);
    stl_le_p(frame->bytes + 68, 0);
    slugarch_t2_finish_frame(frame);
    return true;
}

bool slugarch_t2_decode_hello(const SlugArchT2Frame *frame,
                              SlugArchT2Hello *hello,
                              Error **errp)
{
    SlugArchT2Header header;
    uint32_t role;

    if (!hello) {
        error_setg(errp, "SlugArch Type-2 HELLO output is null");
        return false;
    }
    if (!slugarch_t2_verify_frame(frame, SLUGARCH_T2_FRAME_HELLO,
                                  &header, errp)) {
        return false;
    }
    role = ldl_le_p(frame->bytes + 40);
    if ((role != SLUGARCH_T2_ROLE_QEMU &&
         role != SLUGARCH_T2_ROLE_ORACLE) ||
        ldl_le_p(frame->bytes + 44) != 0 ||
        ldl_le_p(frame->bytes + 64) != SLUGARCH_T2_MAX_FRAME ||
        ldl_le_p(frame->bytes + 68) != 0) {
        error_setg(errp, "SlugArch Type-2 HELLO body is invalid");
        return false;
    }

    memset(hello, 0, sizeof(*hello));
    hello->request_id = header.request_id;
    hello->role = role;
    memcpy(hello->client_nonce, frame->bytes + 48,
           sizeof(hello->client_nonce));
    return true;
}

bool slugarch_t2_encode_ack(const SlugArchT2Ack *ack,
                            SlugArchT2Frame *frame)
{
    if (!ack || !slugarch_t2_valid_status(ack->status) ||
        !slugarch_t2_begin_frame(SLUGARCH_T2_FRAME_ACK,
                                 ack->request_id, ack->client_id, frame)) {
        return false;
    }

    stl_le_p(frame->bytes + 40, ack->status);
    stl_le_p(frame->bytes + 44, SLUGARCH_T2_MAX_FRAME);
    memcpy(frame->bytes + 48, ack->server_uuid,
           sizeof(ack->server_uuid));
    stq_le_p(frame->bytes + 64, ack->capacity);
    stq_le_p(frame->bytes + 72, ack->configured_base_latency);
    stq_le_p(frame->bytes + 80, 0);
    slugarch_t2_finish_frame(frame);
    return true;
}

bool slugarch_t2_decode_ack(const SlugArchT2Frame *frame,
                            SlugArchT2Ack *ack,
                            Error **errp)
{
    SlugArchT2Header header;
    uint32_t status;
    uint32_t maximum_frame;

    if (!ack) {
        error_setg(errp, "SlugArch Type-2 ACK output is null");
        return false;
    }
    if (!slugarch_t2_verify_frame(frame, SLUGARCH_T2_FRAME_ACK,
                                  &header, errp)) {
        return false;
    }
    status = ldl_le_p(frame->bytes + 40);
    maximum_frame = ldl_le_p(frame->bytes + 44);
    if (!slugarch_t2_valid_status(status)) {
        error_setg(errp, "SlugArch Type-2 ACK status is invalid");
        return false;
    }
    if (maximum_frame != SLUGARCH_T2_MAX_FRAME) {
        error_setg(errp,
                   "SlugArch Type-2 ACK maximum frame %u is not 128",
                   maximum_frame);
        return false;
    }
    if (ldq_le_p(frame->bytes + 80) != 0) {
        error_setg(errp, "SlugArch Type-2 ACK reserved field is nonzero");
        return false;
    }

    memset(ack, 0, sizeof(*ack));
    ack->request_id = header.request_id;
    ack->client_id = header.client_id;
    ack->status = status;
    ack->maximum_frame = maximum_frame;
    memcpy(ack->server_uuid, frame->bytes + 48,
           sizeof(ack->server_uuid));
    ack->capacity = ldq_le_p(frame->bytes + 64);
    ack->configured_base_latency = ldq_le_p(frame->bytes + 72);
    return true;
}

bool slugarch_t2_encode_memory_request(
    const SlugArchT2MemoryRequest *request,
    SlugArchT2Frame *frame)
{
    if (!request ||
        (request->type != SLUGARCH_T2_FRAME_READ &&
         request->type != SLUGARCH_T2_FRAME_WRITE) ||
        !slugarch_t2_begin_frame(request->type, request->request_id,
                                 request->client_id, frame)) {
        return false;
    }

    stl_le_p(frame->bytes + 40, request->length);
    stl_le_p(frame->bytes + 44, 0);
    stq_le_p(frame->bytes + 48, request->dpa);
    stq_le_p(frame->bytes + 56, request->client_monotonic_ns);
    memcpy(frame->bytes + 64, request->data, sizeof(request->data));
    slugarch_t2_finish_frame(frame);
    return true;
}

bool slugarch_t2_decode_memory_request(
    const SlugArchT2Frame *frame,
    SlugArchT2MemoryRequest *request,
    Error **errp)
{
    SlugArchT2Header header;

    if (!request) {
        error_setg(errp,
                   "SlugArch Type-2 memory request output is null");
        return false;
    }
    if (!slugarch_t2_verify_frame(frame, 0, &header, errp)) {
        return false;
    }
    if (header.type != SLUGARCH_T2_FRAME_READ &&
        header.type != SLUGARCH_T2_FRAME_WRITE) {
        error_setg(errp, "SlugArch Type-2 frame is not a memory request");
        return false;
    }
    if (ldl_le_p(frame->bytes + 44) != 0) {
        error_setg(errp,
                   "SlugArch Type-2 memory request reserved field is nonzero");
        return false;
    }

    memset(request, 0, sizeof(*request));
    request->type = header.type;
    request->request_id = header.request_id;
    request->client_id = header.client_id;
    request->length = ldl_le_p(frame->bytes + 40);
    request->dpa = ldq_le_p(frame->bytes + 48);
    request->client_monotonic_ns = ldq_le_p(frame->bytes + 56);
    memcpy(request->data, frame->bytes + 64, sizeof(request->data));
    return true;
}

bool slugarch_t2_encode_memory_response(
    const SlugArchT2MemoryResponse *response,
    SlugArchT2Frame *frame)
{
    if (!response || !slugarch_t2_valid_status(response->status) ||
        response->returned_length > SLUGARCH_T2_MAX_PAYLOAD ||
        !slugarch_t2_begin_frame(SLUGARCH_T2_FRAME_MEMORY_RESPONSE,
                                 response->request_id,
                                 response->client_id, frame)) {
        return false;
    }

    stl_le_p(frame->bytes + 40, response->status);
    stl_le_p(frame->bytes + 44, response->returned_length);
    stq_le_p(frame->bytes + 48, response->server_sequence);
    stq_le_p(frame->bytes + 56, response->modeled_latency);
    memcpy(frame->bytes + 64, response->data,
           sizeof(response->data));
    slugarch_t2_finish_frame(frame);
    return true;
}

bool slugarch_t2_decode_memory_response(
    const SlugArchT2Frame *frame,
    SlugArchT2MemoryResponse *response,
    Error **errp)
{
    SlugArchT2Header header;
    uint32_t status;
    uint32_t returned_length;

    if (!response) {
        error_setg(errp,
                   "SlugArch Type-2 memory response output is null");
        return false;
    }
    if (!slugarch_t2_verify_frame(frame,
                                  SLUGARCH_T2_FRAME_MEMORY_RESPONSE,
                                  &header, errp)) {
        return false;
    }
    status = ldl_le_p(frame->bytes + 40);
    returned_length = ldl_le_p(frame->bytes + 44);
    if (!slugarch_t2_valid_status(status) ||
        returned_length > SLUGARCH_T2_MAX_PAYLOAD) {
        error_setg(errp, "SlugArch Type-2 memory response body is invalid");
        return false;
    }

    memset(response, 0, sizeof(*response));
    response->request_id = header.request_id;
    response->client_id = header.client_id;
    response->status = status;
    response->returned_length = returned_length;
    response->server_sequence = ldq_le_p(frame->bytes + 48);
    response->modeled_latency = ldq_le_p(frame->bytes + 56);
    memcpy(response->data, frame->bytes + 64,
           sizeof(response->data));
    return true;
}

bool slugarch_t2_encode_error(const SlugArchT2ErrorResponse *response,
                              SlugArchT2Frame *frame)
{
    if (!response || !slugarch_t2_valid_status(response->status) ||
        !slugarch_t2_begin_frame(SLUGARCH_T2_FRAME_ERROR,
                                 response->request_id,
                                 response->client_id, frame)) {
        return false;
    }

    stl_le_p(frame->bytes + 40, response->status);
    stl_le_p(frame->bytes + 44, response->reason);
    stq_le_p(frame->bytes + 48, response->related_request_id);
    slugarch_t2_finish_frame(frame);
    return true;
}

bool slugarch_t2_decode_error(const SlugArchT2Frame *frame,
                              SlugArchT2ErrorResponse *response,
                              Error **errp)
{
    SlugArchT2Header header;
    uint32_t status;

    if (!response) {
        error_setg(errp, "SlugArch Type-2 error response output is null");
        return false;
    }
    if (!slugarch_t2_verify_frame(frame, SLUGARCH_T2_FRAME_ERROR,
                                  &header, errp)) {
        return false;
    }
    status = ldl_le_p(frame->bytes + 40);
    if (!slugarch_t2_valid_status(status)) {
        error_setg(errp, "SlugArch Type-2 error status is invalid");
        return false;
    }

    memset(response, 0, sizeof(*response));
    response->request_id = header.request_id;
    response->client_id = header.client_id;
    response->status = status;
    response->reason = ldl_le_p(frame->bytes + 44);
    response->related_request_id = ldq_le_p(frame->bytes + 48);
    return true;
}

bool slugarch_t2_decode_header(const SlugArchT2Frame *frame,
                               SlugArchT2Header *header,
                               Error **errp)
{
    return slugarch_t2_verify_frame(frame, 0, header, errp);
}

static SlugArchT2IOResult slugarch_t2_prepare_socket(
    QIOChannelSocket *socket,
    Error **errp)
{
    if (!socket || socket->fd < 0) {
        error_setg(errp, "SlugArch Type-2 socket is not connected");
        return SLUGARCH_T2_IO_SYSTEM_ERROR;
    }
    if (qio_channel_set_blocking(QIO_CHANNEL(socket), false, errp) < 0) {
        return SLUGARCH_T2_IO_SYSTEM_ERROR;
    }
    return SLUGARCH_T2_IO_OK;
}

static SlugArchT2IOResult slugarch_t2_wait_fd(
    QIOChannelSocket *socket,
    GIOCondition events,
    uint64_t deadline_ns,
    Error **errp)
{
    GPollFD descriptor = {
        .fd = socket->fd,
        .events = events,
    };

    for (;;) {
        uint64_t now_ns = slugarch_t2_monotonic_ns();
        uint64_t remaining_ns;
        int result;

        if (now_ns >= deadline_ns) {
            error_setg(errp, "SlugArch Type-2 deadline expired");
            return SLUGARCH_T2_IO_TIMEOUT;
        }
        remaining_ns = deadline_ns - now_ns;
        result = qemu_poll_ns(&descriptor, 1, remaining_ns);
        if (result > 0) {
            if (descriptor.revents & G_IO_NVAL) {
                error_setg(errp, "SlugArch Type-2 poll reported invalid fd");
                return SLUGARCH_T2_IO_SYSTEM_ERROR;
            }
            return SLUGARCH_T2_IO_OK;
        }
        if (result == 0) {
            error_setg(errp, "SlugArch Type-2 deadline expired");
            return SLUGARCH_T2_IO_TIMEOUT;
        }
        if (errno != EINTR) {
            error_setg_errno(errp, errno, "SlugArch Type-2 poll failed");
            return SLUGARCH_T2_IO_SYSTEM_ERROR;
        }
        descriptor.revents = 0;
    }
}

static SlugArchT2IOResult slugarch_t2_read_exact(
    QIOChannelSocket *socket,
    uint8_t *bytes,
    size_t length,
    uint64_t deadline_ns,
    Error **errp)
{
    size_t offset = 0;

    while (offset < length) {
        SlugArchT2IOResult wait_result =
            slugarch_t2_wait_fd(socket, G_IO_IN, deadline_ns, errp);
        ssize_t received;

        if (wait_result != SLUGARCH_T2_IO_OK) {
            return wait_result;
        }
        received = recv(socket->fd, bytes + offset, length - offset,
                        MSG_DONTWAIT);
        if (received > 0) {
            offset += received;
            continue;
        }
        if (received == 0) {
            error_setg(errp,
                       "SlugArch Type-2 EOF after %zu of %zu bytes",
                       offset, length);
            return SLUGARCH_T2_IO_EOF;
        }
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
            continue;
        }
        error_setg_errno(errp, errno, "SlugArch Type-2 receive failed");
        return SLUGARCH_T2_IO_SYSTEM_ERROR;
    }
    return SLUGARCH_T2_IO_OK;
}

SlugArchT2IOResult slugarch_t2_read_frame_until(
    QIOChannelSocket *socket,
    uint64_t deadline_ns,
    SlugArchT2Frame *frame,
    Error **errp)
{
    SlugArchT2Header header;
    SlugArchT2IOResult result;

    if (!frame) {
        error_setg(errp, "SlugArch Type-2 frame output is null");
        return SLUGARCH_T2_IO_PROTOCOL_ERROR;
    }
    result = slugarch_t2_prepare_socket(socket, errp);
    if (result != SLUGARCH_T2_IO_OK) {
        return result;
    }

    memset(frame, 0, sizeof(*frame));
    result = slugarch_t2_read_exact(socket, frame->bytes,
                                    SLUGARCH_T2_HEADER_BYTES,
                                    deadline_ns, errp);
    if (result != SLUGARCH_T2_IO_OK) {
        return result;
    }
    if (!slugarch_t2_decode_header_bytes(frame->bytes,
                                         SLUGARCH_T2_HEADER_BYTES,
                                         &header, errp)) {
        return SLUGARCH_T2_IO_PROTOCOL_ERROR;
    }
    frame->length = header.frame_length;
    result = slugarch_t2_read_exact(
        socket, frame->bytes + SLUGARCH_T2_HEADER_BYTES,
        frame->length - SLUGARCH_T2_HEADER_BYTES, deadline_ns, errp);
    if (result != SLUGARCH_T2_IO_OK) {
        return result;
    }
    if (!slugarch_t2_verify_frame(frame, 0, &header, errp)) {
        return SLUGARCH_T2_IO_PROTOCOL_ERROR;
    }
    return SLUGARCH_T2_IO_OK;
}

SlugArchT2IOResult slugarch_t2_write_frame_until(
    QIOChannelSocket *socket,
    uint64_t deadline_ns,
    const SlugArchT2Frame *frame,
    Error **errp)
{
    SlugArchT2Header header;
    SlugArchT2IOResult result;
    size_t offset = 0;

    if (!slugarch_t2_verify_frame(frame, 0, &header, errp)) {
        return SLUGARCH_T2_IO_PROTOCOL_ERROR;
    }
    result = slugarch_t2_prepare_socket(socket, errp);
    if (result != SLUGARCH_T2_IO_OK) {
        return result;
    }

    while (offset < frame->length) {
        ssize_t sent;

        result = slugarch_t2_wait_fd(socket, G_IO_OUT,
                                     deadline_ns, errp);
        if (result != SLUGARCH_T2_IO_OK) {
            return result;
        }
        sent = send(socket->fd, frame->bytes + offset,
                    frame->length - offset,
                    MSG_DONTWAIT | MSG_NOSIGNAL);
        if (sent > 0) {
            offset += sent;
            continue;
        }
        if (sent == 0) {
            error_setg(errp,
                       "SlugArch Type-2 zero-byte send after %zu bytes",
                       offset);
            return SLUGARCH_T2_IO_EOF;
        }
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
            continue;
        }
        error_setg_errno(errp, errno, "SlugArch Type-2 send failed");
        return SLUGARCH_T2_IO_SYSTEM_ERROR;
    }
    return SLUGARCH_T2_IO_OK;
}

SlugArchT2IOResult slugarch_t2_exchange_until(
    QIOChannelSocket *socket,
    const SlugArchT2Frame *request,
    uint64_t deadline_ns,
    SlugArchT2Frame *response,
    Error **errp)
{
    SlugArchT2IOResult result =
        slugarch_t2_write_frame_until(socket, deadline_ns, request, errp);

    if (result != SLUGARCH_T2_IO_OK) {
        return result;
    }
    return slugarch_t2_read_frame_until(socket, deadline_ns,
                                        response, errp);
}

bool slugarch_t2_apply_delay(uint64_t requested_ns,
                             SlugArchT2DelayResult *result,
                             Error **errp)
{
    struct timespec start, now;
    uint64_t start_ns, now_ns;

    if (!result) {
        error_setg(errp, "SlugArch Type-2 delay result is null");
        return false;
    }
    if (requested_ns > 1000000ULL) {
        error_setg(errp, "SlugArch Type-2 delay %" PRIu64
                   " exceeds 1000000 ns", requested_ns);
        return false;
    }
    if (clock_gettime(CLOCK_MONOTONIC_RAW, &start) != 0) {
        error_setg_errno(errp, errno,
                         "CLOCK_MONOTONIC_RAW start failed");
        return false;
    }
    start_ns = (uint64_t)start.tv_sec * 1000000000ULL + start.tv_nsec;
    do {
        cpu_relax();
        if (clock_gettime(CLOCK_MONOTONIC_RAW, &now) != 0) {
            error_setg_errno(errp, errno,
                             "CLOCK_MONOTONIC_RAW read failed");
            return false;
        }
        now_ns = (uint64_t)now.tv_sec * 1000000000ULL + now.tv_nsec;
    } while (now_ns - start_ns < requested_ns);

    result->requested_ns = requested_ns;
    result->actual_ns = now_ns - start_ns;
    result->undershot = result->actual_ns < requested_ns;
    result->overshoot_ns = result->actual_ns - requested_ns;
    return true;
}
