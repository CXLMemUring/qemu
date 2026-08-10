/*
 * SlugArch synchronous CXL Type-2 wire protocol
 *
 * All public structures contain host-order fields.  Wire frames are encoded
 * and decoded field by field; no native structure is overlaid on wire bytes.
 */

#ifndef HW_CXL_SLUGARCH_TYPE2_PROTOCOL_H
#define HW_CXL_SLUGARCH_TYPE2_PROTOCOL_H

#include "io/channel-socket.h"
#include "qapi/error.h"

#define SLUGARCH_T2_VERSION 1
#define SLUGARCH_T2_HEADER_BYTES 40
#define SLUGARCH_T2_MAX_FRAME 128
#define SLUGARCH_T2_MAX_PAYLOAD 64
#define SLUGARCH_T2_REQUEST_TIMEOUT_NS 5000000000ULL

typedef enum SlugArchT2FrameType {
    SLUGARCH_T2_FRAME_HELLO = 1,
    SLUGARCH_T2_FRAME_ACK = 2,
    SLUGARCH_T2_FRAME_READ = 3,
    SLUGARCH_T2_FRAME_WRITE = 4,
    SLUGARCH_T2_FRAME_MEMORY_RESPONSE = 5,
    SLUGARCH_T2_FRAME_COUNTER_SNAPSHOT = 6,
    SLUGARCH_T2_FRAME_COUNTER_RESPONSE = 7,
    SLUGARCH_T2_FRAME_ERROR = 255,
} SlugArchT2FrameType;

typedef enum SlugArchT2Role {
    SLUGARCH_T2_ROLE_QEMU = 1,
    SLUGARCH_T2_ROLE_ORACLE = 2,
} SlugArchT2Role;

typedef enum SlugArchT2Status {
    SLUGARCH_T2_STATUS_SUCCESS = 0,
    SLUGARCH_T2_STATUS_PROTOCOL = 1,
    SLUGARCH_T2_STATUS_RANGE = 2,
    SLUGARCH_T2_STATUS_CHECKSUM = 3,
    SLUGARCH_T2_STATUS_BACKEND = 4,
    SLUGARCH_T2_STATUS_UNSUPPORTED = 5,
} SlugArchT2Status;

typedef enum SlugArchT2IOResult {
    SLUGARCH_T2_IO_OK = 0,
    SLUGARCH_T2_IO_TIMEOUT,
    SLUGARCH_T2_IO_EOF,
    SLUGARCH_T2_IO_SYSTEM_ERROR,
    SLUGARCH_T2_IO_PROTOCOL_ERROR,
} SlugArchT2IOResult;

typedef struct SlugArchT2Header {
    SlugArchT2FrameType type;
    uint32_t frame_length;
    uint64_t request_id;
    uint64_t client_id;
} SlugArchT2Header;

typedef struct SlugArchT2Frame {
    uint8_t bytes[SLUGARCH_T2_MAX_FRAME];
    uint32_t length;
} SlugArchT2Frame;

typedef struct SlugArchT2Hello {
    uint64_t request_id;
    SlugArchT2Role role;
    uint8_t client_nonce[16];
} SlugArchT2Hello;

typedef struct SlugArchT2Ack {
    uint64_t request_id;
    uint64_t client_id;
    SlugArchT2Status status;
    uint32_t maximum_frame;
    uint8_t server_uuid[16];
    uint64_t capacity;
    uint64_t configured_base_latency;
} SlugArchT2Ack;

typedef struct SlugArchT2MemoryRequest {
    SlugArchT2FrameType type;
    uint64_t request_id;
    uint64_t client_id;
    uint32_t length;
    uint64_t dpa;
    uint64_t client_monotonic_ns;
    uint8_t data[SLUGARCH_T2_MAX_PAYLOAD];
} SlugArchT2MemoryRequest;

typedef struct SlugArchT2MemoryResponse {
    uint64_t request_id;
    uint64_t client_id;
    SlugArchT2Status status;
    uint32_t returned_length;
    uint64_t server_sequence;
    uint64_t modeled_latency;
    uint8_t data[SLUGARCH_T2_MAX_PAYLOAD];
} SlugArchT2MemoryResponse;

typedef struct SlugArchT2ErrorResponse {
    uint64_t request_id;
    uint64_t client_id;
    SlugArchT2Status status;
    uint32_t reason;
    uint64_t related_request_id;
} SlugArchT2ErrorResponse;

typedef struct SlugArchT2DelayResult {
    uint64_t requested_ns;
    uint64_t actual_ns;
    uint64_t overshoot_ns;
    bool undershot;
} SlugArchT2DelayResult;

uint32_t slugarch_t2_crc32c(const uint8_t *data, size_t length);
uint64_t slugarch_t2_monotonic_ns(void);

bool slugarch_t2_encode_hello(const SlugArchT2Hello *hello,
                              SlugArchT2Frame *frame);
bool slugarch_t2_decode_hello(const SlugArchT2Frame *frame,
                              SlugArchT2Hello *hello,
                              Error **errp);
bool slugarch_t2_encode_ack(const SlugArchT2Ack *ack,
                            SlugArchT2Frame *frame);
bool slugarch_t2_decode_ack(const SlugArchT2Frame *frame,
                            SlugArchT2Ack *ack,
                            Error **errp);
bool slugarch_t2_encode_memory_request(
    const SlugArchT2MemoryRequest *request,
    SlugArchT2Frame *frame);
bool slugarch_t2_decode_memory_request(
    const SlugArchT2Frame *frame,
    SlugArchT2MemoryRequest *request,
    Error **errp);
bool slugarch_t2_encode_memory_response(
    const SlugArchT2MemoryResponse *response,
    SlugArchT2Frame *frame);
bool slugarch_t2_decode_memory_response(
    const SlugArchT2Frame *frame,
    SlugArchT2MemoryResponse *response,
    Error **errp);
bool slugarch_t2_encode_error(const SlugArchT2ErrorResponse *response,
                              SlugArchT2Frame *frame);
bool slugarch_t2_decode_error(const SlugArchT2Frame *frame,
                              SlugArchT2ErrorResponse *response,
                              Error **errp);
bool slugarch_t2_decode_header(const SlugArchT2Frame *frame,
                               SlugArchT2Header *header,
                               Error **errp);

SlugArchT2IOResult slugarch_t2_read_frame_until(
    QIOChannelSocket *socket,
    uint64_t deadline_ns,
    SlugArchT2Frame *frame,
    Error **errp);
SlugArchT2IOResult slugarch_t2_write_frame_until(
    QIOChannelSocket *socket,
    uint64_t deadline_ns,
    const SlugArchT2Frame *frame,
    Error **errp);
SlugArchT2IOResult slugarch_t2_exchange_until(
    QIOChannelSocket *socket,
    const SlugArchT2Frame *request,
    uint64_t deadline_ns,
    SlugArchT2Frame *response,
    Error **errp);

bool slugarch_t2_apply_delay(uint64_t requested_ns,
                             SlugArchT2DelayResult *result,
                             Error **errp);

#endif
