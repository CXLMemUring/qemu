#include "qemu/osdep.h"

#include "hw/cxl/slugarch_type2_protocol.h"
#include "io/channel-socket.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qom/object.h"

static const uint8_t k_hello_golden[] = {
    0x53, 0x4c, 0x54, 0x32, 0x01, 0x00, 0x01, 0x00,
    0x48, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xcf, 0x70, 0x20, 0xd2, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static uint64_t deadline_after_ms(uint64_t milliseconds)
{
    return slugarch_t2_monotonic_ns() + milliseconds * 1000000ULL;
}

static SlugArchT2Frame normative_hello(void)
{
    SlugArchT2Hello hello = {
        .request_id = 1,
        .role = SLUGARCH_T2_ROLE_QEMU,
    };
    SlugArchT2Frame frame = { 0 };
    size_t i;

    for (i = 0; i < sizeof(hello.client_nonce); i++) {
        hello.client_nonce[i] = i;
    }
    g_assert_true(slugarch_t2_encode_hello(&hello, &frame));
    return frame;
}

static QIOChannelSocket *wrap_nonblocking_socket(int fd)
{
    QIOChannelSocket *socket;
    Error *err = NULL;

    socket = qio_channel_socket_new_fd(fd, &err);
    g_assert_nonnull(socket);
    g_assert_null(err);
    g_assert_cmpint(qio_channel_set_blocking(QIO_CHANNEL(socket),
                                             false, &err), ==, 0);
    g_assert_null(err);
    return socket;
}

static void test_crc32c(void)
{
    static const uint8_t input[] = "123456789";

    g_assert_cmphex(slugarch_t2_crc32c(input, 9), ==, 0xe3069283U);
}

static void test_hello_golden(void)
{
    SlugArchT2Hello hello = {
        .request_id = 1,
        .role = SLUGARCH_T2_ROLE_QEMU,
    };
    SlugArchT2Frame frame = { 0 };
    size_t i;

    for (i = 0; i < sizeof(hello.client_nonce); i++) {
        hello.client_nonce[i] = i;
    }
    g_assert_true(slugarch_t2_encode_hello(&hello, &frame));
    g_assert_cmpuint(frame.length, ==, sizeof(k_hello_golden));
    g_assert_cmpmem(frame.bytes, frame.length,
                    k_hello_golden, sizeof(k_hello_golden));
}

static void assert_hello_rejected(SlugArchT2Frame *frame)
{
    SlugArchT2Hello decoded = { 0 };
    Error *err = NULL;

    g_assert_false(slugarch_t2_decode_hello(frame, &decoded, &err));
    g_assert_nonnull(err);
    error_free(err);
}

static void test_bad_version(void)
{
    SlugArchT2Frame frame = normative_hello();

    frame.bytes[4] = 2;
    assert_hello_rejected(&frame);
}

static void test_bad_fixed_length(void)
{
    SlugArchT2Frame frame = normative_hello();

    frame.bytes[8] = 71;
    assert_hello_rejected(&frame);
}

static void test_nonzero_reserved(void)
{
    SlugArchT2Frame frame = normative_hello();

    frame.bytes[36] = 1;
    assert_hello_rejected(&frame);
}

static void test_bad_crc(void)
{
    SlugArchT2Frame frame = normative_hello();

    frame.bytes[40] ^= 1;
    assert_hello_rejected(&frame);
}

typedef struct FragmentWriter {
    int fd;
    int error_code;
} FragmentWriter;

static gpointer write_fragments(gpointer opaque)
{
    FragmentWriter *writer = opaque;
    size_t i;

    for (i = 0; i < sizeof(k_hello_golden); i++) {
        ssize_t sent;

        do {
            sent = send(writer->fd, &k_hello_golden[i], 1, MSG_NOSIGNAL);
        } while (sent < 0 && errno == EINTR);
        if (sent != 1) {
            writer->error_code = errno;
            break;
        }
    }
    close(writer->fd);
    return NULL;
}

static void test_fragmented_read(void)
{
    int sockets[2];
    FragmentWriter writer;
    GThread *thread;
    QIOChannelSocket *reader;
    SlugArchT2Frame frame = { 0 };
    Error *err = NULL;

    g_assert_cmpint(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), ==, 0);
    writer = (FragmentWriter) {
        .fd = sockets[0],
    };
    reader = wrap_nonblocking_socket(sockets[1]);
    thread = g_thread_new("slugarch-fragments", write_fragments, &writer);

    g_assert_cmpint(slugarch_t2_read_frame_until(
                        reader, deadline_after_ms(1000), &frame, &err),
                    ==, SLUGARCH_T2_IO_OK);
    g_assert_null(err);
    g_thread_join(thread);
    g_assert_cmpint(writer.error_code, ==, 0);
    g_assert_cmpmem(frame.bytes, frame.length,
                    k_hello_golden, sizeof(k_hello_golden));
    object_unref(OBJECT(reader));
}

static void test_partial_eof(void)
{
    int sockets[2];
    QIOChannelSocket *reader;
    SlugArchT2Frame frame = { 0 };
    Error *err = NULL;

    g_assert_cmpint(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), ==, 0);
    reader = wrap_nonblocking_socket(sockets[1]);
    g_assert_cmpint(send(sockets[0], k_hello_golden, 20, MSG_NOSIGNAL),
                    ==, 20);
    close(sockets[0]);

    g_assert_cmpint(slugarch_t2_read_frame_until(
                        reader, deadline_after_ms(1000), &frame, &err),
                    ==, SLUGARCH_T2_IO_EOF);
    g_assert_nonnull(err);
    error_free(err);
    object_unref(OBJECT(reader));
}

static void test_read_timeout(void)
{
    int sockets[2];
    QIOChannelSocket *reader;
    SlugArchT2Frame frame = { 0 };
    Error *err = NULL;
    uint64_t start_ns;
    uint64_t elapsed_ns;

    g_assert_cmpint(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), ==, 0);
    reader = wrap_nonblocking_socket(sockets[1]);
    start_ns = slugarch_t2_monotonic_ns();
    g_assert_cmpint(slugarch_t2_read_frame_until(
                        reader, start_ns + 20000000ULL, &frame, &err),
                    ==, SLUGARCH_T2_IO_TIMEOUT);
    elapsed_ns = slugarch_t2_monotonic_ns() - start_ns;
    g_assert_nonnull(err);
    g_assert_cmpuint(elapsed_ns, >=, 15000000ULL);
    g_assert_cmpuint(elapsed_ns, <, 100000000ULL);
    error_free(err);
    close(sockets[0]);
    object_unref(OBJECT(reader));
}

typedef struct BudgetPeer {
    int fd;
    size_t drained;
} BudgetPeer;

static gpointer drain_after_fifteen_ms(gpointer opaque)
{
    BudgetPeer *peer = opaque;
    uint8_t buffer[16384];
    ssize_t received;

    g_usleep(15000);
    do {
        received = recv(peer->fd, buffer, sizeof(buffer), 0);
    } while (received < 0 && errno == EINTR);
    if (received > 0) {
        peer->drained = received;
    }
    g_usleep(20000);
    close(peer->fd);
    return NULL;
}

static void fill_send_buffer(int fd)
{
    uint8_t filler[4096] = { 0 };

    for (;;) {
        ssize_t sent = send(fd, filler, sizeof(filler), MSG_NOSIGNAL);

        if (sent > 0) {
            continue;
        }
        g_assert_cmpint(sent, ==, -1);
        if (errno == EINTR) {
            continue;
        }
        g_assert_true(errno == EAGAIN || errno == EWOULDBLOCK);
        break;
    }
}

static void test_exchange_uses_one_deadline(void)
{
    int sockets[2];
    QIOChannelSocket *client;
    SlugArchT2Frame request = normative_hello();
    SlugArchT2Frame response = { 0 };
    BudgetPeer peer;
    GThread *thread;
    Error *err = NULL;
    uint64_t start_ns;
    uint64_t elapsed_ns;

    g_assert_cmpint(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), ==, 0);
    client = wrap_nonblocking_socket(sockets[0]);
    fill_send_buffer(sockets[0]);
    peer = (BudgetPeer) {
        .fd = sockets[1],
    };
    thread = g_thread_new("slugarch-budget", drain_after_fifteen_ms, &peer);

    start_ns = slugarch_t2_monotonic_ns();
    g_assert_cmpint(slugarch_t2_exchange_until(
                        client, &request, start_ns + 20000000ULL,
                        &response, &err),
                    ==, SLUGARCH_T2_IO_TIMEOUT);
    elapsed_ns = slugarch_t2_monotonic_ns() - start_ns;
    g_assert_nonnull(err);
    g_assert_cmpuint(elapsed_ns, >=, 15000000ULL);
    g_assert_cmpuint(elapsed_ns, <, 32000000ULL);
    error_free(err);

    g_thread_join(thread);
    g_assert_cmpuint(peer.drained, >, 0);
    object_unref(OBJECT(client));
}

static void test_delay_bounds(void)
{
    SlugArchT2DelayResult result = { 0 };
    Error *err = NULL;

    g_assert_true(slugarch_t2_apply_delay(80, &result, &err));
    g_assert_null(err);
    g_assert_cmpuint(result.requested_ns, ==, 80);
    g_assert_cmpuint(result.actual_ns, >=, 80);
    g_assert_false(result.undershot);
    g_assert_cmpuint(result.overshoot_ns, ==, result.actual_ns - 80);
}

static void test_delay_rejects_over_one_ms(void)
{
    SlugArchT2DelayResult result = { 0 };
    Error *err = NULL;

    g_assert_false(slugarch_t2_apply_delay(1000001, &result, &err));
    g_assert_nonnull(err);
    g_assert_cmpstr(error_get_pretty(err), ==,
                    "SlugArch Type-2 delay 1000001 exceeds 1000000 ns");
    error_free(err);
}

int main(int argc, char **argv)
{
    module_call_init(MODULE_INIT_QOM);
    g_test_init(&argc, &argv, NULL);

    g_test_add_func("/cxl/type2/wire/crc32c", test_crc32c);
    g_test_add_func("/cxl/type2/wire/hello-golden", test_hello_golden);
    g_test_add_func("/cxl/type2/wire/bad-version", test_bad_version);
    g_test_add_func("/cxl/type2/wire/bad-fixed-length",
                    test_bad_fixed_length);
    g_test_add_func("/cxl/type2/wire/nonzero-reserved",
                    test_nonzero_reserved);
    g_test_add_func("/cxl/type2/wire/bad-crc", test_bad_crc);
    g_test_add_func("/cxl/type2/wire/fragmented-read",
                    test_fragmented_read);
    g_test_add_func("/cxl/type2/wire/partial-eof", test_partial_eof);
    g_test_add_func("/cxl/type2/wire/read-timeout", test_read_timeout);
    g_test_add_func("/cxl/type2/wire/one-deadline",
                    test_exchange_uses_one_deadline);
    g_test_add_func("/cxl/type2/wire/delay-bounds", test_delay_bounds);
    g_test_add_func("/cxl/type2/wire/delay-maximum",
                    test_delay_rejects_over_one_ms);

    return g_test_run();
}
