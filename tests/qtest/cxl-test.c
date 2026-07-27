/*
 * QTest testcase for CXL
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest-single.h"
#include "hw/cxl/cxl_component.h"
#include "hw/pci/pci_regs.h"
#include "qemu/bswap.h"
#include "qemu/crc32c.h"
#include "qemu/units.h"

#include <poll.h>

#define QEMU_PXB_CMD \
    "-machine q35,cxl=on " \
    "-device pxb-cxl,id=cxl.0,bus=pcie.0,bus_nr=52 " \
    "-M cxl-fmw.0.targets.0=cxl.0,cxl-fmw.0.size=4G "

#define QEMU_2PXB_CMD \
    "-machine q35,cxl=on " \
    "-device pxb-cxl,id=cxl.0,bus=pcie.0,bus_nr=52 " \
    "-device pxb-cxl,id=cxl.1,bus=pcie.0,bus_nr=53 " \
    "-M cxl-fmw.0.targets.0=cxl.0,cxl-fmw.0.targets.1=cxl.1,cxl-fmw.0.size=4G "

#define QEMU_VIRT_2PXB_CMD \
    "-machine virt,cxl=on -cpu max " \
    "-device pxb-cxl,id=cxl.0,bus=pcie.0,bus_nr=52 " \
    "-device pxb-cxl,id=cxl.1,bus=pcie.0,bus_nr=53 " \
    "-M cxl-fmw.0.targets.0=cxl.0,cxl-fmw.0.targets.1=cxl.1,cxl-fmw.0.size=4G "

#define QEMU_RP \
    "-device cxl-rp,id=rp0,bus=cxl.0,chassis=0,slot=0 "

#define QEMU_T2_SYNC_BASE \
    "-machine q35,cxl=on " \
    "-device pxb-cxl,id=cxl.0,bus=pcie.0,bus_nr=52 " \
    "-M cxl-fmw.0.targets.0=cxl.0,cxl-fmw.0.size=256M " \
    "-device cxl-rp,id=rp0,bus=cxl.0,chassis=0,slot=0 "

#define QEMU_T2_CFMWS_BASE \
    "-machine q35,cxl=on -m 128M " \
    "-device pxb-cxl,id=cxl.0,bus=pcie.0,bus_nr=52," \
    "hdm_for_passthrough=on " \
    "-M cxl-fmw.0.targets.0=cxl.0,cxl-fmw.0.size=256M " \
    "-device cxl-rp,id=rp0,bus=cxl.0,chassis=0,slot=0 "

#define QEMU_T2_CFMWS_TWO_TARGETS \
    "-machine q35,cxl=on -m 128M " \
    "-device pxb-cxl,id=cxl.0,bus=pcie.0,bus_nr=52 " \
    "-device pxb-cxl,id=cxl.1,bus=pcie.0,bus_nr=53 " \
    "-M cxl-fmw.0.targets.0=cxl.0,cxl-fmw.0.targets.1=cxl.1," \
    "cxl-fmw.0.size=256M " \
    "-device cxl-rp,id=rp0,bus=cxl.0,chassis=0,slot=0 "

#define QEMU_T2_CFMWS_512M \
    "-machine q35,cxl=on -m 128M " \
    "-device pxb-cxl,id=cxl.0,bus=pcie.0,bus_nr=52 " \
    "-M cxl-fmw.0.targets.0=cxl.0,cxl-fmw.0.size=512M " \
    "-device cxl-rp,id=rp0,bus=cxl.0,chassis=0,slot=0 "

#define QEMU_T2_CFMWS_SWITCH \
    "-machine q35,cxl=on -m 128M " \
    "-device pxb-cxl,id=cxl.0,bus=pcie.0,bus_nr=52 " \
    "-M cxl-fmw.0.targets.0=cxl.0,cxl-fmw.0.size=256M " \
    "-device cxl-rp,id=rp0,bus=cxl.0,addr=0.0,chassis=0,slot=0 " \
    "-device cxl-upstream,id=us0,bus=rp0,addr=0.0 " \
    "-device cxl-downstream,id=swport0,bus=us0,addr=0.0,port=0," \
    "chassis=0,slot=4 "

#define Q35_PCIE_MCFG_BASE UINT64_C(0xb0000000)
#define Q35_CXL_HOST_REG_BASE UINT64_C(0x100000000)
#define Q35_CXL_HB_CACHE_MEM_BASE (Q35_CXL_HOST_REG_BASE + 0x1000)
#define Q35_CFMWS_BASE UINT64_C(0x110000000)
#define T2_DVSEC_DEVFN (4U << 3)
#define T2_SENTINEL_DPA (80 * MiB)
#define T2_SERVER_READ_VALUE UINT64_C(0x1122334455667788)
#define T2_CLIENT_WRITE_VALUE UINT64_C(0x8877665544332211)
#define T2_SWITCH_COMPONENT_BAR UINT64_C(0xd0000000)

/* Dual ports on first pxb */
#define QEMU_2RP \
    "-device cxl-rp,id=rp0,bus=cxl.0,chassis=0,slot=0 " \
    "-device cxl-rp,id=rp1,bus=cxl.0,chassis=0,slot=1 "

/* Dual ports on each of the pxb instances */
#define QEMU_4RP \
    "-device cxl-rp,id=rp0,bus=cxl.0,chassis=0,slot=0 " \
    "-device cxl-rp,id=rp1,bus=cxl.0,chassis=0,slot=1 " \
    "-device cxl-rp,id=rp2,bus=cxl.1,chassis=0,slot=2 " \
    "-device cxl-rp,id=rp3,bus=cxl.1,chassis=0,slot=3 "

#define QEMU_T3D_DEPRECATED \
    "-object memory-backend-file,id=cxl-mem0,mem-path=%s,size=256M " \
    "-object memory-backend-file,id=lsa0,mem-path=%s,size=256M " \
    "-device cxl-type3,bus=rp0,memdev=cxl-mem0,lsa=lsa0,id=cxl-pmem0 "

#define QEMU_T3D_PMEM \
    "-object memory-backend-file,id=cxl-mem0,mem-path=%s,size=256M " \
    "-object memory-backend-file,id=lsa0,mem-path=%s,size=256M " \
    "-device cxl-type3,bus=rp0,persistent-memdev=cxl-mem0,lsa=lsa0,id=pmem0 "

#define QEMU_T3D_VMEM \
    "-object memory-backend-ram,id=cxl-mem0,size=256M " \
    "-device cxl-type3,bus=rp0,volatile-memdev=cxl-mem0,id=mem0 "

#define QEMU_T3D_VMEM_LSA \
    "-object memory-backend-ram,id=cxl-mem0,size=256M " \
    "-object memory-backend-file,id=lsa0,mem-path=%s,size=256M " \
    "-device cxl-type3,bus=rp0,volatile-memdev=cxl-mem0,lsa=lsa0,id=mem0 "

#define QEMU_2T3D \
    "-object memory-backend-file,id=cxl-mem0,mem-path=%s,size=256M " \
    "-object memory-backend-file,id=lsa0,mem-path=%s,size=256M " \
    "-device cxl-type3,bus=rp0,persistent-memdev=cxl-mem0,lsa=lsa0,id=pmem0 " \
    "-object memory-backend-file,id=cxl-mem1,mem-path=%s,size=256M " \
    "-object memory-backend-file,id=lsa1,mem-path=%s,size=256M " \
    "-device cxl-type3,bus=rp1,persistent-memdev=cxl-mem1,lsa=lsa1,id=pmem1 "

#define QEMU_4T3D \
    "-object memory-backend-file,id=cxl-mem0,mem-path=%s,size=256M " \
    "-object memory-backend-file,id=lsa0,mem-path=%s,size=256M " \
    "-device cxl-type3,bus=rp0,persistent-memdev=cxl-mem0,lsa=lsa0,id=pmem0 " \
    "-object memory-backend-file,id=cxl-mem1,mem-path=%s,size=256M " \
    "-object memory-backend-file,id=lsa1,mem-path=%s,size=256M " \
    "-device cxl-type3,bus=rp1,persistent-memdev=cxl-mem1,lsa=lsa1,id=pmem1 " \
    "-object memory-backend-file,id=cxl-mem2,mem-path=%s,size=256M " \
    "-object memory-backend-file,id=lsa2,mem-path=%s,size=256M " \
    "-device cxl-type3,bus=rp2,persistent-memdev=cxl-mem2,lsa=lsa2,id=pmem2 " \
    "-object memory-backend-file,id=cxl-mem3,mem-path=%s,size=256M " \
    "-object memory-backend-file,id=lsa3,mem-path=%s,size=256M " \
    "-device cxl-type3,bus=rp3,persistent-memdev=cxl-mem3,lsa=lsa3,id=pmem3 "

#ifdef CONFIG_POSIX
static const uint8_t t2_hello_golden[] = {
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

typedef struct T2FakeServer {
    int listen_fd;
    uint16_t port;
    GThread *thread;
    gint stop;
    bool accepted;
    bool hello_valid;
    bool ack_sent;
    uint64_t ack_capacity;
    uint64_t ack_latency;
    uint64_t ack_request_id;
    bool bad_crc;
    bool memory_protocol_valid;
    bool write_committed;
    bool write_response_sent;
    unsigned memory_requests;
    unsigned read_requests;
    unsigned write_requests;
    uint64_t last_read_dpa;
    uint64_t last_write_dpa;
    uint64_t last_write_value;
    uint64_t read_value;
    uint64_t server_sequence;
} T2FakeServer;

static bool t2_read_exact(int fd, uint8_t *bytes, size_t length)
{
    size_t offset = 0;

    while (offset < length) {
        ssize_t received = recv(fd, bytes + offset, length - offset, 0);

        if (received > 0) {
            offset += received;
            continue;
        }
        if (received < 0 && errno == EINTR) {
            continue;
        }
        return false;
    }
    return true;
}

static bool t2_write_exact(int fd, const uint8_t *bytes, size_t length)
{
    size_t offset = 0;

    while (offset < length) {
        ssize_t sent = send(fd, bytes + offset, length - offset,
                            MSG_NOSIGNAL);

        if (sent > 0) {
            offset += sent;
            continue;
        }
        if (sent < 0 && errno == EINTR) {
            continue;
        }
        return false;
    }
    return true;
}

static void t2_encode_ack(const T2FakeServer *server, uint8_t ack[88])
{
    uint32_t checksum;
    size_t i;

    memset(ack, 0, 88);
    memcpy(ack, "SLT2", 4);
    stw_le_p(ack + 4, 1);
    stw_le_p(ack + 6, 2);
    stl_le_p(ack + 8, 88);
    stq_le_p(ack + 16, server->ack_request_id);
    stq_le_p(ack + 24, 1);
    stl_le_p(ack + 40, 0);
    stl_le_p(ack + 44, 128);
    for (i = 0; i < 16; i++) {
        ack[48 + i] = 0xa0 + i;
    }
    stq_le_p(ack + 64, server->ack_capacity);
    stq_le_p(ack + 72, server->ack_latency);
    checksum = crc32c(0xffffffffU, ack, 88);
    stl_le_p(ack + 32, checksum);
    if (server->bad_crc) {
        ack[32] ^= 1;
    }
}

static bool t2_frame_crc_valid(const uint8_t *frame, size_t length)
{
    uint8_t copy[128];
    uint32_t received;

    if (length > sizeof(copy)) {
        return false;
    }
    memcpy(copy, frame, length);
    received = ldl_le_p(copy + 32);
    stl_le_p(copy + 32, 0);
    return received == crc32c(0xffffffffU, copy, length);
}

static bool t2_decode_memory_request(T2FakeServer *server,
                                     const uint8_t frame[128],
                                     uint16_t *type,
                                     uint64_t *request_id,
                                     uint64_t *client_id,
                                     uint32_t *length,
                                     uint64_t *dpa,
                                     uint64_t *value)
{
    *type = lduw_le_p(frame + 6);
    *request_id = ldq_le_p(frame + 16);
    *client_id = ldq_le_p(frame + 24);
    *length = ldl_le_p(frame + 40);
    *dpa = ldq_le_p(frame + 48);
    *value = ldq_le_p(frame + 64);

    return memcmp(frame, "SLT2", 4) == 0 &&
           lduw_le_p(frame + 4) == 1 &&
           (*type == 3 || *type == 4) &&
           ldl_le_p(frame + 8) == 128 &&
           ldl_le_p(frame + 12) == 0 &&
           *request_id >= 2 &&
           *client_id == 1 &&
           ldl_le_p(frame + 36) == 0 &&
           *length == 8 &&
           ldl_le_p(frame + 44) == 0 &&
           *dpa == T2_SENTINEL_DPA &&
           t2_frame_crc_valid(frame, 128) &&
           server->ack_sent;
}

static void t2_encode_memory_response(T2FakeServer *server,
                                      uint8_t response[128],
                                      uint16_t request_type,
                                      uint64_t request_id,
                                      uint64_t client_id)
{
    uint32_t checksum;

    memset(response, 0, 128);
    memcpy(response, "SLT2", 4);
    stw_le_p(response + 4, 1);
    stw_le_p(response + 6, 5);
    stl_le_p(response + 8, 128);
    stq_le_p(response + 16, request_id);
    stq_le_p(response + 24, client_id);
    stl_le_p(response + 40, 0);
    stl_le_p(response + 44, request_type == 3 ? 8 : 0);
    stq_le_p(response + 48, ++server->server_sequence);
    stq_le_p(response + 56, server->ack_latency);
    if (request_type == 3) {
        stq_le_p(response + 64, server->read_value);
    }
    checksum = crc32c(0xffffffffU, response, 128);
    stl_le_p(response + 32, checksum);
}

static gpointer t2_fake_server_thread(gpointer opaque)
{
    T2FakeServer *server = opaque;
    struct pollfd pollfd = {
        .fd = server->listen_fd,
        .events = POLLIN,
    };
    int client_fd = -1;
    uint8_t hello[sizeof(t2_hello_golden)];
    uint8_t ack[88];

    while (!g_atomic_int_get(&server->stop)) {
        int ready = poll(&pollfd, 1, 50);

        if (ready > 0) {
            client_fd = accept(server->listen_fd, NULL, NULL);
            break;
        }
        if (ready < 0 && errno != EINTR) {
            return NULL;
        }
    }
    if (client_fd < 0) {
        return NULL;
    }

    server->accepted = true;
    server->hello_valid =
        t2_read_exact(client_fd, hello, sizeof(hello)) &&
        memcmp(hello, t2_hello_golden, sizeof(hello)) == 0;
    t2_encode_ack(server, ack);
    server->ack_sent = t2_write_exact(client_fd, ack, sizeof(ack));

    while (!g_atomic_int_get(&server->stop)) {
        struct pollfd client_pollfd = {
            .fd = client_fd,
            .events = POLLIN,
        };
        uint8_t request[128];
        uint8_t response[128];
        uint16_t type;
        uint64_t request_id;
        uint64_t client_id;
        uint64_t dpa;
        uint64_t value;
        uint32_t length;
        int ready = poll(&client_pollfd, 1, 50);

        if (ready > 0) {
            if (!t2_read_exact(client_fd, request, sizeof(request))) {
                break;
            }
            server->memory_protocol_valid =
                t2_decode_memory_request(server, request, &type,
                                         &request_id, &client_id,
                                         &length, &dpa, &value);
            if (!server->memory_protocol_valid) {
                break;
            }
            server->memory_requests++;
            if (type == 3) {
                server->read_requests++;
                server->last_read_dpa = dpa;
            } else {
                server->write_requests++;
                server->last_write_dpa = dpa;
                server->last_write_value = value;
                server->write_committed = true;
            }
            t2_encode_memory_response(server, response, type,
                                      request_id, client_id);
            if (!t2_write_exact(client_fd, response, sizeof(response))) {
                break;
            }
            if (type == 4) {
                server->write_response_sent = true;
            }
        } else if (ready < 0 && errno != EINTR) {
            break;
        }
    }
    close(client_fd);
    return NULL;
}

static T2FakeServer *t2_fake_server_start(uint64_t capacity,
                                          uint64_t latency,
                                          uint64_t request_id,
                                          bool bad_crc)
{
    T2FakeServer *server = g_new0(T2FakeServer, 1);
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
    };
    socklen_t address_length = sizeof(address);
    int reuse = 1;

    server->ack_capacity = capacity;
    server->ack_latency = latency;
    server->ack_request_id = request_id;
    server->bad_crc = bad_crc;
    server->read_value = T2_SERVER_READ_VALUE;
    server->listen_fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    g_assert_cmpint(server->listen_fd, >=, 0);
    g_assert_cmpint(setsockopt(server->listen_fd, SOL_SOCKET, SO_REUSEADDR,
                              &reuse, sizeof(reuse)), ==, 0);
    g_assert_cmpint(bind(server->listen_fd, (struct sockaddr *)&address,
                         sizeof(address)), ==, 0);
    g_assert_cmpint(getsockname(server->listen_fd,
                               (struct sockaddr *)&address,
                               &address_length), ==, 0);
    server->port = ntohs(address.sin_port);
    g_assert_cmpuint(server->port, >, 0);
    g_assert_cmpint(listen(server->listen_fd, 1), ==, 0);
    server->thread = g_thread_new("cxl-t2-fake-server",
                                  t2_fake_server_thread, server);
    return server;
}

static void t2_fake_server_stop(T2FakeServer *server)
{
    g_atomic_int_set(&server->stop, 1);
    shutdown(server->listen_fd, SHUT_RDWR);
    close(server->listen_fd);
    g_thread_join(server->thread);
}

static QDict *t2_device_add(QTestState *qts, const char *bus,
                            uint16_t port, const char *event_path)
{
    QDict *arguments = qdict_new();

    qdict_put_str(arguments, "driver", "cxl-type2");
    qdict_put_str(arguments, "id", "t2");
    qdict_put_str(arguments, "bus", bus);
    qdict_put_int(arguments, "gpu-mode", 0);
    qdict_put_bool(arguments, "coherency-enabled", false);
    qdict_put_int(arguments, "cache-size", 128 * MiB);
    qdict_put_int(arguments, "mem-size", 256 * MiB);
    qdict_put_bool(arguments, "sync-type2-wire", true);
    qdict_put_int(arguments, "type2-wire-version", 1);
    qdict_put_str(arguments, "slugarch-event-log", event_path);
    qdict_put_str(arguments, "cxlmemsim-addr", "127.0.0.1");
    qdict_put_int(arguments, "cxlmemsim-port", port);
    return qtest_qmp(qts,
                     "{'execute': 'device_add', 'arguments': %p}",
                     arguments);
}

static void t2_assert_observability(QTestState *qts)
{
    static const char *zero_counters[] = {
        "slugarch-completed-reads",
        "slugarch-completed-writes",
        "slugarch-read-bytes",
        "slugarch-written-bytes",
        "slugarch-failed-requests",
        "slugarch-timed-out-requests",
        "slugarch-partial-io-failures",
        "slugarch-mismatched-responses",
        "slugarch-direct-cfmws",
        "slugarch-bar4-overlay",
        "slugarch-bulk-overlay",
        "slugarch-coherent-pool",
        "slugarch-local-shadow",
        "slugarch-local-cache",
        "slugarch-delay-events",
        "slugarch-delay-undershoots",
    };
    QDict *response;
    size_t i;

    response = qtest_qmp(
        qts,
        "{'execute':'qom-set','arguments':{"
        "'path':'/machine/peripheral/t2',"
        "'property':'slugarch-phase-id','value':'phase:test'}}");
    g_assert_false(qdict_haskey(response, "error"));
    qobject_unref(response);

    response = qtest_qmp(
        qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/machine/peripheral/t2',"
        "'property':'slugarch-phase-id'}}");
    g_assert_cmpstr(qdict_get_str(response, "return"), ==, "phase:test");
    qobject_unref(response);

    response = qtest_qmp(
        qts,
        "{'execute':'qom-set','arguments':{"
        "'path':'/machine/peripheral/t2',"
        "'property':'slugarch-phase-id','value':'bad phase'}}");
    g_assert_true(qdict_haskey(response, "error"));
    qobject_unref(response);

    response = qtest_qmp(
        qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/machine/peripheral/t2',"
        "'property':'slugarch-client-id'}}");
    g_assert_cmpuint(qdict_get_int(response, "return"), ==, 1);
    qobject_unref(response);

    for (i = 0; i < G_N_ELEMENTS(zero_counters); i++) {
        response = qtest_qmp(
            qts,
            "{'execute':'qom-get','arguments':{"
            "'path':'/machine/peripheral/t2','property':%s}}",
            zero_counters[i]);
        g_assert_cmpuint(qdict_get_int(response, "return"), ==, 0);
        qobject_unref(response);
    }
}

static void t2_sync_handshake_case(uint64_t capacity,
                                   uint64_t latency,
                                   uint64_t request_id,
                                   bool bad_crc,
                                   bool expect_success,
                                   const char *expected_error)
{
    g_autofree char *run_dir = NULL;
    g_autofree char *event_path = NULL;
    g_autofree char *error_description = NULL;
    T2FakeServer *server;
    QTestState *qts;
    QDict *response;
    bool success;

    run_dir = g_dir_make_tmp("cxl-t2-sync-XXXXXX", NULL);
    g_assert_nonnull(run_dir);
    event_path = g_build_filename(run_dir, "qemu-events.jsonl", NULL);
    server = t2_fake_server_start(capacity, latency, request_id, bad_crc);
    qts = qtest_init(QEMU_T2_SYNC_BASE);
    response = t2_device_add(qts, "rp0", server->port, event_path);
    success = !qdict_haskey(response, "error");
    if (!success) {
        error_description = g_strdup(qdict_get_str(
            qdict_get_qdict(response, "error"), "desc"));
    }
    qobject_unref(response);

    if (success) {
        t2_assert_observability(qts);
    }
    t2_fake_server_stop(server);
    qtest_quit(qts);

    g_assert_true(server->accepted);
    g_assert_true(server->hello_valid);
    g_assert_true(server->ack_sent);
    if (expect_success) {
        g_assert_true(success);
        g_assert_true(g_file_test(event_path, G_FILE_TEST_IS_REGULAR));
        {
            g_autofree char *events = NULL;

            g_assert_true(g_file_get_contents(event_path, &events,
                                              NULL, NULL));
            g_assert_nonnull(strstr(events, "\"event\":\"handshake\""));
            g_assert_nonnull(strstr(events,
                                    "\"phase_id\":\"phase:test\""));
        }
    } else {
        g_assert_false(success);
        g_assert_nonnull(error_description);
        g_assert_nonnull(strstr(error_description,
                                "SlugArch Type-2 handshake failed:"));
        g_assert_nonnull(strstr(error_description, expected_error));
    }

    unlink(event_path);
    rmdir(run_dir);
    g_free(server);
}

static void cxl_t2_sync_handshake(void)
{
    t2_sync_handshake_case(256 * MiB, 400, 1, false, true, NULL);
}

static void cxl_t2_sync_bad_capacity(void)
{
    t2_sync_handshake_case(64 * MiB, 400, 1, false, false, "capacity");
}

static void cxl_t2_sync_bad_latency(void)
{
    t2_sync_handshake_case(256 * MiB, 1000001, 1, false, false,
                           "latency");
}

static void cxl_t2_sync_bad_request_id(void)
{
    t2_sync_handshake_case(256 * MiB, 400, 2, false, false,
                           "request ID");
}

static void cxl_t2_sync_bad_crc(void)
{
    t2_sync_handshake_case(256 * MiB, 400, 1, true, false, "CRC32C");
}

static void t2_program_host_decoder(QTestState *qts)
{
    const uint64_t registers = Q35_CXL_HB_CACHE_MEM_BASE;
    uint32_t control;

    qtest_writel(qts, registers + A_CXL_HDM_DECODER0_BASE_LO,
                 Q35_CFMWS_BASE & 0xf0000000U);
    qtest_writel(qts, registers + A_CXL_HDM_DECODER0_BASE_HI,
                 Q35_CFMWS_BASE >> 32);
    qtest_writel(qts, registers + A_CXL_HDM_DECODER0_SIZE_LO,
                 256 * MiB);
    qtest_writel(qts, registers + A_CXL_HDM_DECODER0_SIZE_HI, 0);
    qtest_writel(qts, registers + A_CXL_HDM_DECODER0_TARGET_LIST_LO, 0);
    qtest_writel(qts, registers + A_CXL_HDM_DECODER0_TARGET_LIST_HI, 0);
    qtest_writel(qts, registers + A_CXL_HDM_DECODER_GLOBAL_CONTROL,
                 1U << 1);
    qtest_writel(qts, registers + A_CXL_HDM_DECODER0_CTRL, 1U << 9);
    control = qtest_readl(qts, registers + A_CXL_HDM_DECODER0_CTRL);
    g_assert_cmphex(control & (1U << 10), ==, 1U << 10);
}

static uint32_t t2_pci_config_address(uint8_t bus, uint8_t devfn,
                                      uint8_t offset)
{
    return (1U << 31) | ((uint32_t)bus << 16) |
           ((uint32_t)devfn << 8) | (offset & ~3U);
}

static uint16_t t2_pci_config_readw(QTestState *qts, uint8_t bus,
                                    uint8_t devfn, uint8_t offset)
{
    qtest_outl(qts, 0xcf8, t2_pci_config_address(bus, devfn, offset));
    return qtest_inw(qts, 0xcfc + (offset & 3));
}

static uint32_t t2_pci_config_readl(QTestState *qts, uint8_t bus,
                                    uint8_t devfn, uint8_t offset)
{
    qtest_outl(qts, 0xcf8, t2_pci_config_address(bus, devfn, offset));
    return qtest_inl(qts, 0xcfc);
}

static void t2_pci_config_writew(QTestState *qts, uint8_t bus,
                                 uint8_t devfn, uint8_t offset,
                                 uint16_t value)
{
    qtest_outl(qts, 0xcf8, t2_pci_config_address(bus, devfn, offset));
    qtest_outw(qts, 0xcfc + (offset & 3), value);
}

static void t2_pci_config_writel(QTestState *qts, uint8_t bus,
                                 uint8_t devfn, uint8_t offset,
                                 uint32_t value)
{
    qtest_outl(qts, 0xcf8, t2_pci_config_address(bus, devfn, offset));
    qtest_outl(qts, 0xcfc, value);
}

static void t2_enable_bridge_window(QTestState *qts, uint8_t bus,
                                    uint8_t devfn)
{
    uint16_t command = t2_pci_config_readw(qts, bus, devfn,
                                           PCI_COMMAND);

    t2_pci_config_writel(qts, bus, devfn, PCI_MEMORY_BASE,
                         0xd000d000U);
    t2_pci_config_writew(qts, bus, devfn, PCI_COMMAND,
                         command | PCI_COMMAND_MEMORY);
}

static void t2_program_switch_decoder(QTestState *qts)
{
    const uint64_t registers = T2_SWITCH_COMPONENT_BAR + 0x1000;
    uint32_t control;

    t2_pci_config_writel(qts, 52, 0, PCI_PRIMARY_BUS,
                         52U | (53U << 8) | (55U << 16));
    g_assert_cmphex(t2_pci_config_readw(qts, 53, 0, PCI_VENDOR_ID),
                    !=, 0xffff);
    t2_pci_config_writel(qts, 53, 0, PCI_PRIMARY_BUS,
                         53U | (54U << 8) | (55U << 16));
    g_assert_cmphex(t2_pci_config_readw(qts, 54, 0, PCI_VENDOR_ID),
                    !=, 0xffff);
    t2_pci_config_writel(qts, 54, 0, PCI_PRIMARY_BUS,
                         54U | (55U << 8) | (55U << 16));
    g_assert_cmphex(t2_pci_config_readw(qts, 55, 0, PCI_VENDOR_ID),
                    ==, 0x8086);

    t2_enable_bridge_window(qts, 52, 0);
    t2_enable_bridge_window(qts, 53, 0);
    t2_enable_bridge_window(qts, 54, 0);
    t2_pci_config_writel(qts, 53, 0, PCI_BASE_ADDRESS_0,
                         T2_SWITCH_COMPONENT_BAR);
    t2_pci_config_writel(qts, 53, 0, PCI_BASE_ADDRESS_1, 0);
    g_assert_cmphex(
        t2_pci_config_readl(qts, 53, 0, PCI_BASE_ADDRESS_0) &
        PCI_BASE_ADDRESS_MEM_MASK,
        ==, T2_SWITCH_COMPONENT_BAR);

    qtest_writel(qts, registers + A_CXL_HDM_DECODER0_BASE_LO,
                 Q35_CFMWS_BASE & 0xf0000000U);
    qtest_writel(qts, registers + A_CXL_HDM_DECODER0_BASE_HI,
                 Q35_CFMWS_BASE >> 32);
    qtest_writel(qts, registers + A_CXL_HDM_DECODER0_SIZE_LO,
                 256 * MiB);
    qtest_writel(qts, registers + A_CXL_HDM_DECODER0_SIZE_HI, 0);
    qtest_writel(qts, registers + A_CXL_HDM_DECODER0_TARGET_LIST_LO, 0);
    qtest_writel(qts, registers + A_CXL_HDM_DECODER0_TARGET_LIST_HI, 0);
    qtest_writel(qts, registers + A_CXL_HDM_DECODER0_CTRL, 1U << 9);
    control = qtest_readl(qts, registers + A_CXL_HDM_DECODER0_CTRL);
    g_assert_cmphex(control & (1U << 10), ==, 1U << 10);
}

static uint64_t t2_qom_counter(QTestState *qts, const char *property)
{
    QDict *response;
    uint64_t value;

    response = qtest_qmp(
        qts,
        "{'execute':'qom-get','arguments':{"
        "'path':'/machine/peripheral/t2','property':%s}}",
        property);
    g_assert_false(qdict_haskey(response, "error"));
    value = qdict_get_int(response, "return");
    qobject_unref(response);
    return value;
}

static void cxl_t2_direct_cfmws(void)
{
    g_autofree char *run_dir = NULL;
    g_autofree char *event_path = NULL;
    T2FakeServer *server;
    QTestState *qts;
    QDict *response;
    uint64_t read_value;

    run_dir = g_dir_make_tmp("cxl-t2-cfmws-XXXXXX", NULL);
    g_assert_nonnull(run_dir);
    event_path = g_build_filename(run_dir, "qemu-events.jsonl", NULL);
    server = t2_fake_server_start(256 * MiB, 400, 1, false);
    qts = qtest_init(QEMU_T2_CFMWS_BASE);
    response = t2_device_add(qts, "rp0", server->port, event_path);
    g_assert_false(qdict_haskey(response, "error"));
    qobject_unref(response);

    t2_program_host_decoder(qts);
    read_value = qtest_readq(qts, Q35_CFMWS_BASE + T2_SENTINEL_DPA);
    g_assert_cmphex(read_value, ==, T2_SERVER_READ_VALUE);
    qtest_writeq(qts, Q35_CFMWS_BASE + T2_SENTINEL_DPA,
                 T2_CLIENT_WRITE_VALUE);

    g_assert_cmpuint(t2_qom_counter(qts, "slugarch-completed-reads"),
                     ==, 1);
    g_assert_cmpuint(t2_qom_counter(qts, "slugarch-completed-writes"),
                     ==, 1);
    g_assert_cmpuint(t2_qom_counter(qts, "slugarch-read-bytes"),
                     ==, 8);
    g_assert_cmpuint(t2_qom_counter(qts, "slugarch-written-bytes"),
                     ==, 8);
    g_assert_cmpuint(t2_qom_counter(qts, "slugarch-direct-cfmws"),
                     ==, 2);
    g_assert_cmpuint(t2_qom_counter(qts, "slugarch-bar4-overlay"),
                     ==, 0);
    g_assert_cmpuint(t2_qom_counter(qts, "slugarch-local-shadow"),
                     ==, 0);
    g_assert_cmpuint(t2_qom_counter(qts, "slugarch-local-cache"),
                     ==, 0);

    t2_fake_server_stop(server);
    qtest_quit(qts);
    g_assert_true(server->memory_protocol_valid);
    g_assert_cmpuint(server->memory_requests, ==, 2);
    g_assert_cmpuint(server->read_requests, ==, 1);
    g_assert_cmpuint(server->write_requests, ==, 1);
    g_assert_cmphex(server->last_read_dpa, ==, T2_SENTINEL_DPA);
    g_assert_cmphex(server->last_write_dpa, ==, T2_SENTINEL_DPA);
    g_assert_cmphex(server->last_read_dpa, !=,
                    Q35_CFMWS_BASE + T2_SENTINEL_DPA);
    g_assert_cmphex(server->last_write_dpa, !=,
                    Q35_CFMWS_BASE + T2_SENTINEL_DPA);
    g_assert_cmphex(server->last_write_value, ==,
                    T2_CLIENT_WRITE_VALUE);
    g_assert_true(server->write_committed);
    g_assert_true(server->write_response_sent);

    unlink(event_path);
    rmdir(run_dir);
    g_free(server);
}

static void t2_rejected_cfmws_case(const char *command_line,
                                   const char *endpoint_bus,
                                   void (*topology_setup)(QTestState *))
{
    g_autofree char *run_dir = NULL;
    g_autofree char *event_path = NULL;
    T2FakeServer *server;
    QTestState *qts;
    QDict *response;

    run_dir = g_dir_make_tmp("cxl-t2-rejected-XXXXXX", NULL);
    g_assert_nonnull(run_dir);
    event_path = g_build_filename(run_dir, "qemu-events.jsonl", NULL);
    server = t2_fake_server_start(256 * MiB, 400, 1, false);
    qts = qtest_init(command_line);
    response = t2_device_add(qts, endpoint_bus, server->port, event_path);
    g_assert_false(qdict_haskey(response, "error"));
    qobject_unref(response);
    if (topology_setup) {
        topology_setup(qts);
    }

    g_assert_cmphex(qtest_readq(qts,
                                Q35_CFMWS_BASE + T2_SENTINEL_DPA),
                    ==, 0);
    qtest_writeq(qts, Q35_CFMWS_BASE + T2_SENTINEL_DPA,
                 T2_CLIENT_WRITE_VALUE);
    g_assert_cmpuint(t2_qom_counter(qts, "slugarch-direct-cfmws"),
                     ==, 0);
    g_assert_cmpuint(t2_qom_counter(qts, "slugarch-completed-reads"),
                     ==, 0);
    g_assert_cmpuint(t2_qom_counter(qts, "slugarch-completed-writes"),
                     ==, 0);

    t2_fake_server_stop(server);
    qtest_quit(qts);
    g_assert_cmpuint(server->memory_requests, ==, 0);

    unlink(event_path);
    rmdir(run_dir);
    g_free(server);
}

static void cxl_t2_cfmws_reject_two_targets(void)
{
    t2_rejected_cfmws_case(QEMU_T2_CFMWS_TWO_TARGETS, "rp0", NULL);
}

static void cxl_t2_cfmws_reject_512m(void)
{
    t2_rejected_cfmws_case(QEMU_T2_CFMWS_512M, "rp0", NULL);
}

static void cxl_t2_cfmws_reject_switch(void)
{
    t2_rejected_cfmws_case(QEMU_T2_CFMWS_SWITCH, "swport0",
                           t2_program_switch_decoder);
}

static uint16_t t2_find_device_dvsec(QTestState *qts,
                                     uint64_t config_base)
{
    uint16_t offset = 0x100;
    unsigned hops;

    for (hops = 0; hops < 256 && offset; hops++) {
        uint32_t header = qtest_readl(qts, config_base + offset);

        if (header == 0 || header == UINT32_MAX) {
            break;
        }
        if (PCI_EXT_CAP_ID(header) == PCI_EXT_CAP_ID_DVSEC &&
            qtest_readw(qts, config_base + offset + PCI_DVSEC_HEADER2) ==
            0) {
            return offset;
        }
        offset = PCI_EXT_CAP_NEXT(header);
    }

    g_assert_not_reached();
}

static void cxl_t2_dvsec(void)
{
    g_autofree char *run_dir = NULL;
    g_autofree char *event_path = NULL;
    g_autoptr(GString) cmdline = g_string_new(NULL);
    T2FakeServer *server;
    QTestState *qts;
    uint64_t config_base =
        Q35_PCIE_MCFG_BASE + ((uint64_t)T2_DVSEC_DEVFN << 12);
    uint16_t dvsec;
    uint16_t cap;
    uint16_t cap2;
    uint32_t range1_size_hi;
    uint32_t range1_size_lo;

    run_dir = g_dir_make_tmp("cxl-t2-dvsec-XXXXXX", NULL);
    g_assert_nonnull(run_dir);
    event_path = g_build_filename(run_dir, "qemu-events.jsonl", NULL);
    server = t2_fake_server_start(256 * MiB, 400, 1, false);
    g_string_printf(
        cmdline,
        "-machine q35,cxl=on "
        "-device cxl-type2,id=t2,bus=pcie.0,addr=4.0,gpu-mode=0,"
        "coherency-enabled=false,cache-size=128M,mem-size=256M,"
        "sync-type2-wire=on,type2-wire-version=1,"
        "slugarch-event-log=%s,cxlmemsim-addr=127.0.0.1,"
        "cxlmemsim-port=%u",
        event_path, server->port);
    qts = qtest_init(cmdline->str);

    qtest_outl(qts, 0xcf8, (1U << 31) | 0x64);
    qtest_outl(qts, 0xcfc, 0);
    qtest_outl(qts, 0xcf8, (1U << 31) | 0x60);
    qtest_outl(qts, 0xcfc, Q35_PCIE_MCFG_BASE | 1);
    g_assert_cmphex(qtest_readw(qts, config_base), ==, 0x8086);

    dvsec = t2_find_device_dvsec(qts, config_base);
    cap = qtest_readw(qts, config_base + dvsec + 0x0a);
    cap2 = qtest_readw(qts, config_base + dvsec + 0x16);
    range1_size_hi = qtest_readl(qts, config_base + dvsec + 0x18);
    range1_size_lo = qtest_readl(qts, config_base + dvsec + 0x1c);

    g_assert_cmphex(cap & 0xf, ==, 0xf);
    g_assert_cmpuint((cap >> 4) & 0x3, ==, 1);
    g_assert_cmpuint(cap2 & 0xf, ==, 2);
    g_assert_cmpuint((cap2 >> 8) & 0xff, ==, 128);
    g_assert_cmpuint(range1_size_lo & 0x3, ==, 0x3);
    g_assert_cmpuint(((uint64_t)range1_size_hi << 32) |
                     (range1_size_lo & 0xf0000000U), ==, 256 * MiB);
    g_assert_cmphex(qtest_readl(qts, config_base + dvsec + 0x20),
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, config_base + dvsec + 0x24),
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, config_base + dvsec + 0x28),
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, config_base + dvsec + 0x2c),
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, config_base + dvsec + 0x30),
                    ==, 0);
    g_assert_cmphex(qtest_readl(qts, config_base + dvsec + 0x34),
                    ==, 0);

    t2_fake_server_stop(server);
    qtest_quit(qts);
    unlink(event_path);
    rmdir(run_dir);
    g_free(server);
}

static void t2_bad_cache_size_case(uint64_t cache_size)
{
    QTestState *qts = qtest_init(QEMU_T2_SYNC_BASE);
    QDict *arguments = qdict_new();
    QDict *response;
    const char *description;

    qdict_put_str(arguments, "driver", "cxl-type2");
    qdict_put_str(arguments, "id", "t2-bad-cache");
    qdict_put_str(arguments, "bus", "rp0");
    qdict_put_int(arguments, "gpu-mode", 0);
    qdict_put_bool(arguments, "coherency-enabled", false);
    qdict_put_int(arguments, "cache-size", cache_size);
    qdict_put_int(arguments, "mem-size", 256 * MiB);
    qdict_put_str(arguments, "cxlmemsim-addr", "127.0.0.1");
    qdict_put_int(arguments, "cxlmemsim-port", 1);
    response = qtest_qmp(
        qts, "{'execute':'device_add','arguments':%p}", arguments);

    g_assert_true(qdict_haskey(response, "error"));
    description = qdict_get_str(qdict_get_qdict(response, "error"),
                                "desc");
    g_assert_nonnull(strstr(
        description,
        "cache-size must be an integral value from 1 MiB through 255 MiB"));
    qobject_unref(response);
    qtest_quit(qts);
}

static void cxl_t2_dvsec_bad_fractional_cache(void)
{
    t2_bad_cache_size_case(128 * MiB + 1);
}

static void cxl_t2_dvsec_bad_oversized_cache(void)
{
    t2_bad_cache_size_case(256 * MiB);
}
#endif /* CONFIG_POSIX */

static void cxl_basic_hb(void)
{
    qtest_start("-machine q35,cxl=on");
    qtest_end();
}

static void cxl_basic_pxb(void)
{
    qtest_start("-machine q35,cxl=on -device pxb-cxl,bus=pcie.0");
    qtest_end();
}

static void cxl_pxb_with_window(void)
{
    qtest_start(QEMU_PXB_CMD);
    qtest_end();
}

static void cxl_2pxb_with_window(void)
{
    qtest_start(QEMU_2PXB_CMD);
    qtest_end();
}

static void cxl_root_port(void)
{
    qtest_start(QEMU_PXB_CMD QEMU_RP);
    qtest_end();
}

static void cxl_2root_port(void)
{
    qtest_start(QEMU_PXB_CMD QEMU_2RP);
    qtest_end();
}

#ifdef CONFIG_POSIX
static void cxl_t3d_deprecated(void)
{
    g_autoptr(GString) cmdline = g_string_new(NULL);
    g_autofree const char *tmpfs = NULL;

    tmpfs = g_dir_make_tmp("cxl-test-XXXXXX", NULL);

    g_string_printf(cmdline, QEMU_PXB_CMD QEMU_RP QEMU_T3D_DEPRECATED,
                    tmpfs, tmpfs);

    qtest_start(cmdline->str);
    qtest_end();
    rmdir(tmpfs);
}

static void cxl_t3d_persistent(void)
{
    g_autoptr(GString) cmdline = g_string_new(NULL);
    g_autofree const char *tmpfs = NULL;

    tmpfs = g_dir_make_tmp("cxl-test-XXXXXX", NULL);

    g_string_printf(cmdline, QEMU_PXB_CMD QEMU_RP QEMU_T3D_PMEM,
                    tmpfs, tmpfs);

    qtest_start(cmdline->str);
    qtest_end();
    rmdir(tmpfs);
}

static void cxl_t3d_volatile(void)
{
    g_autoptr(GString) cmdline = g_string_new(NULL);

    g_string_printf(cmdline, QEMU_PXB_CMD QEMU_RP QEMU_T3D_VMEM);

    qtest_start(cmdline->str);
    qtest_end();
}

static void cxl_t3d_volatile_lsa(void)
{
    g_autoptr(GString) cmdline = g_string_new(NULL);
    g_autofree const char *tmpfs = NULL;

    tmpfs = g_dir_make_tmp("cxl-test-XXXXXX", NULL);

    g_string_printf(cmdline, QEMU_PXB_CMD QEMU_RP QEMU_T3D_VMEM_LSA,
                    tmpfs);

    qtest_start(cmdline->str);
    qtest_end();
    rmdir(tmpfs);
}

static void cxl_1pxb_2rp_2t3d(void)
{
    g_autoptr(GString) cmdline = g_string_new(NULL);
    g_autofree const char *tmpfs = NULL;

    tmpfs = g_dir_make_tmp("cxl-test-XXXXXX", NULL);

    g_string_printf(cmdline, QEMU_PXB_CMD QEMU_2RP QEMU_2T3D,
                    tmpfs, tmpfs, tmpfs, tmpfs);

    qtest_start(cmdline->str);
    qtest_end();
    rmdir(tmpfs);
}

static void cxl_2pxb_4rp_4t3d(void)
{
    g_autoptr(GString) cmdline = g_string_new(NULL);
    g_autofree const char *tmpfs = NULL;

    tmpfs = g_dir_make_tmp("cxl-test-XXXXXX", NULL);

    g_string_printf(cmdline, QEMU_2PXB_CMD QEMU_4RP QEMU_4T3D,
                    tmpfs, tmpfs, tmpfs, tmpfs, tmpfs, tmpfs,
                    tmpfs, tmpfs);

    qtest_start(cmdline->str);
    qtest_end();
    rmdir(tmpfs);
}

static void cxl_virt_2pxb_4rp_4t3d(void)
{
    g_autoptr(GString) cmdline = g_string_new(NULL);
    g_autofree const char *tmpfs = NULL;

    tmpfs = g_dir_make_tmp("cxl-test-XXXXXX", NULL);

    g_string_printf(cmdline, QEMU_VIRT_2PXB_CMD QEMU_4RP QEMU_4T3D,
                    tmpfs, tmpfs, tmpfs, tmpfs, tmpfs, tmpfs,
                    tmpfs, tmpfs);

    qtest_start(cmdline->str);
    qtest_end();
    rmdir(tmpfs);
}
#endif /* CONFIG_POSIX */

int main(int argc, char **argv)
{
    const char *arch = qtest_get_arch();

    g_test_init(&argc, &argv, NULL);
    if (strcmp(arch, "i386") == 0 || strcmp(arch, "x86_64") == 0) {
        qtest_add_func("/pci/cxl/basic_hostbridge", cxl_basic_hb);
        qtest_add_func("/pci/cxl/basic_pxb", cxl_basic_pxb);
        qtest_add_func("/pci/cxl/pxb_with_window", cxl_pxb_with_window);
        qtest_add_func("/pci/cxl/pxb_x2_with_window", cxl_2pxb_with_window);
        qtest_add_func("/pci/cxl/rp", cxl_root_port);
        qtest_add_func("/pci/cxl/rp_x2", cxl_2root_port);
#ifdef CONFIG_POSIX
        qtest_add_func("/pci/cxl/type2_sync_handshake",
                       cxl_t2_sync_handshake);
        qtest_add_func("/pci/cxl/type2_sync_bad_capacity",
                       cxl_t2_sync_bad_capacity);
        qtest_add_func("/pci/cxl/type2_sync_bad_latency",
                       cxl_t2_sync_bad_latency);
        qtest_add_func("/pci/cxl/type2_sync_bad_request_id",
                       cxl_t2_sync_bad_request_id);
        qtest_add_func("/pci/cxl/type2_sync_bad_crc",
                       cxl_t2_sync_bad_crc);
        qtest_add_func("/pci/cxl/type2_direct_cfmws",
                       cxl_t2_direct_cfmws);
        qtest_add_func("/pci/cxl/type2_cfmws_reject_two_targets",
                       cxl_t2_cfmws_reject_two_targets);
        qtest_add_func("/pci/cxl/type2_cfmws_reject_512m",
                       cxl_t2_cfmws_reject_512m);
        qtest_add_func("/pci/cxl/type2_cfmws_reject_switch",
                       cxl_t2_cfmws_reject_switch);
        qtest_add_func("/pci/cxl/type2_dvsec", cxl_t2_dvsec);
        qtest_add_func("/pci/cxl/type2_dvsec_bad_fractional_cache",
                       cxl_t2_dvsec_bad_fractional_cache);
        qtest_add_func("/pci/cxl/type2_dvsec_bad_oversized_cache",
                       cxl_t2_dvsec_bad_oversized_cache);
        qtest_add_func("/pci/cxl/type3_device", cxl_t3d_deprecated);
        qtest_add_func("/pci/cxl/type3_device_pmem", cxl_t3d_persistent);
        qtest_add_func("/pci/cxl/type3_device_vmem", cxl_t3d_volatile);
        qtest_add_func("/pci/cxl/type3_device_vmem_lsa", cxl_t3d_volatile_lsa);
        qtest_add_func("/pci/cxl/rp_x2_type3_x2", cxl_1pxb_2rp_2t3d);
        qtest_add_func("/pci/cxl/pxb_x2_root_port_x4_type3_x4",
                       cxl_2pxb_4rp_4t3d);
#endif
    } else if (strcmp(arch, "aarch64") == 0) {
#ifdef CONFIG_POSIX
        qtest_add_func("/pci/cxl/virt/pxb_x2_root_port_x4_type3_x4",
                       cxl_virt_2pxb_4rp_4t3d);
#endif
    }

    return g_test_run();
}
