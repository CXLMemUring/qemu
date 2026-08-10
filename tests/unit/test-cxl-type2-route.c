/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "qemu/osdep.h"

#include "hw/cxl/cxl_type2.h"
#include "qemu/units.h"

static void test_shape_valid(void)
{
    g_assert_true(cxl_type2_cfmws_shape_valid(
        1, 1, 0, 256 * MiB, 0, 256 * MiB, 80 * MiB, 8));
}

static void test_shape_rejections(void)
{
    g_assert_false(cxl_type2_cfmws_shape_valid(
        2, 1, 0, 256 * MiB, 0, 256 * MiB, 80 * MiB, 8));
    g_assert_false(cxl_type2_cfmws_shape_valid(
        1, 2, 0, 256 * MiB, 0, 256 * MiB, 80 * MiB, 8));
    g_assert_false(cxl_type2_cfmws_shape_valid(
        1, 1, 1, 256 * MiB, 0, 256 * MiB, 80 * MiB, 8));
    g_assert_false(cxl_type2_cfmws_shape_valid(
        1, 1, 0, 512 * MiB, 0, 256 * MiB, 80 * MiB, 8));
    g_assert_false(cxl_type2_cfmws_shape_valid(
        1, 1, 0, 256 * MiB, 4 * KiB, 256 * MiB, 80 * MiB, 8));
    g_assert_false(cxl_type2_cfmws_shape_valid(
        1, 1, 0, 256 * MiB, 0, 64 * MiB, 80 * MiB, 8));
    g_assert_false(cxl_type2_cfmws_shape_valid(
        1, 1, 0, 256 * MiB, 0, 256 * MiB, 256 * MiB, 8));
    g_assert_false(cxl_type2_cfmws_shape_valid(
        1, 1, 0, 256 * MiB, 0, 256 * MiB, UINT64_MAX - 3, 8));
}

static void test_protocol_gate(void)
{
    g_assert_true(cxl_type2_cfmws_protocol_enabled(true, false));
    g_assert_true(cxl_type2_cfmws_protocol_enabled(false, true));
    g_assert_false(cxl_type2_cfmws_protocol_enabled(false, false));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/cxl/type2/route/valid", test_shape_valid);
    g_test_add_func("/cxl/type2/route/rejections", test_shape_rejections);
    g_test_add_func("/cxl/type2/route/protocol-gate", test_protocol_gate);
    return g_test_run();
}
