#include "qemu/osdep.h"

#include "hw/cxl/cxl_type3_memsim_v2.h"
#include "qapi/error.h"

static void test_default_config_is_disabled_and_valid(void)
{
    CxlType3MemsimV2Config config = cxl_type3_memsim_v2_default_config();
    Error *err = NULL;

    g_assert_false(config.enabled);
    g_assert_true(cxl_type3_memsim_v2_validate(&config, &err));
    g_assert_null(err);
}

static void test_enabled_config_rejects_invalid_host_id(void)
{
    CxlType3MemsimV2Config config = cxl_type3_memsim_v2_default_config();
    Error *err = NULL;

    config.enabled = true;
    config.host_id = CXL_MEMSIM_V2_MAX_ENDPOINTS;
    g_assert_false(cxl_type3_memsim_v2_validate(&config, &err));
    g_assert_nonnull(err);
    error_free(err);
}

static void test_enabled_config_rejects_invalid_geometry(void)
{
    CxlType3MemsimV2Config config = cxl_type3_memsim_v2_default_config();
    Error *err = NULL;

    config.enabled = true;
    config.cache_capacity = CXL_MEMSIM_V2_LINE_SIZE + 1;
    g_assert_false(cxl_type3_memsim_v2_validate(&config, &err));
    g_assert_nonnull(err);
    error_free(err);
}

static void test_enabled_config_rejects_write_through(void)
{
    CxlType3MemsimV2Config config = cxl_type3_memsim_v2_default_config();
    Error *err = NULL;

    config.enabled = true;
    config.write_through = true;
    g_assert_false(cxl_type3_memsim_v2_validate(&config, &err));
    g_assert_nonnull(err);
    error_free(err);
}

static void test_enabled_access_without_session_fails_closed(void)
{
    CxlType3MemsimV2 state = {
        .config = cxl_type3_memsim_v2_default_config(),
        .enabled = true,
    };
    uint64_t value = 0;

    g_assert_cmpint(cxl_type3_memsim_v2_read(&state, 0, &value, 8),
                    ==, MEMTX_ERROR);
    g_assert_cmpint(cxl_type3_memsim_v2_write(&state, 0, 1, 8),
                    ==, MEMTX_ERROR);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/cxl/type3/memsim-v2/default-config",
                    test_default_config_is_disabled_and_valid);
    g_test_add_func("/cxl/type3/memsim-v2/invalid-host-id",
                    test_enabled_config_rejects_invalid_host_id);
    g_test_add_func("/cxl/type3/memsim-v2/invalid-geometry",
                    test_enabled_config_rejects_invalid_geometry);
    g_test_add_func("/cxl/type3/memsim-v2/write-through-rejected",
                    test_enabled_config_rejects_write_through);
    g_test_add_func("/cxl/type3/memsim-v2/fail-closed",
                    test_enabled_access_without_session_fails_closed);
    return g_test_run();
}
