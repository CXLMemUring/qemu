/*
 * Unit tests for registering existing host memory with CXL hetGPU
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/cxl/cxl_hetgpu.h"

#include <dlfcn.h>

typedef void (*FakeReset)(void);
typedef unsigned (*FakeCounter)(void);
typedef void (*FakeSetResult)(int);

typedef struct FakeCuda {
    void *handle;
    FakeReset reset;
    FakeCounter host_register_calls;
    FakeCounter host_get_device_pointer_calls;
    FakeCounter host_unregister_calls;
    FakeSetResult set_host_get_device_pointer_result;
    FakeSetResult set_host_unregister_result;
} FakeCuda;

static char *fake_cuda_path(void)
{
    const char *build_dir = g_getenv("G_TEST_BUILDDIR");
    g_autofree char *current_dir = NULL;
    g_autofree char *relative = NULL;

    if (build_dir) {
        relative = g_build_filename(build_dir, "hetgpu-cuda-fake.so",
                                    NULL);
    } else {
        current_dir = g_get_current_dir();
        relative = g_build_filename(current_dir, "tests", "unit",
                                    "hetgpu-cuda-fake.so", NULL);
    }
    return g_canonicalize_filename(relative, NULL);
}

static FakeCuda fake_cuda_open(const char *path)
{
    FakeCuda fake = { 0 };

    fake.handle = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    g_assert_nonnull(fake.handle);
    fake.reset = dlsym(fake.handle, "hetgpu_cuda_fake_reset");
    fake.host_register_calls =
        dlsym(fake.handle, "hetgpu_cuda_fake_host_register_calls");
    fake.host_get_device_pointer_calls = dlsym(
        fake.handle, "hetgpu_cuda_fake_host_get_device_pointer_calls");
    fake.host_unregister_calls =
        dlsym(fake.handle, "hetgpu_cuda_fake_host_unregister_calls");
    fake.set_host_get_device_pointer_result = dlsym(
        fake.handle, "hetgpu_cuda_fake_set_host_get_device_pointer_result");
    fake.set_host_unregister_result = dlsym(
        fake.handle, "hetgpu_cuda_fake_set_host_unregister_result");
    g_assert_nonnull(fake.reset);
    g_assert_nonnull(fake.host_register_calls);
    g_assert_nonnull(fake.host_get_device_pointer_calls);
    g_assert_nonnull(fake.host_unregister_calls);
    g_assert_nonnull(fake.set_host_get_device_pointer_result);
    g_assert_nonnull(fake.set_host_unregister_result);
    return fake;
}

static void test_register_existing_backing(void)
{
    g_autofree char *path = fake_cuda_path();
    g_autofree void *backing = g_malloc0(4096);
    FakeCuda fake = fake_cuda_open(path);
    HetGPUState state;
    HetGPUCoherentRegion region = { 0 };

    fake.reset();
    g_assert_cmpint(hetgpu_init(&state, HETGPU_BACKEND_NVIDIA, 0, path),
                    ==, HETGPU_SUCCESS);
    g_assert_cmpint(
        hetgpu_register_coherent_region(&state, backing, 4096, &region),
        ==, HETGPU_SUCCESS);
    g_assert_true(region.host_ptr == backing);
    g_assert_cmpuint(region.size, ==, 4096);
    g_assert_cmphex(region.device_ptr, ==,
                    (uint64_t)(uintptr_t)backing +
                    UINT64_C(0x100000000));
    g_assert_true(region.is_coherent);
    g_assert_true(region.host_registered);
    g_assert_cmpuint(fake.host_register_calls(), ==, 1);
    g_assert_cmpuint(fake.host_get_device_pointer_calls(), ==, 1);

    g_assert_cmpint(hetgpu_unregister_coherent_region(&state, &region),
                    ==, HETGPU_SUCCESS);
    g_assert_null(region.host_ptr);
    g_assert_cmpuint(fake.host_unregister_calls(), ==, 1);
    g_assert_cmpint(hetgpu_unregister_coherent_region(&state, &region),
                    ==, HETGPU_SUCCESS);
    g_assert_cmpuint(fake.host_unregister_calls(), ==, 1);
    hetgpu_cleanup(&state);
    dlclose(fake.handle);
}

static void test_device_pointer_failure_rolls_back_registration(void)
{
    g_autofree char *path = fake_cuda_path();
    g_autofree void *backing = g_malloc0(4096);
    FakeCuda fake = fake_cuda_open(path);
    HetGPUState state;
    HetGPUCoherentRegion region = { 0 };

    fake.reset();
    g_assert_cmpint(hetgpu_init(&state, HETGPU_BACKEND_NVIDIA, 0, path),
                    ==, HETGPU_SUCCESS);
    fake.set_host_get_device_pointer_result(17);
    g_assert_cmpint(
        hetgpu_register_coherent_region(&state, backing, 4096, &region),
        ==, HETGPU_ERROR_NOT_SUPPORTED);
    g_assert_null(region.host_ptr);
    g_assert_cmpuint(fake.host_register_calls(), ==, 1);
    g_assert_cmpuint(fake.host_get_device_pointer_calls(), ==, 1);
    g_assert_cmpuint(fake.host_unregister_calls(), ==, 1);
    hetgpu_cleanup(&state);
    dlclose(fake.handle);
}

static void test_registration_rejects_unsupported_backends(void)
{
    g_autofree void *backing = g_malloc0(4096);
    HetGPUState state = {
        .initialized = true,
        .backend = HETGPU_BACKEND_SIMULATION,
    };
    HetGPUCoherentRegion region = { 0 };

    g_assert_cmpint(
        hetgpu_register_coherent_region(&state, backing, 4096, &region),
        ==, HETGPU_ERROR_NOT_SUPPORTED);
    state.backend = HETGPU_BACKEND_NVIDIA;
    state.props.supports_coherent_memory = false;
    g_assert_cmpint(
        hetgpu_register_coherent_region(&state, backing, 4096, &region),
        ==, HETGPU_ERROR_NOT_SUPPORTED);
}

static void test_active_region_is_not_overwritten(void)
{
    g_autofree char *path = fake_cuda_path();
    g_autofree void *backing = g_malloc0(4096);
    FakeCuda fake = fake_cuda_open(path);
    HetGPUState state;
    HetGPUCoherentRegion region = { 0 };
    void *registered_host;

    fake.reset();
    g_assert_cmpint(hetgpu_init(&state, HETGPU_BACKEND_NVIDIA, 0, path),
                    ==, HETGPU_SUCCESS);
    g_assert_cmpint(
        hetgpu_register_coherent_region(&state, backing, 4096, &region),
        ==, HETGPU_SUCCESS);
    registered_host = region.host_ptr;
    g_assert_cmpint(
        hetgpu_register_coherent_region(&state, backing, 4096, &region),
        ==, HETGPU_ERROR_INVALID_VALUE);
    g_assert_true(region.host_ptr == registered_host);
    g_assert_true(region.host_registered);
    g_assert_cmpuint(fake.host_register_calls(), ==, 1);
    g_assert_cmpint(hetgpu_unregister_coherent_region(&state, &region),
                    ==, HETGPU_SUCCESS);
    hetgpu_cleanup(&state);
    dlclose(fake.handle);
}

static void test_failed_rollback_is_retryable(void)
{
    g_autofree char *path = fake_cuda_path();
    g_autofree void *backing = g_malloc0(4096);
    FakeCuda fake = fake_cuda_open(path);
    HetGPUState state;
    HetGPUCoherentRegion region = { 0 };

    fake.reset();
    g_assert_cmpint(hetgpu_init(&state, HETGPU_BACKEND_NVIDIA, 0, path),
                    ==, HETGPU_SUCCESS);
    fake.set_host_get_device_pointer_result(17);
    fake.set_host_unregister_result(23);
    g_assert_cmpint(
        hetgpu_register_coherent_region(&state, backing, 4096, &region),
        ==, HETGPU_ERROR_NOT_SUPPORTED);
    g_assert_true(region.host_ptr == backing);
    g_assert_true(region.host_registered);
    g_assert_cmpuint(fake.host_unregister_calls(), ==, 1);

    fake.set_host_unregister_result(0);
    g_assert_cmpint(hetgpu_unregister_coherent_region(&state, &region),
                    ==, HETGPU_SUCCESS);
    g_assert_cmpuint(fake.host_unregister_calls(), ==, 2);
    hetgpu_cleanup(&state);
    dlclose(fake.handle);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/cxl/hetgpu/register-existing-backing",
                    test_register_existing_backing);
    g_test_add_func("/cxl/hetgpu/device-pointer-failure",
                    test_device_pointer_failure_rolls_back_registration);
    g_test_add_func("/cxl/hetgpu/reject-unsupported",
                    test_registration_rejects_unsupported_backends);
    g_test_add_func("/cxl/hetgpu/reject-active-region",
                    test_active_region_is_not_overwritten);
    g_test_add_func("/cxl/hetgpu/retry-failed-rollback",
                    test_failed_rollback_is_retryable);
    return g_test_run();
}
