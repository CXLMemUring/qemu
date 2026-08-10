/*
 * Fake CUDA driver API for CXL hetGPU unit and qtests
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "hetgpu-cuda-fake.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static unsigned host_register_calls;
static unsigned host_get_device_pointer_calls;
static unsigned host_unregister_calls;
static int host_get_device_pointer_result;
static int host_unregister_result;
static int forced_unregister_failures = -1;

static void trace_event(const char *event)
{
    const char *path = getenv("HETGPU_CUDA_FAKE_TRACE");
    FILE *stream;

    if (!path) {
        return;
    }
    stream = fopen(path, "a");
    if (!stream) {
        return;
    }
    fputs(event, stream);
    fclose(stream);
}

void hetgpu_cuda_fake_reset(void)
{
    host_register_calls = 0;
    host_get_device_pointer_calls = 0;
    host_unregister_calls = 0;
    host_get_device_pointer_result = 0;
    host_unregister_result = 0;
    forced_unregister_failures = -1;
}

unsigned hetgpu_cuda_fake_host_register_calls(void)
{
    return host_register_calls;
}

unsigned hetgpu_cuda_fake_host_get_device_pointer_calls(void)
{
    return host_get_device_pointer_calls;
}

unsigned hetgpu_cuda_fake_host_unregister_calls(void)
{
    return host_unregister_calls;
}

void hetgpu_cuda_fake_set_host_get_device_pointer_result(int result)
{
    host_get_device_pointer_result = result;
}

void hetgpu_cuda_fake_set_host_unregister_result(int result)
{
    host_unregister_result = result;
}

int cuInit(unsigned int flags)
{
    const char *forced_result = getenv("HETGPU_CUDA_FAKE_INIT_RESULT");

    if (forced_result) {
        return atoi(forced_result);
    }
    return flags == 0 ? 0 : 1;
}

int cuDeviceGetCount(int *count)
{
    *count = 1;
    return 0;
}

int cuDeviceGet(int *device, int ordinal)
{
    *device = ordinal;
    return 0;
}

int cuDeviceGetName(char *name, int length, int device)
{
    (void)device;
    strncpy(name, "hetGPU fake CUDA", length);
    if (length > 0) {
        name[length - 1] = '\0';
    }
    return 0;
}

int cuDeviceTotalMem_v2(size_t *bytes, int device)
{
    (void)device;
    *bytes = UINT64_C(16) << 30;
    return 0;
}

int cuDeviceGetAttribute(int *value, int attribute, int device)
{
    (void)attribute;
    (void)device;
    *value = 1;
    return 0;
}

int cuCtxCreate_v2(void **context, unsigned int flags, int device)
{
    (void)flags;
    (void)device;
    *context = (void *)(uintptr_t)0x12340000;
    return 0;
}

int cuCtxDestroy_v2(void *context)
{
    (void)context;
    return 0;
}

int cuCtxSynchronize(void)
{
    const char *forced_result = getenv(
        "HETGPU_CUDA_FAKE_SYNCHRONIZE_RESULT");

    return forced_result ? atoi(forced_result) : 0;
}

int cuCtxPopCurrent_v2(void **context)
{
    *context = (void *)(uintptr_t)0x12340000;
    return 0;
}

int cuCtxSetCurrent(void *context)
{
    (void)context;
    return 0;
}

int cuMemHostRegister_v2(void *pointer, size_t size, unsigned int flags)
{
    const char *forced_result;

    (void)pointer;
    (void)size;
    (void)flags;
    host_register_calls++;
    forced_result = getenv("HETGPU_CUDA_FAKE_HOST_REGISTER_RESULT");
    if (forced_result) {
        return atoi(forced_result);
    }
    trace_event("register\n");
    return 0;
}

int cuMemHostGetDevicePointer_v2(uint64_t *device_pointer,
                                 void *host_pointer, unsigned int flags)
{
    (void)flags;
    host_get_device_pointer_calls++;
    if (host_get_device_pointer_result == 0) {
        *device_pointer = (uint64_t)(uintptr_t)host_pointer +
                          UINT64_C(0x100000000);
    }
    return host_get_device_pointer_result;
}

int cuMemHostUnregister(void *pointer)
{
    const char *forced_failures;

    (void)pointer;
    host_unregister_calls++;
    trace_event("unregister\n");
    if (forced_unregister_failures < 0) {
        forced_failures = getenv(
            "HETGPU_CUDA_FAKE_UNREGISTER_FAILURES");
        forced_unregister_failures = forced_failures ?
            atoi(forced_failures) : 0;
    }
    if (forced_unregister_failures > 0) {
        forced_unregister_failures--;
        return 23;
    }
    return host_unregister_result;
}

int cuModuleLoadData(void **module, const void *image)
{
    if (!module || !image) {
        return 1;
    }
    *module = (void *)(uintptr_t)0x23450000;
    return 0;
}

int cuModuleGetFunction(void **function, void *module, const char *name)
{
    if (!function || !module || !name) {
        return 1;
    }
    *function = (void *)(uintptr_t)0x34560000;
    return 0;
}

static void create_marker(const char *path)
{
    FILE *stream;

    if (!path) {
        return;
    }
    stream = fopen(path, "w");
    if (!stream) {
        return;
    }
    fclose(stream);
}

int cuLaunchKernel(void *function, unsigned int grid_x,
                   unsigned int grid_y, unsigned int grid_z,
                   unsigned int block_x, unsigned int block_y,
                   unsigned int block_z, unsigned int shared_mem,
                   void *stream, void **args, void **extra)
{
    const char *entered = getenv("HETGPU_CUDA_FAKE_LAUNCH_ENTERED");
    const char *release = getenv("HETGPU_CUDA_FAKE_LAUNCH_RELEASE");
    unsigned int attempt;

    (void)grid_x;
    (void)grid_y;
    (void)grid_z;
    (void)block_x;
    (void)block_y;
    (void)block_z;
    (void)shared_mem;
    (void)stream;
    (void)args;
    (void)extra;
    if (!function) {
        return 1;
    }
    create_marker(entered);
    if (!release) {
        return 0;
    }
    for (attempt = 0; attempt < 5000; attempt++) {
        if (access(release, F_OK) == 0) {
            return 0;
        }
        usleep(1000);
    }
    return 1;
}

int cuGetErrorName(int error, const char **name)
{
    (void)error;
    *name = "CUDA_FAKE_ERROR";
    return 0;
}

int cuGetErrorString(int error, const char **description)
{
    (void)error;
    *description = "fake CUDA error";
    return 0;
}
