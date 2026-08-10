/*
 * Fake CUDA driver API for CXL hetGPU unit and qtests
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef TESTS_UNIT_HETGPU_CUDA_FAKE_H
#define TESTS_UNIT_HETGPU_CUDA_FAKE_H

#include <stddef.h>
#include <stdint.h>

void hetgpu_cuda_fake_reset(void);
unsigned hetgpu_cuda_fake_host_register_calls(void);
unsigned hetgpu_cuda_fake_host_get_device_pointer_calls(void);
unsigned hetgpu_cuda_fake_host_unregister_calls(void);
void hetgpu_cuda_fake_set_host_get_device_pointer_result(int result);
void hetgpu_cuda_fake_set_host_unregister_result(int result);

int cuInit(unsigned int flags);
int cuDeviceGetCount(int *count);
int cuDeviceGet(int *device, int ordinal);
int cuDeviceGetName(char *name, int length, int device);
int cuDeviceTotalMem_v2(size_t *bytes, int device);
int cuDeviceGetAttribute(int *value, int attribute, int device);
int cuCtxCreate_v2(void **context, unsigned int flags, int device);
int cuCtxDestroy_v2(void *context);
int cuCtxSynchronize(void);
int cuCtxPopCurrent_v2(void **context);
int cuCtxSetCurrent(void *context);
int cuMemHostRegister_v2(void *pointer, size_t size, unsigned int flags);
int cuMemHostGetDevicePointer_v2(uint64_t *device_pointer,
                                 void *host_pointer, unsigned int flags);
int cuMemHostUnregister(void *pointer);
int cuModuleLoadData(void **module, const void *image);
int cuModuleGetFunction(void **function, void *module, const char *name);
int cuLaunchKernel(void *function, unsigned int grid_x,
                   unsigned int grid_y, unsigned int grid_z,
                   unsigned int block_x, unsigned int block_y,
                   unsigned int block_z, unsigned int shared_mem,
                   void *stream, void **args, void **extra);
int cuGetErrorName(int error, const char **name);
int cuGetErrorString(int error, const char **description);

#endif
