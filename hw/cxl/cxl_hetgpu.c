/*
 * CXL Type 2 Device - hetGPU Backend Integration
 * Provides CUDA compatibility layer for CXL Type 2 GPU accelerators
 *
 * This file implements stub functions for the hetGPU backend.
 * When the actual hetGPU library is available, it will be loaded dynamically.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/cxl/cxl_hetgpu.h"
#include <dlfcn.h>

/* Default device properties for simulation mode */
static const HetGPUDeviceProps default_props = {
    .name = "Virtual GPU (TMatmul)",
    .total_memory = 4ULL * 1024 * 1024 * 1024,  /* 4GB */
    .compute_capability_major = 8,
    .compute_capability_minor = 0,
    .max_threads_per_block = 1024,
    .max_block_dim = {1024, 1024, 64},
    .max_grid_dim = {65535, 65535, 65535},
    .warp_size = 32,
    .multiprocessor_count = 80,
    .clock_rate_khz = 1500000,
    .memory_clock_rate_khz = 5000000,
    .memory_bus_width = 256,
    .l2_cache_size = 6 * 1024 * 1024,  /* 6MB */
    .supports_managed_memory = true,
    .supports_coherent_memory = true,
    .backend_type = HETGPU_BACKEND_SIMULATION,
};

/* Function pointer types for dynamic loading */
typedef int (*cuInit_fn)(unsigned int);
typedef int (*cuDeviceGetCount_fn)(int *);
typedef int (*cuCtxCreate_fn)(void **, unsigned int, int);
typedef int (*cuCtxDestroy_fn)(void *);
typedef int (*cuCtxSynchronize_fn)(void);
typedef int (*cuMemAlloc_fn)(uint64_t *, size_t);
typedef int (*cuMemFree_fn)(uint64_t);
typedef int (*cuMemcpyHtoD_fn)(uint64_t, const void *, size_t);
typedef int (*cuMemcpyDtoH_fn)(void *, uint64_t, size_t);
typedef int (*cuModuleLoadData_fn)(void **, const void *);
typedef int (*cuModuleGetFunction_fn)(void **, void *, const char *);
typedef int (*cuLaunchKernel_fn)(void *, unsigned int, unsigned int, unsigned int,
                                  unsigned int, unsigned int, unsigned int,
                                  unsigned int, void *, void **, void **);

/* Loaded function pointers */
static struct {
    cuInit_fn cuInit;
    cuDeviceGetCount_fn cuDeviceGetCount;
    cuCtxCreate_fn cuCtxCreate;
    cuCtxDestroy_fn cuCtxDestroy;
    cuCtxSynchronize_fn cuCtxSynchronize;
    cuMemAlloc_fn cuMemAlloc;
    cuMemFree_fn cuMemFree;
    cuMemcpyHtoD_fn cuMemcpyHtoD;
    cuMemcpyDtoH_fn cuMemcpyDtoH;
    cuModuleLoadData_fn cuModuleLoadData;
    cuModuleGetFunction_fn cuModuleGetFunction;
    cuLaunchKernel_fn cuLaunchKernel;
} g_cuda_funcs = {0};

HetGPUError hetgpu_init(HetGPUState *state, HetGPUBackendType backend,
                        int device_index, const char *hetgpu_lib_path)
{
    if (!state) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    memset(state, 0, sizeof(*state));
    state->backend = backend;
    state->device_index = device_index;

    /* Try to load hetGPU library */
    if (hetgpu_lib_path && hetgpu_lib_path[0] != '\0') {
        state->hetgpu_lib = dlopen(hetgpu_lib_path, RTLD_NOW | RTLD_LOCAL);
        if (state->hetgpu_lib) {
            qemu_log("CXL hetGPU: Loaded library from %s\n", hetgpu_lib_path);

            /* Load function pointers */
            g_cuda_funcs.cuInit = dlsym(state->hetgpu_lib, "cuInit");
            g_cuda_funcs.cuDeviceGetCount = dlsym(state->hetgpu_lib, "cuDeviceGetCount");
            g_cuda_funcs.cuCtxCreate = dlsym(state->hetgpu_lib, "cuCtxCreate_v2");
            g_cuda_funcs.cuCtxDestroy = dlsym(state->hetgpu_lib, "cuCtxDestroy_v2");
            g_cuda_funcs.cuCtxSynchronize = dlsym(state->hetgpu_lib, "cuCtxSynchronize");
            g_cuda_funcs.cuMemAlloc = dlsym(state->hetgpu_lib, "cuMemAlloc_v2");
            g_cuda_funcs.cuMemFree = dlsym(state->hetgpu_lib, "cuMemFree_v2");
            g_cuda_funcs.cuMemcpyHtoD = dlsym(state->hetgpu_lib, "cuMemcpyHtoD_v2");
            g_cuda_funcs.cuMemcpyDtoH = dlsym(state->hetgpu_lib, "cuMemcpyDtoH_v2");
            g_cuda_funcs.cuModuleLoadData = dlsym(state->hetgpu_lib, "cuModuleLoadData");
            g_cuda_funcs.cuModuleGetFunction = dlsym(state->hetgpu_lib, "cuModuleGetFunction");
            g_cuda_funcs.cuLaunchKernel = dlsym(state->hetgpu_lib, "cuLaunchKernel");

            if (g_cuda_funcs.cuInit) {
                int err = g_cuda_funcs.cuInit(0);
                if (err == 0) {
                    state->initialized = true;
                    state->backend = backend != HETGPU_BACKEND_AUTO ? backend : HETGPU_BACKEND_SIMULATION;
                    state->props = default_props;
                    qemu_log("CXL hetGPU: Backend initialized successfully\n");
                    return HETGPU_SUCCESS;
                }
                qemu_log("CXL hetGPU: cuInit failed with error %d\n", err);
            }
        } else {
            qemu_log("CXL hetGPU: Failed to load library: %s\n", dlerror());
        }
    }

    /* Fall back to simulation mode */
    qemu_log("CXL hetGPU: Using simulation mode\n");
    state->initialized = true;
    state->backend = HETGPU_BACKEND_SIMULATION;
    state->props = default_props;
    state->context = (void *)1;  /* Dummy context for simulation */

    return HETGPU_SUCCESS;
}

void hetgpu_cleanup(HetGPUState *state)
{
    if (!state) {
        return;
    }

    if (state->context && g_cuda_funcs.cuCtxDestroy) {
        g_cuda_funcs.cuCtxDestroy(state->context);
    }

    if (state->hetgpu_lib) {
        dlclose(state->hetgpu_lib);
    }

    qemu_log("CXL hetGPU: Stats - Kernel launches: %lu, Memory ops: %lu, Coherency ops: %lu\n",
             state->kernel_launches, state->memory_ops, state->coherency_ops);

    memset(state, 0, sizeof(*state));
}

HetGPUError hetgpu_get_device_count(int *count)
{
    if (!count) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    if (g_cuda_funcs.cuDeviceGetCount) {
        int err = g_cuda_funcs.cuDeviceGetCount(count);
        return err == 0 ? HETGPU_SUCCESS : HETGPU_ERROR_NO_DEVICE;
    }

    *count = 1;  /* Simulation mode: 1 virtual device */
    return HETGPU_SUCCESS;
}

HetGPUError hetgpu_get_device_props(HetGPUState *state, HetGPUDeviceProps *props)
{
    if (!state || !props) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    *props = state->props;
    return HETGPU_SUCCESS;
}

HetGPUError hetgpu_create_context(HetGPUState *state)
{
    if (!state || !state->initialized) {
        return HETGPU_ERROR_NOT_INITIALIZED;
    }

    if (state->backend == HETGPU_BACKEND_SIMULATION) {
        state->context = (void *)1;  /* Dummy context */
        return HETGPU_SUCCESS;
    }

    if (g_cuda_funcs.cuCtxCreate) {
        void *ctx = NULL;
        int err = g_cuda_funcs.cuCtxCreate(&ctx, 0, state->device_index);
        if (err == 0) {
            state->context = ctx;
            return HETGPU_SUCCESS;
        }
        qemu_log("CXL hetGPU: cuCtxCreate failed with error %d\n", err);
        return HETGPU_ERROR_INVALID_CONTEXT;
    }

    return HETGPU_ERROR_NOT_INITIALIZED;
}

void hetgpu_destroy_context(HetGPUState *state)
{
    if (!state || !state->context) {
        return;
    }

    if (state->backend != HETGPU_BACKEND_SIMULATION && g_cuda_funcs.cuCtxDestroy) {
        g_cuda_funcs.cuCtxDestroy(state->context);
    }

    state->context = NULL;
}

HetGPUError hetgpu_synchronize(HetGPUState *state)
{
    if (!state || !state->initialized) {
        return HETGPU_ERROR_NOT_INITIALIZED;
    }

    if (state->backend == HETGPU_BACKEND_SIMULATION) {
        return HETGPU_SUCCESS;
    }

    if (g_cuda_funcs.cuCtxSynchronize) {
        int err = g_cuda_funcs.cuCtxSynchronize();
        return err == 0 ? HETGPU_SUCCESS : HETGPU_ERROR_UNKNOWN;
    }

    return HETGPU_SUCCESS;
}

HetGPUError hetgpu_malloc(HetGPUState *state, size_t size,
                          HetGPUMemFlags flags, HetGPUDevicePtr *dev_ptr)
{
    (void)flags;

    if (!state || !state->initialized || !dev_ptr) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    state->memory_ops++;

    if (state->backend == HETGPU_BACKEND_SIMULATION) {
        /* Simulate allocation with a fake pointer */
        static uint64_t next_ptr = 0x100000000ULL;
        *dev_ptr = next_ptr;
        next_ptr += (size + 0xFFF) & ~0xFFF;  /* Page align */
        state->allocated_memory += size;
        return HETGPU_SUCCESS;
    }

    if (g_cuda_funcs.cuMemAlloc) {
        uint64_t ptr = 0;
        int err = g_cuda_funcs.cuMemAlloc(&ptr, size);
        if (err == 0) {
            *dev_ptr = ptr;
            state->allocated_memory += size;
            return HETGPU_SUCCESS;
        }
        return HETGPU_ERROR_OUT_OF_MEMORY;
    }

    return HETGPU_ERROR_NOT_INITIALIZED;
}

HetGPUError hetgpu_free(HetGPUState *state, HetGPUDevicePtr dev_ptr)
{
    if (!state || !state->initialized) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    state->memory_ops++;

    if (state->backend == HETGPU_BACKEND_SIMULATION) {
        return HETGPU_SUCCESS;  /* No-op for simulation */
    }

    if (g_cuda_funcs.cuMemFree) {
        int err = g_cuda_funcs.cuMemFree(dev_ptr);
        return err == 0 ? HETGPU_SUCCESS : HETGPU_ERROR_INVALID_VALUE;
    }

    return HETGPU_SUCCESS;
}

HetGPUError hetgpu_memcpy_htod(HetGPUState *state, HetGPUDevicePtr dst,
                               const void *src, size_t size)
{
    if (!state || !state->initialized || !src) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    state->memory_ops++;

    if (state->backend == HETGPU_BACKEND_SIMULATION) {
        return HETGPU_SUCCESS;  /* No-op for simulation */
    }

    if (g_cuda_funcs.cuMemcpyHtoD) {
        int err = g_cuda_funcs.cuMemcpyHtoD(dst, src, size);
        return err == 0 ? HETGPU_SUCCESS : HETGPU_ERROR_INVALID_VALUE;
    }

    return HETGPU_SUCCESS;
}

HetGPUError hetgpu_memcpy_dtoh(HetGPUState *state, void *dst,
                               HetGPUDevicePtr src, size_t size)
{
    if (!state || !state->initialized || !dst) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    state->memory_ops++;

    if (state->backend == HETGPU_BACKEND_SIMULATION) {
        memset(dst, 0, size);  /* Return zeros for simulation */
        return HETGPU_SUCCESS;
    }

    if (g_cuda_funcs.cuMemcpyDtoH) {
        int err = g_cuda_funcs.cuMemcpyDtoH(dst, src, size);
        return err == 0 ? HETGPU_SUCCESS : HETGPU_ERROR_INVALID_VALUE;
    }

    return HETGPU_SUCCESS;
}

HetGPUError hetgpu_memset(HetGPUState *state, HetGPUDevicePtr dev_ptr,
                          int value, size_t size)
{
    (void)dev_ptr;
    (void)value;
    (void)size;

    if (!state || !state->initialized) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    state->memory_ops++;
    return HETGPU_SUCCESS;
}

HetGPUError hetgpu_create_coherent_region(HetGPUState *state, size_t size,
                                          HetGPUCoherentRegion *region)
{
    if (!state || !state->initialized || !region) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    state->coherency_ops++;

    /* Allocate host-mapped coherent memory */
    region->host_ptr = g_malloc0(size);
    if (!region->host_ptr) {
        return HETGPU_ERROR_OUT_OF_MEMORY;
    }

    region->size = size;
    region->flags = HETGPU_MEM_HOST_MAPPED;
    region->is_coherent = true;

    /* Get device pointer */
    HetGPUError err = hetgpu_malloc(state, size, HETGPU_MEM_HOST_MAPPED,
                                    &region->device_ptr);
    if (err != HETGPU_SUCCESS) {
        g_free(region->host_ptr);
        return err;
    }

    return HETGPU_SUCCESS;
}

void hetgpu_destroy_coherent_region(HetGPUState *state,
                                    HetGPUCoherentRegion *region)
{
    if (!state || !region) {
        return;
    }

    if (region->device_ptr) {
        hetgpu_free(state, region->device_ptr);
    }
    if (region->host_ptr) {
        g_free(region->host_ptr);
    }

    memset(region, 0, sizeof(*region));
}

void hetgpu_set_coherency_callback(HetGPUState *state,
                                   HetGPUCoherencyCallback callback,
                                   void *opaque)
{
    if (!state) {
        return;
    }

    state->coherency_callback = callback;
    state->cxl_opaque = opaque;
}

HetGPUError hetgpu_flush_cache(HetGPUState *state, HetGPUDevicePtr dev_ptr,
                               size_t size)
{
    (void)dev_ptr;
    (void)size;

    if (!state || !state->initialized) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    state->coherency_ops++;
    return HETGPU_SUCCESS;
}

HetGPUError hetgpu_invalidate_cache(HetGPUState *state, HetGPUDevicePtr dev_ptr,
                                    size_t size)
{
    (void)dev_ptr;
    (void)size;

    if (!state || !state->initialized) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    state->coherency_ops++;
    return HETGPU_SUCCESS;
}

HetGPUError hetgpu_load_ptx(HetGPUState *state, const char *ptx_source,
                            HetGPUModule *module)
{
    if (!state || !state->initialized || !ptx_source || !module) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    if (state->backend == HETGPU_BACKEND_SIMULATION) {
        *module = (void *)0x12345678;  /* Dummy module handle */
        return HETGPU_SUCCESS;
    }

    if (g_cuda_funcs.cuModuleLoadData) {
        void *mod = NULL;
        int err = g_cuda_funcs.cuModuleLoadData(&mod, ptx_source);
        if (err == 0) {
            *module = mod;
            return HETGPU_SUCCESS;
        }
        qemu_log("CXL hetGPU: cuModuleLoadData failed with error %d\n", err);
        return HETGPU_ERROR_INVALID_PTX;
    }

    return HETGPU_ERROR_NOT_INITIALIZED;
}

HetGPUError hetgpu_load_cubin(HetGPUState *state, const void *cubin_data,
                              size_t cubin_size, HetGPUModule *module)
{
    (void)cubin_size;

    if (!state || !state->initialized || !cubin_data || !module) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    if (state->backend == HETGPU_BACKEND_SIMULATION) {
        *module = (void *)0x12345678;
        return HETGPU_SUCCESS;
    }

    if (g_cuda_funcs.cuModuleLoadData) {
        void *mod = NULL;
        int err = g_cuda_funcs.cuModuleLoadData(&mod, cubin_data);
        if (err == 0) {
            *module = mod;
            return HETGPU_SUCCESS;
        }
        return HETGPU_ERROR_INVALID_PTX;
    }

    return HETGPU_ERROR_NOT_INITIALIZED;
}

void hetgpu_unload_module(HetGPUState *state, HetGPUModule module)
{
    (void)state;
    (void)module;
    /* Module unloading is handled by context destruction */
}

HetGPUError hetgpu_get_function(HetGPUState *state, HetGPUModule module,
                                const char *name, HetGPUFunction *function)
{
    if (!state || !state->initialized || !module || !name || !function) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    if (state->backend == HETGPU_BACKEND_SIMULATION) {
        *function = (void *)0x87654321;  /* Dummy function handle */
        return HETGPU_SUCCESS;
    }

    if (g_cuda_funcs.cuModuleGetFunction) {
        void *func = NULL;
        int err = g_cuda_funcs.cuModuleGetFunction(&func, module, name);
        if (err == 0) {
            *function = func;
            return HETGPU_SUCCESS;
        }
        return HETGPU_ERROR_UNKNOWN;
    }

    return HETGPU_ERROR_NOT_INITIALIZED;
}

HetGPUError hetgpu_launch_kernel(HetGPUState *state, HetGPUFunction function,
                                 const HetGPULaunchConfig *config,
                                 void **args, size_t num_args)
{
    if (!state || !state->initialized || !function || !config) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    state->kernel_launches++;

    if (state->backend == HETGPU_BACKEND_SIMULATION) {
        qemu_log("CXL hetGPU: Simulated kernel launch grid=(%u,%u,%u) block=(%u,%u,%u)\n",
                 config->grid_dim[0], config->grid_dim[1], config->grid_dim[2],
                 config->block_dim[0], config->block_dim[1], config->block_dim[2]);
        return HETGPU_SUCCESS;
    }

    if (g_cuda_funcs.cuLaunchKernel) {
        int err = g_cuda_funcs.cuLaunchKernel(
            function,
            config->grid_dim[0], config->grid_dim[1], config->grid_dim[2],
            config->block_dim[0], config->block_dim[1], config->block_dim[2],
            config->shared_mem_bytes, config->stream,
            args, NULL);
        if (err == 0) {
            return HETGPU_SUCCESS;
        }
        qemu_log("CXL hetGPU: cuLaunchKernel failed with error %d\n", err);
        return HETGPU_ERROR_LAUNCH_FAILED;
    }

    return HETGPU_SUCCESS;
}

HetGPUError hetgpu_create_stream(HetGPUState *state, HetGPUStream *stream)
{
    if (!state || !state->initialized || !stream) {
        return HETGPU_ERROR_INVALID_VALUE;
    }

    *stream = (void *)1;  /* Dummy stream for now */
    return HETGPU_SUCCESS;
}

void hetgpu_destroy_stream(HetGPUState *state, HetGPUStream stream)
{
    (void)state;
    (void)stream;
}

HetGPUError hetgpu_stream_synchronize(HetGPUState *state, HetGPUStream stream)
{
    (void)stream;
    return hetgpu_synchronize(state);
}

const char *hetgpu_get_error_string(HetGPUError error)
{
    switch (error) {
    case HETGPU_SUCCESS:
        return "Success";
    case HETGPU_ERROR_NOT_INITIALIZED:
        return "Not initialized";
    case HETGPU_ERROR_NO_DEVICE:
        return "No device";
    case HETGPU_ERROR_INVALID_DEVICE:
        return "Invalid device";
    case HETGPU_ERROR_INVALID_CONTEXT:
        return "Invalid context";
    case HETGPU_ERROR_OUT_OF_MEMORY:
        return "Out of memory";
    case HETGPU_ERROR_INVALID_PTX:
        return "Invalid PTX";
    case HETGPU_ERROR_LAUNCH_FAILED:
        return "Kernel launch failed";
    case HETGPU_ERROR_INVALID_VALUE:
        return "Invalid value";
    case HETGPU_ERROR_NOT_SUPPORTED:
        return "Not supported";
    default:
        return "Unknown error";
    }
}

const char *hetgpu_get_backend_name(HetGPUBackendType backend)
{
    switch (backend) {
    case HETGPU_BACKEND_AUTO:
        return "Auto";
    case HETGPU_BACKEND_INTEL:
        return "Intel Level Zero";
    case HETGPU_BACKEND_AMD:
        return "AMD HIP/ROCm";
    case HETGPU_BACKEND_NVIDIA:
        return "NVIDIA CUDA";
    case HETGPU_BACKEND_TENSTORRENT:
        return "Tenstorrent";
    case HETGPU_BACKEND_SIMULATION:
        return "Simulation";
    default:
        return "Unknown";
    }
}
