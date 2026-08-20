/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef HW_CXL_CXL_MEMSIM_WAIT_H
#define HW_CXL_CXL_MEMSIM_WAIT_H

#include <stdint.h>

typedef enum CXLMemSimWaitAction {
    CXL_MEMSIM_WAIT_SPIN,
    CXL_MEMSIM_WAIT_SLEEP,
} CXLMemSimWaitAction;

static inline CXLMemSimWaitAction
cxl_memsim_wait_action(uint64_t elapsed_ns, uint64_t spin_ns)
{
    return elapsed_ns < spin_ns ? CXL_MEMSIM_WAIT_SPIN
                                : CXL_MEMSIM_WAIT_SLEEP;
}

#endif
