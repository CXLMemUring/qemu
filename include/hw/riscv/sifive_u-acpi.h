/*
 * SiFive U ACPI table generation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_RISCV_SIFIVE_U_ACPI_H
#define HW_RISCV_SIFIVE_U_ACPI_H

typedef struct SiFiveUState SiFiveUState;

void sifive_u_acpi_setup(SiFiveUState *s);

#endif
