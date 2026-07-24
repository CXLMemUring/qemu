/*
 * QEMU RISC-V Board Compatible with SiFive Freedom U SDK
 *
 * Copyright (c) 2016-2017 Sagar Karandikar, sagark@eecs.berkeley.edu
 * Copyright (c) 2017 SiFive, Inc.
 * Copyright (c) 2019 Bin Meng <bmeng.cn@gmail.com>
 *
 * Provides a board compatible with the SiFive Freedom U SDK:
 *
 * 0) UART
 * 1) CLINT (Core Level Interruptor)
 * 2) PLIC (Platform Level Interrupt Controller)
 * 3) PRCI (Power, Reset, Clock, Interrupt)
 * 4) GPIO (General Purpose Input/Output Controller)
 * 5) OTP (One-Time Programmable) memory with stored serial number
 * 6) GEM (Gigabit Ethernet Controller) and management block
 * 7) DMA (Direct Memory Access Controller)
 * 8) SPI0 connected to an SPI flash
 * 9) SPI2 connected to an SD card
 * 10) PWM0 and PWM1
 *
 * This board currently generates devicetree dynamically that indicates at least
 * two harts and up to five harts.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/boards.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/cpu/cluster.h"
#include "hw/misc/unimp.h"
#include "hw/sd/sd.h"
#include "hw/ssi/ssi.h"
#include "target/riscv/cpu.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/numa.h"
#include "hw/riscv/sifive_u.h"
#include "hw/riscv/sifive_u-acpi.h"
#include "hw/riscv/boot.h"
#include "hw/char/sifive_uart.h"
#include "hw/cxl/cxl_host.h"
#include "hw/intc/riscv_aclint.h"
#include "hw/intc/sifive_plic.h"
#include "chardev/char.h"
#include "net/eth.h"
#include "system/device_tree.h"
#include "system/runstate.h"
#include "system/system.h"

#include <libfdt.h>

/* CLINT timebase frequency */
#define CLINT_TIMEBASE_FREQ 1000000

const MemMapEntry sifive_u_memmap[] = {
    [SIFIVE_U_DEV_DEBUG] =    {        0x0,      0x100 },
    [SIFIVE_U_DEV_MROM] =     {     0x1000,     0xf000 },
    [SIFIVE_U_DEV_CLINT] =    {  0x2000000,    0x10000 },
    [SIFIVE_U_DEV_L2CC] =     {  0x2010000,     0x1000 },
    [SIFIVE_U_DEV_PDMA] =     {  0x3000000,   0x100000 },
    [SIFIVE_U_DEV_L2LIM] =    {  0x8000000,  0x2000000 },
    [SIFIVE_U_DEV_PLIC] =     {  0xc000000,  0x4000000 },
    [SIFIVE_U_DEV_PRCI] =     { 0x10000000,     0x1000 },
    [SIFIVE_U_DEV_UART0] =    { 0x10010000,     0x1000 },
    [SIFIVE_U_DEV_UART1] =    { 0x10011000,     0x1000 },
    [SIFIVE_U_DEV_PWM0] =     { 0x10020000,     0x1000 },
    [SIFIVE_U_DEV_PWM1] =     { 0x10021000,     0x1000 },
    [SIFIVE_U_DEV_QSPI0] =    { 0x10040000,     0x1000 },
    [SIFIVE_U_DEV_QSPI2] =    { 0x10050000,     0x1000 },
    [SIFIVE_U_DEV_GPIO] =     { 0x10060000,     0x1000 },
    [SIFIVE_U_DEV_OTP] =      { 0x10070000,     0x1000 },
    [SIFIVE_U_DEV_GEM] =      { 0x10090000,     0x2000 },
    [SIFIVE_U_DEV_GEM_MGMT] = { 0x100a0000,     0x1000 },
    [SIFIVE_U_DEV_DMC] =      { 0x100b0000,    0x10000 },
    [SIFIVE_U_DEV_FLASH0] =   { 0x20000000, 0x10000000 },
    [SIFIVE_U_DEV_DRAM] =     { 0x80000000,        0x0 },
    [SIFIVE_U_DEV_PCIE_ECAM] =      { 0x30000000, 256 * MiB },
    [SIFIVE_U_DEV_PCIE_PIO] =       { 0x10080000,  64 * KiB },
    [SIFIVE_U_DEV_PCIE_MMIO] =      { 0x40000000,   1 * GiB },
    [SIFIVE_U_DEV_FW_CFG] =         { 0x10100000,        0x18 },
    [SIFIVE_U_DEV_PCIE_MMIO_HIGH] = { 0x400000000ULL, 16 * GiB },
    [SIFIVE_U_DEV_CXL_HOST_REG] =   { 0x800000000ULL,  1 * MiB },
    [SIFIVE_U_DEV_CXL_FMW] =        { 0x1000000000ULL, 4 * GiB },
};

#define OTP_SERIAL          1
#define GEM_REVISION        0x10070109
#define SIFIVE_U_FDT_PCI_ADDR_CELLS 3
#define SIFIVE_U_FDT_PCI_INT_CELLS 1
#define SIFIVE_U_FDT_INT_MAP_WIDTH \
    (SIFIVE_U_FDT_PCI_ADDR_CELLS + SIFIVE_U_FDT_PCI_INT_CELLS + 2)

static void sifive_u_cxl_irq_map(void *fdt, const char *node,
                                 uint32_t plic_phandle)
{
    uint32_t irq_map[PCI_NUM_PINS * PCI_NUM_PINS *
                     SIFIVE_U_FDT_INT_MAP_WIDTH] = {};
    uint32_t *entry = irq_map;
    int dev;
    int pin;

    for (dev = 0; dev < PCI_NUM_PINS; dev++) {
        int devfn = dev * PCI_FUNC_MAX;

        for (pin = 0; pin < PCI_NUM_PINS; pin++) {
            int irq = SIFIVE_U_PCIE_IRQ_BASE +
                      ((pin + PCI_SLOT(devfn)) % PCI_NUM_PINS);

            entry[0] = cpu_to_be32(devfn << 8);
            entry[SIFIVE_U_FDT_PCI_ADDR_CELLS] = cpu_to_be32(pin + 1);
            entry[SIFIVE_U_FDT_PCI_ADDR_CELLS +
                  SIFIVE_U_FDT_PCI_INT_CELLS] =
                cpu_to_be32(plic_phandle);
            entry[SIFIVE_U_FDT_PCI_ADDR_CELLS +
                  SIFIVE_U_FDT_PCI_INT_CELLS + 1] =
                cpu_to_be32(irq);
            entry += SIFIVE_U_FDT_INT_MAP_WIDTH;
        }
    }

    qemu_fdt_setprop(fdt, node, "interrupt-map", irq_map,
                     (entry - irq_map) * sizeof(*irq_map));
    qemu_fdt_setprop_cells(fdt, node, "interrupt-map-mask",
                           0x1800, 0, 0, 0x7);
}

static void sifive_u_cxl_fdt(SiFiveUState *s, uint32_t plic_phandle)
{
    const MemMapEntry *memmap = sifive_u_memmap;
    void *fdt = MACHINE(s)->fdt;
    g_autofree char *node = NULL;
    hwaddr ecam = memmap[SIFIVE_U_DEV_PCIE_ECAM].base;
    hwaddr pio = memmap[SIFIVE_U_DEV_PCIE_PIO].base;
    hwaddr mmio = memmap[SIFIVE_U_DEV_PCIE_MMIO].base;
    hwaddr cxl_mmio = mmio + memmap[SIFIVE_U_DEV_PCIE_MMIO].size -
                      SIFIVE_U_CXL_MMIO32_SIZE;
    hwaddr high = memmap[SIFIVE_U_DEV_PCIE_MMIO_HIGH].base;
    hwaddr fw_cfg = memmap[SIFIVE_U_DEV_FW_CFG].base;

    node = g_strdup_printf("/soc/pci@%" HWADDR_PRIx, ecam);
    qemu_fdt_add_subnode(fdt, node);
    qemu_fdt_setprop_cell(fdt, node, "#address-cells",
                          SIFIVE_U_FDT_PCI_ADDR_CELLS);
    qemu_fdt_setprop_cell(fdt, node, "#interrupt-cells",
                          SIFIVE_U_FDT_PCI_INT_CELLS);
    qemu_fdt_setprop_cell(fdt, node, "#size-cells", 2);
    qemu_fdt_setprop_string(fdt, node, "compatible",
                            "pci-host-ecam-generic");
    qemu_fdt_setprop_string(fdt, node, "device_type", "pci");
    qemu_fdt_setprop_cell(fdt, node, "linux,pci-domain", 0);
    qemu_fdt_setprop_cells(fdt, node, "bus-range", 0, 0xff);
    qemu_fdt_setprop(fdt, node, "dma-coherent", NULL, 0);
    qemu_fdt_setprop(fdt, node, "qemu,synthetic-cxl-host", NULL, 0);
    qemu_fdt_setprop_u64(fdt, node, "qemu,cxl-host-reg-base",
                         memmap[SIFIVE_U_DEV_CXL_HOST_REG].base);
    qemu_fdt_setprop_u64(fdt, node, "qemu,cxl-host-reg-size",
                         memmap[SIFIVE_U_DEV_CXL_HOST_REG].size);
    qemu_fdt_setprop_u64(fdt, node, "qemu,cxl-fmw-aperture-base",
                         memmap[SIFIVE_U_DEV_CXL_FMW].base);
    qemu_fdt_setprop_u64(fdt, node, "qemu,cxl-fmw-aperture-size",
                         memmap[SIFIVE_U_DEV_CXL_FMW].size);
    qemu_fdt_setprop_sized_cells(fdt, node, "reg",
                                 2, ecam,
                                 2, memmap[SIFIVE_U_DEV_PCIE_ECAM].size);
    qemu_fdt_setprop_sized_cells(
        fdt, node, "ranges",
        1, FDT_PCI_RANGE_IOPORT, 2, 0,
        2, pio, 2, memmap[SIFIVE_U_DEV_PCIE_PIO].size,
        1, FDT_PCI_RANGE_MMIO, 2, cxl_mmio,
        2, cxl_mmio, 2, SIFIVE_U_CXL_MMIO32_SIZE,
        1, FDT_PCI_RANGE_MMIO_64BIT | FDT_PCI_RANGE_PREFETCHABLE, 2, high,
        2, high, 2, memmap[SIFIVE_U_DEV_PCIE_MMIO_HIGH].size);
    sifive_u_cxl_irq_map(fdt, node, plic_phandle);

    g_clear_pointer(&node, g_free);
    node = g_strdup_printf("/soc/fw-cfg@%" HWADDR_PRIx, fw_cfg);
    qemu_fdt_add_subnode(fdt, node);
    qemu_fdt_setprop_string(fdt, node, "compatible", "qemu,fw-cfg-mmio");
    qemu_fdt_setprop_sized_cells(fdt, node, "reg",
                                 2, fw_cfg, 2, 8,
                                 2, fw_cfg + 8, 2, 2,
                                 2, fw_cfg + 16, 2, 8);
}

static void create_fdt(SiFiveUState *s, const MemMapEntry *memmap,
                       bool is_32_bit)
{
    MachineState *ms = MACHINE(s);
    uint64_t mem_size = ms->ram_size;
    void *fdt;
    int cpu;
    uint32_t *cells;
    char *nodename;
    uint32_t plic_phandle, prci_phandle, gpio_phandle, phandle = 1;
    uint32_t hfclk_phandle, rtcclk_phandle, phy_phandle;
    static const char * const ethclk_names[2] = { "pclk", "hclk" };
    static const char * const clint_compat[2] = {
        "sifive,clint0", "riscv,clint0"
    };
    static const char * const plic_compat[2] = {
        "sifive,plic-1.0.0", "riscv,plic0"
    };

    fdt = ms->fdt = create_device_tree(&s->fdt_size);
    if (!fdt) {
        error_report("create_device_tree() failed");
        exit(1);
    }

    qemu_fdt_setprop_string(fdt, "/", "model", "SiFive HiFive Unleashed A00");
    qemu_fdt_setprop_string(fdt, "/", "compatible",
                            "sifive,hifive-unleashed-a00");
    qemu_fdt_setprop_cell(fdt, "/", "#size-cells", 0x2);
    qemu_fdt_setprop_cell(fdt, "/", "#address-cells", 0x2);

    qemu_fdt_add_subnode(fdt, "/soc");
    qemu_fdt_setprop(fdt, "/soc", "ranges", NULL, 0);
    qemu_fdt_setprop_string(fdt, "/soc", "compatible", "simple-bus");
    qemu_fdt_setprop_cell(fdt, "/soc", "#size-cells", 0x2);
    qemu_fdt_setprop_cell(fdt, "/soc", "#address-cells", 0x2);

    hfclk_phandle = phandle++;
    nodename = g_strdup_printf("/hfclk");
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cell(fdt, nodename, "phandle", hfclk_phandle);
    qemu_fdt_setprop_string(fdt, nodename, "clock-output-names", "hfclk");
    qemu_fdt_setprop_cell(fdt, nodename, "clock-frequency",
        SIFIVE_U_HFCLK_FREQ);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", "fixed-clock");
    qemu_fdt_setprop_cell(fdt, nodename, "#clock-cells", 0x0);
    g_free(nodename);

    rtcclk_phandle = phandle++;
    nodename = g_strdup_printf("/rtcclk");
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cell(fdt, nodename, "phandle", rtcclk_phandle);
    qemu_fdt_setprop_string(fdt, nodename, "clock-output-names", "rtcclk");
    qemu_fdt_setprop_cell(fdt, nodename, "clock-frequency",
        SIFIVE_U_RTCCLK_FREQ);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", "fixed-clock");
    qemu_fdt_setprop_cell(fdt, nodename, "#clock-cells", 0x0);
    g_free(nodename);

    nodename = g_strdup_printf("/memory@%lx",
        (long)memmap[SIFIVE_U_DEV_DRAM].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        memmap[SIFIVE_U_DEV_DRAM].base >> 32, memmap[SIFIVE_U_DEV_DRAM].base,
        mem_size >> 32, mem_size);
    qemu_fdt_setprop_string(fdt, nodename, "device_type", "memory");
    g_free(nodename);

    qemu_fdt_add_subnode(fdt, "/cpus");
    qemu_fdt_setprop_cell(fdt, "/cpus", "timebase-frequency",
        CLINT_TIMEBASE_FREQ);
    qemu_fdt_setprop_cell(fdt, "/cpus", "#size-cells", 0x0);
    qemu_fdt_setprop_cell(fdt, "/cpus", "#address-cells", 0x1);

    for (cpu = ms->smp.cpus - 1; cpu >= 0; cpu--) {
        int cpu_phandle = phandle++;
        nodename = g_strdup_printf("/cpus/cpu@%d", cpu);
        char *intc = g_strdup_printf("/cpus/cpu@%d/interrupt-controller", cpu);
        qemu_fdt_add_subnode(fdt, nodename);
        /* cpu 0 is the management hart that does not have mmu */
        if (cpu != 0) {
            if (is_32_bit) {
                qemu_fdt_setprop_string(fdt, nodename, "mmu-type", "riscv,sv32");
            } else {
                qemu_fdt_setprop_string(fdt, nodename, "mmu-type", "riscv,sv48");
            }
            riscv_isa_write_fdt(&s->soc.u_cpus.harts[cpu - 1], fdt, nodename);
        } else {
            riscv_isa_write_fdt(&s->soc.e_cpus.harts[0], fdt, nodename);
        }
        qemu_fdt_setprop_string(fdt, nodename, "compatible", "riscv");
        qemu_fdt_setprop_string(fdt, nodename, "status", "okay");
        qemu_fdt_setprop_cell(fdt, nodename, "reg", cpu);
        qemu_fdt_setprop_string(fdt, nodename, "device_type", "cpu");
        qemu_fdt_add_subnode(fdt, intc);
        qemu_fdt_setprop_cell(fdt, intc, "phandle", cpu_phandle);
        qemu_fdt_setprop_string(fdt, intc, "compatible", "riscv,cpu-intc");
        qemu_fdt_setprop(fdt, intc, "interrupt-controller", NULL, 0);
        qemu_fdt_setprop_cell(fdt, intc, "#interrupt-cells", 1);
        g_free(intc);
        g_free(nodename);
    }

    cells =  g_new0(uint32_t, ms->smp.cpus * 4);
    for (cpu = 0; cpu < ms->smp.cpus; cpu++) {
        nodename =
            g_strdup_printf("/cpus/cpu@%d/interrupt-controller", cpu);
        uint32_t intc_phandle = qemu_fdt_get_phandle(fdt, nodename);
        cells[cpu * 4 + 0] = cpu_to_be32(intc_phandle);
        cells[cpu * 4 + 1] = cpu_to_be32(IRQ_M_SOFT);
        cells[cpu * 4 + 2] = cpu_to_be32(intc_phandle);
        cells[cpu * 4 + 3] = cpu_to_be32(IRQ_M_TIMER);
        g_free(nodename);
    }
    nodename = g_strdup_printf("/soc/clint@%lx",
        (long)memmap[SIFIVE_U_DEV_CLINT].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_string_array(fdt, nodename, "compatible",
        (char **)&clint_compat, ARRAY_SIZE(clint_compat));
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[SIFIVE_U_DEV_CLINT].base,
        0x0, memmap[SIFIVE_U_DEV_CLINT].size);
    qemu_fdt_setprop(fdt, nodename, "interrupts-extended",
        cells, ms->smp.cpus * sizeof(uint32_t) * 4);
    g_free(cells);
    g_free(nodename);

    nodename = g_strdup_printf("/soc/otp@%lx",
        (long)memmap[SIFIVE_U_DEV_OTP].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cell(fdt, nodename, "fuse-count", SIFIVE_U_OTP_REG_SIZE);
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[SIFIVE_U_DEV_OTP].base,
        0x0, memmap[SIFIVE_U_DEV_OTP].size);
    qemu_fdt_setprop_string(fdt, nodename, "compatible",
        "sifive,fu540-c000-otp");
    g_free(nodename);

    prci_phandle = phandle++;
    nodename = g_strdup_printf("/soc/clock-controller@%lx",
        (long)memmap[SIFIVE_U_DEV_PRCI].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cell(fdt, nodename, "phandle", prci_phandle);
    qemu_fdt_setprop_cell(fdt, nodename, "#clock-cells", 0x1);
    qemu_fdt_setprop_cells(fdt, nodename, "clocks",
        hfclk_phandle, rtcclk_phandle);
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[SIFIVE_U_DEV_PRCI].base,
        0x0, memmap[SIFIVE_U_DEV_PRCI].size);
    qemu_fdt_setprop_string(fdt, nodename, "compatible",
        "sifive,fu540-c000-prci");
    g_free(nodename);

    plic_phandle = phandle++;
    cells =  g_new0(uint32_t, ms->smp.cpus * 4 - 2);
    for (cpu = 0; cpu < ms->smp.cpus; cpu++) {
        nodename =
            g_strdup_printf("/cpus/cpu@%d/interrupt-controller", cpu);
        uint32_t intc_phandle = qemu_fdt_get_phandle(fdt, nodename);
        /* cpu 0 is the management hart that does not have S-mode */
        if (cpu == 0) {
            cells[0] = cpu_to_be32(intc_phandle);
            cells[1] = cpu_to_be32(IRQ_M_EXT);
        } else {
            cells[cpu * 4 - 2] = cpu_to_be32(intc_phandle);
            cells[cpu * 4 - 1] = cpu_to_be32(IRQ_M_EXT);
            cells[cpu * 4 + 0] = cpu_to_be32(intc_phandle);
            cells[cpu * 4 + 1] = cpu_to_be32(IRQ_S_EXT);
        }
        g_free(nodename);
    }
    nodename = g_strdup_printf("/soc/interrupt-controller@%lx",
        (long)memmap[SIFIVE_U_DEV_PLIC].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cell(fdt, nodename, "#interrupt-cells", 1);
    qemu_fdt_setprop_string_array(fdt, nodename, "compatible",
        (char **)&plic_compat, ARRAY_SIZE(plic_compat));
    qemu_fdt_setprop(fdt, nodename, "interrupt-controller", NULL, 0);
    qemu_fdt_setprop(fdt, nodename, "interrupts-extended",
        cells, (ms->smp.cpus * 4 - 2) * sizeof(uint32_t));
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[SIFIVE_U_DEV_PLIC].base,
        0x0, memmap[SIFIVE_U_DEV_PLIC].size);
    qemu_fdt_setprop_cell(fdt, nodename, "riscv,ndev",
                          SIFIVE_U_PLIC_NUM_SOURCES - 1);
    qemu_fdt_setprop_cell(fdt, nodename, "phandle", plic_phandle);
    plic_phandle = qemu_fdt_get_phandle(fdt, nodename);
    g_free(cells);
    g_free(nodename);

    gpio_phandle = phandle++;
    nodename = g_strdup_printf("/soc/gpio@%lx",
        (long)memmap[SIFIVE_U_DEV_GPIO].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cell(fdt, nodename, "phandle", gpio_phandle);
    qemu_fdt_setprop_cells(fdt, nodename, "clocks",
        prci_phandle, PRCI_CLK_TLCLK);
    qemu_fdt_setprop_cell(fdt, nodename, "#interrupt-cells", 2);
    qemu_fdt_setprop(fdt, nodename, "interrupt-controller", NULL, 0);
    qemu_fdt_setprop_cell(fdt, nodename, "#gpio-cells", 2);
    qemu_fdt_setprop(fdt, nodename, "gpio-controller", NULL, 0);
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[SIFIVE_U_DEV_GPIO].base,
        0x0, memmap[SIFIVE_U_DEV_GPIO].size);
    qemu_fdt_setprop_cells(fdt, nodename, "interrupts", SIFIVE_U_GPIO_IRQ0,
        SIFIVE_U_GPIO_IRQ1, SIFIVE_U_GPIO_IRQ2, SIFIVE_U_GPIO_IRQ3,
        SIFIVE_U_GPIO_IRQ4, SIFIVE_U_GPIO_IRQ5, SIFIVE_U_GPIO_IRQ6,
        SIFIVE_U_GPIO_IRQ7, SIFIVE_U_GPIO_IRQ8, SIFIVE_U_GPIO_IRQ9,
        SIFIVE_U_GPIO_IRQ10, SIFIVE_U_GPIO_IRQ11, SIFIVE_U_GPIO_IRQ12,
        SIFIVE_U_GPIO_IRQ13, SIFIVE_U_GPIO_IRQ14, SIFIVE_U_GPIO_IRQ15);
    qemu_fdt_setprop_cell(fdt, nodename, "interrupt-parent", plic_phandle);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", "sifive,gpio0");
    g_free(nodename);

    nodename = g_strdup_printf("/gpio-restart");
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cells(fdt, nodename, "gpios", gpio_phandle, 10, 1);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", "gpio-restart");
    g_free(nodename);

    nodename = g_strdup_printf("/soc/dma@%lx",
        (long)memmap[SIFIVE_U_DEV_PDMA].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cell(fdt, nodename, "#dma-cells", 1);
    qemu_fdt_setprop_cells(fdt, nodename, "interrupts",
        SIFIVE_U_PDMA_IRQ0, SIFIVE_U_PDMA_IRQ1, SIFIVE_U_PDMA_IRQ2,
        SIFIVE_U_PDMA_IRQ3, SIFIVE_U_PDMA_IRQ4, SIFIVE_U_PDMA_IRQ5,
        SIFIVE_U_PDMA_IRQ6, SIFIVE_U_PDMA_IRQ7);
    qemu_fdt_setprop_cell(fdt, nodename, "interrupt-parent", plic_phandle);
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[SIFIVE_U_DEV_PDMA].base,
        0x0, memmap[SIFIVE_U_DEV_PDMA].size);
    qemu_fdt_setprop_string(fdt, nodename, "compatible",
                            "sifive,fu540-c000-pdma");
    g_free(nodename);

    nodename = g_strdup_printf("/soc/cache-controller@%lx",
        (long)memmap[SIFIVE_U_DEV_L2CC].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[SIFIVE_U_DEV_L2CC].base,
        0x0, memmap[SIFIVE_U_DEV_L2CC].size);
    qemu_fdt_setprop_cells(fdt, nodename, "interrupts",
        SIFIVE_U_L2CC_IRQ0, SIFIVE_U_L2CC_IRQ1, SIFIVE_U_L2CC_IRQ2);
    qemu_fdt_setprop_cell(fdt, nodename, "interrupt-parent", plic_phandle);
    qemu_fdt_setprop(fdt, nodename, "cache-unified", NULL, 0);
    qemu_fdt_setprop_cell(fdt, nodename, "cache-size", 2097152);
    qemu_fdt_setprop_cell(fdt, nodename, "cache-sets", 1024);
    qemu_fdt_setprop_cell(fdt, nodename, "cache-level", 2);
    qemu_fdt_setprop_cell(fdt, nodename, "cache-block-size", 64);
    qemu_fdt_setprop_string(fdt, nodename, "compatible",
                            "sifive,fu540-c000-ccache");
    g_free(nodename);

    nodename = g_strdup_printf("/soc/spi@%lx",
        (long)memmap[SIFIVE_U_DEV_QSPI2].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cell(fdt, nodename, "#size-cells", 0);
    qemu_fdt_setprop_cell(fdt, nodename, "#address-cells", 1);
    qemu_fdt_setprop_cells(fdt, nodename, "clocks",
        prci_phandle, PRCI_CLK_TLCLK);
    qemu_fdt_setprop_cell(fdt, nodename, "interrupts", SIFIVE_U_QSPI2_IRQ);
    qemu_fdt_setprop_cell(fdt, nodename, "interrupt-parent", plic_phandle);
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[SIFIVE_U_DEV_QSPI2].base,
        0x0, memmap[SIFIVE_U_DEV_QSPI2].size);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", "sifive,spi0");
    g_free(nodename);

    nodename = g_strdup_printf("/soc/spi@%lx/mmc@0",
        (long)memmap[SIFIVE_U_DEV_QSPI2].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop(fdt, nodename, "disable-wp", NULL, 0);
    qemu_fdt_setprop_cells(fdt, nodename, "voltage-ranges", 3300, 3300);
    qemu_fdt_setprop_cell(fdt, nodename, "spi-max-frequency", 20000000);
    qemu_fdt_setprop_cell(fdt, nodename, "reg", 0);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", "mmc-spi-slot");
    g_free(nodename);

    nodename = g_strdup_printf("/soc/spi@%lx",
        (long)memmap[SIFIVE_U_DEV_QSPI0].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cell(fdt, nodename, "#size-cells", 0);
    qemu_fdt_setprop_cell(fdt, nodename, "#address-cells", 1);
    qemu_fdt_setprop_cells(fdt, nodename, "clocks",
        prci_phandle, PRCI_CLK_TLCLK);
    qemu_fdt_setprop_cell(fdt, nodename, "interrupts", SIFIVE_U_QSPI0_IRQ);
    qemu_fdt_setprop_cell(fdt, nodename, "interrupt-parent", plic_phandle);
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[SIFIVE_U_DEV_QSPI0].base,
        0x0, memmap[SIFIVE_U_DEV_QSPI0].size);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", "sifive,spi0");
    g_free(nodename);

    nodename = g_strdup_printf("/soc/spi@%lx/flash@0",
        (long)memmap[SIFIVE_U_DEV_QSPI0].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cell(fdt, nodename, "spi-rx-bus-width", 4);
    qemu_fdt_setprop_cell(fdt, nodename, "spi-tx-bus-width", 4);
    qemu_fdt_setprop(fdt, nodename, "m25p,fast-read", NULL, 0);
    qemu_fdt_setprop_cell(fdt, nodename, "spi-max-frequency", 50000000);
    qemu_fdt_setprop_cell(fdt, nodename, "reg", 0);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", "jedec,spi-nor");
    g_free(nodename);

    phy_phandle = phandle++;
    nodename = g_strdup_printf("/soc/ethernet@%lx",
        (long)memmap[SIFIVE_U_DEV_GEM].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_string(fdt, nodename, "compatible",
        "sifive,fu540-c000-gem");
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[SIFIVE_U_DEV_GEM].base,
        0x0, memmap[SIFIVE_U_DEV_GEM].size,
        0x0, memmap[SIFIVE_U_DEV_GEM_MGMT].base,
        0x0, memmap[SIFIVE_U_DEV_GEM_MGMT].size);
    qemu_fdt_setprop_string(fdt, nodename, "reg-names", "control");
    qemu_fdt_setprop_string(fdt, nodename, "phy-mode", "gmii");
    qemu_fdt_setprop_cell(fdt, nodename, "phy-handle", phy_phandle);
    qemu_fdt_setprop_cell(fdt, nodename, "interrupt-parent", plic_phandle);
    qemu_fdt_setprop_cell(fdt, nodename, "interrupts", SIFIVE_U_GEM_IRQ);
    qemu_fdt_setprop_cells(fdt, nodename, "clocks",
        prci_phandle, PRCI_CLK_GEMGXLPLL, prci_phandle, PRCI_CLK_GEMGXLPLL);
    qemu_fdt_setprop_string_array(fdt, nodename, "clock-names",
        (char **)&ethclk_names, ARRAY_SIZE(ethclk_names));
    qemu_fdt_setprop(fdt, nodename, "local-mac-address",
        s->soc.gem.conf.macaddr.a, ETH_ALEN);
    qemu_fdt_setprop_cell(fdt, nodename, "#address-cells", 1);
    qemu_fdt_setprop_cell(fdt, nodename, "#size-cells", 0);

    qemu_fdt_add_subnode(fdt, "/aliases");
    qemu_fdt_setprop_string(fdt, "/aliases", "ethernet0", nodename);

    g_free(nodename);

    nodename = g_strdup_printf("/soc/ethernet@%lx/ethernet-phy@0",
        (long)memmap[SIFIVE_U_DEV_GEM].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cell(fdt, nodename, "phandle", phy_phandle);
    qemu_fdt_setprop_cell(fdt, nodename, "reg", 0x0);
    g_free(nodename);

    nodename = g_strdup_printf("/soc/pwm@%lx",
        (long)memmap[SIFIVE_U_DEV_PWM0].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", "sifive,pwm0");
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[SIFIVE_U_DEV_PWM0].base,
        0x0, memmap[SIFIVE_U_DEV_PWM0].size);
    qemu_fdt_setprop_cell(fdt, nodename, "interrupt-parent", plic_phandle);
    qemu_fdt_setprop_cells(fdt, nodename, "interrupts",
                           SIFIVE_U_PWM0_IRQ0, SIFIVE_U_PWM0_IRQ1,
                           SIFIVE_U_PWM0_IRQ2, SIFIVE_U_PWM0_IRQ3);
    qemu_fdt_setprop_cells(fdt, nodename, "clocks",
                           prci_phandle, PRCI_CLK_TLCLK);
    qemu_fdt_setprop_cell(fdt, nodename, "#pwm-cells", 0);
    g_free(nodename);

    nodename = g_strdup_printf("/soc/pwm@%lx",
        (long)memmap[SIFIVE_U_DEV_PWM1].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", "sifive,pwm0");
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[SIFIVE_U_DEV_PWM1].base,
        0x0, memmap[SIFIVE_U_DEV_PWM1].size);
    qemu_fdt_setprop_cell(fdt, nodename, "interrupt-parent", plic_phandle);
    qemu_fdt_setprop_cells(fdt, nodename, "interrupts",
                           SIFIVE_U_PWM1_IRQ0, SIFIVE_U_PWM1_IRQ1,
                           SIFIVE_U_PWM1_IRQ2, SIFIVE_U_PWM1_IRQ3);
    qemu_fdt_setprop_cells(fdt, nodename, "clocks",
                           prci_phandle, PRCI_CLK_TLCLK);
    qemu_fdt_setprop_cell(fdt, nodename, "#pwm-cells", 0);
    g_free(nodename);

    nodename = g_strdup_printf("/soc/serial@%lx",
        (long)memmap[SIFIVE_U_DEV_UART1].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", "sifive,uart0");
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[SIFIVE_U_DEV_UART1].base,
        0x0, memmap[SIFIVE_U_DEV_UART1].size);
    qemu_fdt_setprop_cells(fdt, nodename, "clocks",
        prci_phandle, PRCI_CLK_TLCLK);
    qemu_fdt_setprop_cell(fdt, nodename, "interrupt-parent", plic_phandle);
    qemu_fdt_setprop_cell(fdt, nodename, "interrupts", SIFIVE_U_UART1_IRQ);

    qemu_fdt_setprop_string(fdt, "/aliases", "serial1", nodename);
    g_free(nodename);

    nodename = g_strdup_printf("/soc/serial@%lx",
        (long)memmap[SIFIVE_U_DEV_UART0].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", "sifive,uart0");
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[SIFIVE_U_DEV_UART0].base,
        0x0, memmap[SIFIVE_U_DEV_UART0].size);
    qemu_fdt_setprop_cells(fdt, nodename, "clocks",
        prci_phandle, PRCI_CLK_TLCLK);
    qemu_fdt_setprop_cell(fdt, nodename, "interrupt-parent", plic_phandle);
    qemu_fdt_setprop_cell(fdt, nodename, "interrupts", SIFIVE_U_UART0_IRQ);

    qemu_fdt_add_subnode(fdt, "/chosen");
    qemu_fdt_setprop_string(fdt, "/chosen", "stdout-path", nodename);
    qemu_fdt_setprop_string(fdt, "/aliases", "serial0", nodename);

    g_free(nodename);

    if (s->cxl_devices_state.is_enabled) {
        sifive_u_cxl_fdt(s, plic_phandle);
    }
}

static void sifive_u_machine_reset(void *opaque, int n, int level)
{
    /* gpio pin active low triggers reset */
    if (!level) {
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
    }
}

static void sifive_u_validate_cxl_map(SiFiveUState *s)
{
    MachineState *ms = MACHINE(s);
    hwaddr ram_start = sifive_u_memmap[SIFIVE_U_DEV_DRAM].base;
    hwaddr ram_end;

    if (uadd64_overflow(ram_start, ms->ram_size, &ram_end) ||
        ram_end > sifive_u_memmap[SIFIVE_U_DEV_PCIE_MMIO_HIGH].base) {
        error_report("sifive_u CXL: RAM overlaps synthetic PCI MMIO64");
        exit(EXIT_FAILURE);
    }
}

static void sifive_u_create_gpex(SiFiveUState *s)
{
    const MemMapEntry *memmap = sifive_u_memmap;
    DeviceState *dev = qdev_new(TYPE_GPEX_HOST);
    MemoryRegion *ecam_alias = g_new0(MemoryRegion, 1);
    MemoryRegion *mmio_alias = g_new0(MemoryRegion, 1);
    MemoryRegion *high_alias = g_new0(MemoryRegion, 1);
    MemoryRegion *ecam;
    MemoryRegion *mmio;
    int i;

    object_property_set_uint(OBJECT(dev), PCI_HOST_ECAM_BASE,
                             memmap[SIFIVE_U_DEV_PCIE_ECAM].base, NULL);
    object_property_set_int(OBJECT(dev), PCI_HOST_ECAM_SIZE,
                            memmap[SIFIVE_U_DEV_PCIE_ECAM].size, NULL);
    object_property_set_uint(OBJECT(dev), PCI_HOST_PIO_BASE,
                             memmap[SIFIVE_U_DEV_PCIE_PIO].base, NULL);
    object_property_set_int(OBJECT(dev), PCI_HOST_PIO_SIZE,
                            memmap[SIFIVE_U_DEV_PCIE_PIO].size, NULL);
    object_property_set_uint(OBJECT(dev), PCI_HOST_BELOW_4G_MMIO_BASE,
                             memmap[SIFIVE_U_DEV_PCIE_MMIO].base, NULL);
    object_property_set_int(
        OBJECT(dev), PCI_HOST_BELOW_4G_MMIO_SIZE,
        memmap[SIFIVE_U_DEV_PCIE_MMIO].size - SIFIVE_U_CXL_MMIO32_SIZE,
        NULL);
    object_property_set_uint(OBJECT(dev), PCI_HOST_ABOVE_4G_MMIO_BASE,
                             memmap[SIFIVE_U_DEV_PCIE_MMIO_HIGH].base, NULL);
    object_property_set_int(OBJECT(dev), PCI_HOST_ABOVE_4G_MMIO_SIZE,
                            memmap[SIFIVE_U_DEV_PCIE_MMIO_HIGH].size, NULL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    ecam = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
    memory_region_init_alias(ecam_alias, OBJECT(dev), "sifive-u-pcie-ecam",
                             ecam, 0,
                             memmap[SIFIVE_U_DEV_PCIE_ECAM].size);
    memory_region_add_subregion(
        get_system_memory(), memmap[SIFIVE_U_DEV_PCIE_ECAM].base, ecam_alias);

    mmio = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 1);
    memory_region_init_alias(
        mmio_alias, OBJECT(dev), "sifive-u-pcie-mmio", mmio,
        memmap[SIFIVE_U_DEV_PCIE_MMIO].base,
        memmap[SIFIVE_U_DEV_PCIE_MMIO].size);
    memory_region_add_subregion(
        get_system_memory(), memmap[SIFIVE_U_DEV_PCIE_MMIO].base, mmio_alias);

    memory_region_init_alias(
        high_alias, OBJECT(dev), "sifive-u-pcie-mmio-high", mmio,
        memmap[SIFIVE_U_DEV_PCIE_MMIO_HIGH].base,
        memmap[SIFIVE_U_DEV_PCIE_MMIO_HIGH].size);
    memory_region_add_subregion(get_system_memory(),
        memmap[SIFIVE_U_DEV_PCIE_MMIO_HIGH].base, high_alias);

    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 2,
                    memmap[SIFIVE_U_DEV_PCIE_PIO].base);
    for (i = 0; i < SIFIVE_U_PCIE_IRQ_COUNT; i++) {
        sysbus_connect_irq(
            SYS_BUS_DEVICE(dev), i,
            qdev_get_gpio_in(DEVICE(s->soc.plic),
                             SIFIVE_U_PCIE_IRQ_BASE + i));
        gpex_set_irq_num(GPEX_HOST(dev), i, SIFIVE_U_PCIE_IRQ_BASE + i);
    }

    s->gpex_host = GPEX_HOST(dev);
    s->pci_bus = PCI_HOST_BRIDGE(dev)->bus;
    s->gpex_host->gpex_cfg = (struct GPEXConfig) {
        .ecam = memmap[SIFIVE_U_DEV_PCIE_ECAM],
        .mmio32 = {
            .base = memmap[SIFIVE_U_DEV_PCIE_MMIO].base,
            .size = memmap[SIFIVE_U_DEV_PCIE_MMIO].size -
                    SIFIVE_U_CXL_MMIO32_SIZE,
        },
        .cxl_mmio32 = {
            .base = memmap[SIFIVE_U_DEV_PCIE_MMIO].base +
                    memmap[SIFIVE_U_DEV_PCIE_MMIO].size -
                    SIFIVE_U_CXL_MMIO32_SIZE,
            .size = SIFIVE_U_CXL_MMIO32_SIZE,
        },
        .mmio64 = {
            .base = memmap[SIFIVE_U_DEV_PCIE_MMIO_HIGH].base +
                    SIFIVE_U_CXL_MMIO64_SIZE,
            .size = memmap[SIFIVE_U_DEV_PCIE_MMIO_HIGH].size -
                    SIFIVE_U_CXL_MMIO64_SIZE,
        },
        .cxl_mmio64 = {
            .base = memmap[SIFIVE_U_DEV_PCIE_MMIO_HIGH].base,
            .size = SIFIVE_U_CXL_MMIO64_SIZE,
        },
        .pio = memmap[SIFIVE_U_DEV_PCIE_PIO],
        .irq = SIFIVE_U_PCIE_IRQ_BASE,
        .bus = s->pci_bus,
        .pci_native_hotplug = true,
    };
}

static void sifive_u_create_fw_cfg(SiFiveUState *s)
{
    MachineState *ms = MACHINE(s);
    hwaddr base = sifive_u_memmap[SIFIVE_U_DEV_FW_CFG].base;

    s->fw_cfg = fw_cfg_init_mem_wide(base + 8, base, 8, base + 16,
                                     &address_space_memory);
    fw_cfg_add_i16(s->fw_cfg, FW_CFG_NB_CPUS, ms->smp.cpus);
    rom_set_fw(s->fw_cfg);
}

static void sifive_u_create_cxl_regions(SiFiveUState *s)
{
    CXLState *cxl = &s->cxl_devices_state;
    CXLFixedMemoryWindowOptionsList *it;
    MemoryRegion *sysmem = get_system_memory();
    hwaddr fmw_size = sifive_u_memmap[SIFIVE_U_DEV_CXL_FMW].size;
    hwaddr configured_size = 0;

    if (!cxl->is_enabled) {
        return;
    }
    cxl->firmware_handoff = true;

    for (it = cxl->cfmw_list; it; it = it->next) {
        if (uadd64_overflow(configured_size, it->value->size,
                            &configured_size) ||
            configured_size > fmw_size) {
            error_report(
                "sifive_u CXL: fixed windows exceed 4 GiB aperture");
            exit(EXIT_FAILURE);
        }
    }

    memory_region_init(&cxl->host_mr, OBJECT(s), "sifive-u-cxl-host-reg",
                       sifive_u_memmap[SIFIVE_U_DEV_CXL_HOST_REG].size);
    memory_region_add_subregion(
        sysmem, sifive_u_memmap[SIFIVE_U_DEV_CXL_HOST_REG].base,
        &cxl->host_mr);
    cxl_fmws_set_memmap(sifive_u_memmap[SIFIVE_U_DEV_CXL_FMW].base,
                        sifive_u_memmap[SIFIVE_U_DEV_CXL_FMW].base + fmw_size);
    cxl_fmws_update_mmio();
}

static void sifive_u_machine_done(Notifier *notifier, void *data)
{
    SiFiveUState *s = container_of(notifier, SiFiveUState, machine_done);

    cxl_hook_up_pxb_registers(s->pci_bus, &s->cxl_devices_state,
                              &error_fatal);
    if (s->cxl_devices_state.is_enabled) {
        cxl_fmws_link_targets(&error_fatal);
        sifive_u_acpi_setup(s);
    }
}

static void sifive_u_machine_init(MachineState *machine)
{
    const MemMapEntry *memmap = sifive_u_memmap;
    SiFiveUState *s = RISCV_U_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *flash0 = g_new(MemoryRegion, 1);
    hwaddr start_addr = memmap[SIFIVE_U_DEV_DRAM].base;
    target_ulong firmware_end_addr, kernel_start_addr;
    const char *firmware_name;
    uint32_t start_addr_hi32 = 0x00000000;
    uint32_t fdt_load_addr_hi32 = 0x00000000;
    int i;
    uint64_t fdt_load_addr;
    uint64_t kernel_entry;
    DriveInfo *dinfo;
    BlockBackend *blk;
    DeviceState *flash_dev, *sd_dev, *card_dev;
    qemu_irq flash_cs, sd_cs;
    RISCVBootInfo boot_info;

    if (s->cxl_devices_state.is_enabled) {
        sifive_u_validate_cxl_map(s);
    }

    /* Initialize SoC */
    object_initialize_child(OBJECT(machine), "soc", &s->soc, TYPE_RISCV_U_SOC);
    object_property_set_uint(OBJECT(&s->soc), "serial", s->serial,
                             &error_abort);
    object_property_set_str(OBJECT(&s->soc), "cpu-type", machine->cpu_type,
                             &error_abort);
    qdev_realize(DEVICE(&s->soc), NULL, &error_fatal);

    if (s->cxl_devices_state.is_enabled) {
        sifive_u_create_fw_cfg(s);
        sifive_u_create_gpex(s);
        sifive_u_create_cxl_regions(s);
        s->machine_done.notify = sifive_u_machine_done;
        qemu_add_machine_init_done_notifier(&s->machine_done);
    }

    /* register RAM */
    memory_region_add_subregion(system_memory, memmap[SIFIVE_U_DEV_DRAM].base,
                                machine->ram);

    /* register QSPI0 Flash */
    memory_region_init_ram(flash0, NULL, "riscv.sifive.u.flash0",
                           memmap[SIFIVE_U_DEV_FLASH0].size, &error_fatal);
    memory_region_add_subregion(system_memory, memmap[SIFIVE_U_DEV_FLASH0].base,
                                flash0);

    /* register gpio-restart */
    qdev_connect_gpio_out(DEVICE(&(s->soc.gpio)), 10,
                          qemu_allocate_irq(sifive_u_machine_reset, NULL, 0));

    /* load/create device tree */
    if (machine->dtb) {
        machine->fdt = load_device_tree(machine->dtb, &s->fdt_size);
        if (!machine->fdt) {
            error_report("load_device_tree() failed");
            exit(1);
        }
    } else {
        create_fdt(s, memmap, riscv_is_32bit(&s->soc.u_cpus));
    }

    if (s->start_in_flash) {
        /*
         * If start_in_flash property is given, assign s->msel to a value
         * that representing booting from QSPI0 memory-mapped flash.
         *
         * This also means that when both start_in_flash and msel properties
         * are given, start_in_flash takes the precedence over msel.
         *
         * Note this is to keep backward compatibility not to break existing
         * users that use start_in_flash property.
         */
        s->msel = MSEL_MEMMAP_QSPI0_FLASH;
    }

    switch (s->msel) {
    case MSEL_MEMMAP_QSPI0_FLASH:
        start_addr = memmap[SIFIVE_U_DEV_FLASH0].base;
        break;
    case MSEL_L2LIM_QSPI0_FLASH:
    case MSEL_L2LIM_QSPI2_SD:
        start_addr = memmap[SIFIVE_U_DEV_L2LIM].base;
        break;
    default:
        start_addr = memmap[SIFIVE_U_DEV_DRAM].base;
        break;
    }

    firmware_name = riscv_default_firmware_name(&s->soc.u_cpus);
    firmware_end_addr = riscv_find_and_load_firmware(machine, firmware_name,
                                                     &start_addr, NULL);

    riscv_boot_info_init(&boot_info, &s->soc.u_cpus);
    if (machine->kernel_filename) {
        kernel_start_addr = riscv_calc_kernel_start_addr(&boot_info,
                                                         firmware_end_addr);
        riscv_load_kernel(machine, &boot_info, kernel_start_addr,
                          true, NULL);
        kernel_entry = boot_info.image_low_addr;
    } else {
       /*
        * If dynamic firmware is used, it doesn't know where is the next mode
        * if kernel argument is not set.
        */
        kernel_entry = 0;
    }

    fdt_load_addr = riscv_compute_fdt_addr(memmap[SIFIVE_U_DEV_DRAM].base,
                                           memmap[SIFIVE_U_DEV_DRAM].size,
                                           machine, &boot_info);
    riscv_load_fdt(fdt_load_addr, machine->fdt);

    if (!riscv_is_32bit(&s->soc.u_cpus)) {
        start_addr_hi32 = (uint64_t)start_addr >> 32;
        fdt_load_addr_hi32 = fdt_load_addr >> 32;
    }

    /* reset vector */
    uint32_t reset_vec[12] = {
        s->msel,                       /* MSEL pin state */
        0x00000297,                    /* 1:  auipc  t0, %pcrel_hi(fw_dyn) */
        0x02c28613,                    /*     addi   a2, t0, %pcrel_lo(1b) */
        0xf1402573,                    /*     csrr   a0, mhartid  */
        0,
        0,
        0x00028067,                    /*     jr     t0 */
        start_addr,                    /* start: .dword */
        start_addr_hi32,
        fdt_load_addr,                 /* fdt_laddr: .dword */
        fdt_load_addr_hi32,
        0x00000000,
                                       /* fw_dyn: */
    };
    if (riscv_is_32bit(&s->soc.u_cpus)) {
        reset_vec[4] = 0x0202a583;     /*     lw     a1, 32(t0) */
        reset_vec[5] = 0x0182a283;     /*     lw     t0, 24(t0) */
    } else {
        reset_vec[4] = 0x0202b583;     /*     ld     a1, 32(t0) */
        reset_vec[5] = 0x0182b283;     /*     ld     t0, 24(t0) */
    }


    /* copy in the reset vector in little_endian byte order */
    for (i = 0; i < ARRAY_SIZE(reset_vec); i++) {
        reset_vec[i] = cpu_to_le32(reset_vec[i]);
    }
    rom_add_blob_fixed_as("mrom.reset", reset_vec, sizeof(reset_vec),
                          memmap[SIFIVE_U_DEV_MROM].base, &address_space_memory);

    riscv_rom_copy_firmware_info(machine, &s->soc.u_cpus,
                                 memmap[SIFIVE_U_DEV_MROM].base,
                                 memmap[SIFIVE_U_DEV_MROM].size,
                                 sizeof(reset_vec), kernel_entry);

    /* Connect an SPI flash to SPI0 */
    flash_dev = qdev_new("is25wp256");
    dinfo = drive_get(IF_MTD, 0, 0);
    if (dinfo) {
        qdev_prop_set_drive_err(flash_dev, "drive",
                                blk_by_legacy_dinfo(dinfo),
                                &error_fatal);
    }
    qdev_realize_and_unref(flash_dev, BUS(s->soc.spi0.spi), &error_fatal);

    flash_cs = qdev_get_gpio_in_named(flash_dev, SSI_GPIO_CS, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->soc.spi0), 1, flash_cs);

    /* Connect an SD card to SPI2 */
    sd_dev = ssi_create_peripheral(s->soc.spi2.spi, "ssi-sd");

    sd_cs = qdev_get_gpio_in_named(sd_dev, SSI_GPIO_CS, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->soc.spi2), 1, sd_cs);

    dinfo = drive_get(IF_SD, 0, 0);
    blk = dinfo ? blk_by_legacy_dinfo(dinfo) : NULL;
    card_dev = qdev_new(TYPE_SD_CARD_SPI);
    qdev_prop_set_drive_err(card_dev, "drive", blk, &error_fatal);
    qdev_realize_and_unref(card_dev,
                           qdev_get_child_bus(sd_dev, "sd-bus"),
                           &error_fatal);
}

static bool sifive_u_machine_get_start_in_flash(Object *obj, Error **errp)
{
    SiFiveUState *s = RISCV_U_MACHINE(obj);

    return s->start_in_flash;
}

static void sifive_u_machine_set_start_in_flash(Object *obj, bool value, Error **errp)
{
    SiFiveUState *s = RISCV_U_MACHINE(obj);

    s->start_in_flash = value;
}

static void sifive_u_machine_instance_init(Object *obj)
{
    SiFiveUState *s = RISCV_U_MACHINE(obj);

    s->start_in_flash = false;
    s->msel = 0;
    cxl_machine_init(obj, &s->cxl_devices_state);
    object_property_add_uint32_ptr(obj, "msel", &s->msel,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "msel",
                                    "Mode Select (MSEL[3:0]) pin state");

    s->serial = OTP_SERIAL;
    object_property_add_uint32_ptr(obj, "serial", &s->serial,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_set_description(obj, "serial", "Board serial number");
}

static void sifive_u_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "RISC-V Board compatible with SiFive U SDK";
    mc->init = sifive_u_machine_init;
    mc->max_cpus = SIFIVE_U_MANAGEMENT_CPU_COUNT + SIFIVE_U_COMPUTE_CPU_COUNT;
    mc->min_cpus = SIFIVE_U_MANAGEMENT_CPU_COUNT + 1;
    mc->default_cpu_type = SIFIVE_U_CPU;
    mc->default_cpus = mc->min_cpus;
    mc->possible_cpu_arch_ids = riscv_numa_possible_cpu_arch_ids;
    mc->cpu_index_to_instance_props = riscv_numa_cpu_index_to_props;
    mc->get_default_cpu_node_id = riscv_numa_get_default_cpu_node_id;
    mc->numa_mem_supported = true;
    mc->cpu_cluster_has_numa_boundary = true;
    mc->default_ram_id = "riscv.sifive.u.ram";
    mc->auto_create_sdcard = true;

    object_class_property_add_bool(oc, "start-in-flash",
                                   sifive_u_machine_get_start_in_flash,
                                   sifive_u_machine_set_start_in_flash);
    object_class_property_set_description(oc, "start-in-flash",
                                          "Set on to tell QEMU's ROM to jump to "
                                          "flash. Otherwise QEMU will jump to DRAM "
                                          "or L2LIM depending on the msel value");
}

static const TypeInfo sifive_u_machine_typeinfo = {
    .name       = MACHINE_TYPE_NAME("sifive_u"),
    .parent     = TYPE_MACHINE,
    .class_init = sifive_u_machine_class_init,
    .instance_init = sifive_u_machine_instance_init,
    .instance_size = sizeof(SiFiveUState),
};

static void sifive_u_machine_init_register_types(void)
{
    type_register_static(&sifive_u_machine_typeinfo);
}

type_init(sifive_u_machine_init_register_types)

static void sifive_u_soc_instance_init(Object *obj)
{
    SiFiveUSoCState *s = RISCV_U_SOC(obj);

    object_initialize_child(obj, "e-cluster", &s->e_cluster, TYPE_CPU_CLUSTER);
    qdev_prop_set_uint32(DEVICE(&s->e_cluster), "cluster-id", 0);

    object_initialize_child(OBJECT(&s->e_cluster), "e-cpus", &s->e_cpus,
                            TYPE_RISCV_HART_ARRAY);
    qdev_prop_set_uint32(DEVICE(&s->e_cpus), "num-harts", 1);
    qdev_prop_set_uint32(DEVICE(&s->e_cpus), "hartid-base", 0);
    qdev_prop_set_string(DEVICE(&s->e_cpus), "cpu-type", SIFIVE_E_CPU);
    qdev_prop_set_uint64(DEVICE(&s->e_cpus), "resetvec", 0x1004);

    object_initialize_child(obj, "u-cluster", &s->u_cluster, TYPE_CPU_CLUSTER);
    qdev_prop_set_uint32(DEVICE(&s->u_cluster), "cluster-id", 1);

    object_initialize_child(OBJECT(&s->u_cluster), "u-cpus", &s->u_cpus,
                            TYPE_RISCV_HART_ARRAY);

    object_initialize_child(obj, "prci", &s->prci, TYPE_SIFIVE_U_PRCI);
    object_initialize_child(obj, "otp", &s->otp, TYPE_SIFIVE_U_OTP);
    object_initialize_child(obj, "gem", &s->gem, TYPE_CADENCE_GEM);
    object_initialize_child(obj, "gpio", &s->gpio, TYPE_SIFIVE_GPIO);
    object_initialize_child(obj, "pdma", &s->dma, TYPE_SIFIVE_PDMA);
    object_initialize_child(obj, "spi0", &s->spi0, TYPE_SIFIVE_SPI);
    object_initialize_child(obj, "spi2", &s->spi2, TYPE_SIFIVE_SPI);
    object_initialize_child(obj, "pwm0", &s->pwm[0], TYPE_SIFIVE_PWM);
    object_initialize_child(obj, "pwm1", &s->pwm[1], TYPE_SIFIVE_PWM);
}

static void sifive_u_soc_realize(DeviceState *dev, Error **errp)
{
    MachineState *ms = MACHINE(qdev_get_machine());
    SiFiveUSoCState *s = RISCV_U_SOC(dev);
    const MemMapEntry *memmap = sifive_u_memmap;
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *mask_rom = g_new(MemoryRegion, 1);
    MemoryRegion *l2lim_mem = g_new(MemoryRegion, 1);
    char *plic_hart_config;
    int i, j;

    qdev_prop_set_uint32(DEVICE(&s->u_cpus), "num-harts", ms->smp.cpus - 1);
    qdev_prop_set_uint32(DEVICE(&s->u_cpus), "hartid-base", 1);
    qdev_prop_set_string(DEVICE(&s->u_cpus), "cpu-type", s->cpu_type);
    qdev_prop_set_uint64(DEVICE(&s->u_cpus), "resetvec", 0x1004);

    sysbus_realize(SYS_BUS_DEVICE(&s->e_cpus), &error_fatal);
    sysbus_realize(SYS_BUS_DEVICE(&s->u_cpus), &error_fatal);
    /*
     * The cluster must be realized after the RISC-V hart array container,
     * as the container's CPU object is only created on realize, and the
     * CPU must exist and have been parented into the cluster before the
     * cluster is realized.
     */
    qdev_realize(DEVICE(&s->e_cluster), NULL, &error_abort);
    qdev_realize(DEVICE(&s->u_cluster), NULL, &error_abort);

    /* boot rom */
    memory_region_init_rom(mask_rom, OBJECT(dev), "riscv.sifive.u.mrom",
                           memmap[SIFIVE_U_DEV_MROM].size, &error_fatal);
    memory_region_add_subregion(system_memory, memmap[SIFIVE_U_DEV_MROM].base,
                                mask_rom);

    /*
     * Add L2-LIM at reset size.
     * This should be reduced in size as the L2 Cache Controller WayEnable
     * register is incremented. Unfortunately I don't see a nice (or any) way
     * to handle reducing or blocking out the L2 LIM while still allowing it
     * be re returned to all enabled after a reset. For the time being, just
     * leave it enabled all the time. This won't break anything, but will be
     * too generous to misbehaving guests.
     */
    memory_region_init_ram(l2lim_mem, NULL, "riscv.sifive.u.l2lim",
                           memmap[SIFIVE_U_DEV_L2LIM].size, &error_fatal);
    memory_region_add_subregion(system_memory, memmap[SIFIVE_U_DEV_L2LIM].base,
                                l2lim_mem);

    /* create PLIC hart topology configuration string */
    plic_hart_config = riscv_plic_hart_config_string(ms->smp.cpus);

    /* MMIO */
    s->plic = sifive_plic_create(memmap[SIFIVE_U_DEV_PLIC].base,
        plic_hart_config, ms->smp.cpus, 0,
        SIFIVE_U_PLIC_NUM_SOURCES,
        SIFIVE_U_PLIC_NUM_PRIORITIES,
        SIFIVE_U_PLIC_PRIORITY_BASE,
        SIFIVE_U_PLIC_PENDING_BASE,
        SIFIVE_U_PLIC_ENABLE_BASE,
        SIFIVE_U_PLIC_ENABLE_STRIDE,
        SIFIVE_U_PLIC_CONTEXT_BASE,
        SIFIVE_U_PLIC_CONTEXT_STRIDE,
        memmap[SIFIVE_U_DEV_PLIC].size);
    g_free(plic_hart_config);
    sifive_uart_create(system_memory, memmap[SIFIVE_U_DEV_UART0].base,
        serial_hd(0), qdev_get_gpio_in(DEVICE(s->plic), SIFIVE_U_UART0_IRQ));
    sifive_uart_create(system_memory, memmap[SIFIVE_U_DEV_UART1].base,
        serial_hd(1), qdev_get_gpio_in(DEVICE(s->plic), SIFIVE_U_UART1_IRQ));
    riscv_aclint_swi_create(memmap[SIFIVE_U_DEV_CLINT].base, 0,
        ms->smp.cpus, false);
    riscv_aclint_mtimer_create(memmap[SIFIVE_U_DEV_CLINT].base +
            RISCV_ACLINT_SWI_SIZE,
        RISCV_ACLINT_DEFAULT_MTIMER_SIZE, 0, ms->smp.cpus,
        RISCV_ACLINT_DEFAULT_MTIMECMP, RISCV_ACLINT_DEFAULT_MTIME,
        CLINT_TIMEBASE_FREQ, false);

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->prci), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->prci), 0, memmap[SIFIVE_U_DEV_PRCI].base);

    qdev_prop_set_uint32(DEVICE(&s->gpio), "ngpio", 16);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpio), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio), 0, memmap[SIFIVE_U_DEV_GPIO].base);

    /* Pass all GPIOs to the SOC layer so they are available to the board */
    qdev_pass_gpios(DEVICE(&s->gpio), dev, NULL);

    /* Connect GPIO interrupts to the PLIC */
    for (i = 0; i < 16; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio), i,
                           qdev_get_gpio_in(DEVICE(s->plic),
                                            SIFIVE_U_GPIO_IRQ0 + i));
    }

    /* PDMA */
    sysbus_realize(SYS_BUS_DEVICE(&s->dma), errp);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->dma), 0, memmap[SIFIVE_U_DEV_PDMA].base);

    /* Connect PDMA interrupts to the PLIC */
    for (i = 0; i < SIFIVE_PDMA_IRQS; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->dma), i,
                           qdev_get_gpio_in(DEVICE(s->plic),
                                            SIFIVE_U_PDMA_IRQ0 + i));
    }

    qdev_prop_set_uint32(DEVICE(&s->otp), "serial", s->serial);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->otp), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->otp), 0, memmap[SIFIVE_U_DEV_OTP].base);

    qemu_configure_nic_device(DEVICE(&s->gem), true, NULL);
    object_property_set_int(OBJECT(&s->gem), "revision", GEM_REVISION,
                            &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->gem), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gem), 0, memmap[SIFIVE_U_DEV_GEM].base);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gem), 0,
                       qdev_get_gpio_in(DEVICE(s->plic), SIFIVE_U_GEM_IRQ));

    /* PWM */
    for (i = 0; i < 2; i++) {
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->pwm[i]), errp)) {
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->pwm[i]), 0,
                                memmap[SIFIVE_U_DEV_PWM0].base + (0x1000 * i));

        /* Connect PWM interrupts to the PLIC */
        for (j = 0; j < SIFIVE_PWM_IRQS; j++) {
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->pwm[i]), j,
                               qdev_get_gpio_in(DEVICE(s->plic),
                                        SIFIVE_U_PWM0_IRQ0 + (i * 4) + j));
        }
    }

    create_unimplemented_device("riscv.sifive.u.gem-mgmt",
        memmap[SIFIVE_U_DEV_GEM_MGMT].base, memmap[SIFIVE_U_DEV_GEM_MGMT].size);

    create_unimplemented_device("riscv.sifive.u.dmc",
        memmap[SIFIVE_U_DEV_DMC].base, memmap[SIFIVE_U_DEV_DMC].size);

    create_unimplemented_device("riscv.sifive.u.l2cc",
        memmap[SIFIVE_U_DEV_L2CC].base, memmap[SIFIVE_U_DEV_L2CC].size);

    sysbus_realize(SYS_BUS_DEVICE(&s->spi0), errp);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->spi0), 0,
                    memmap[SIFIVE_U_DEV_QSPI0].base);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->spi0), 0,
                       qdev_get_gpio_in(DEVICE(s->plic), SIFIVE_U_QSPI0_IRQ));
    sysbus_realize(SYS_BUS_DEVICE(&s->spi2), errp);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->spi2), 0,
                    memmap[SIFIVE_U_DEV_QSPI2].base);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->spi2), 0,
                       qdev_get_gpio_in(DEVICE(s->plic), SIFIVE_U_QSPI2_IRQ));
}

static const Property sifive_u_soc_props[] = {
    DEFINE_PROP_UINT32("serial", SiFiveUSoCState, serial, OTP_SERIAL),
    DEFINE_PROP_STRING("cpu-type", SiFiveUSoCState, cpu_type),
};

static void sifive_u_soc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_props(dc, sifive_u_soc_props);
    dc->realize = sifive_u_soc_realize;
    /* Reason: Uses serial_hds in realize function, thus can't be used twice */
    dc->user_creatable = false;
}

static const TypeInfo sifive_u_soc_type_info = {
    .name = TYPE_RISCV_U_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(SiFiveUSoCState),
    .instance_init = sifive_u_soc_instance_init,
    .class_init = sifive_u_soc_class_init,
};

static void sifive_u_soc_register_types(void)
{
    type_register_static(&sifive_u_soc_type_info);
}

type_init(sifive_u_soc_register_types)
