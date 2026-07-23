/*
 * ACPI tables for the SiFive U synthetic PCIe/CXL extension
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/acpi/acpi-defs.h"
#include "hw/acpi/acpi.h"
#include "hw/acpi/aml-build.h"
#include "hw/acpi/cxl.h"
#include "hw/acpi/pci.h"
#include "hw/acpi/utils.h"
#include "hw/nvram/fw_cfg_acpi.h"
#include "hw/pci/pci_bus.h"
#include "hw/riscv/sifive_u.h"
#include "hw/riscv/sifive_u-acpi.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "system/reset.h"
#include "target/riscv/cpu.h"

#define SIFIVE_U_ACPI_TABLE_SIZE 0x20000
#define SIFIVE_U_RHCT_NODE_ARRAY_OFFSET 56

static char sifive_u_acpi_oem_id[] = ACPI_BUILD_APPNAME6;
static char sifive_u_acpi_oem_table_id[] = ACPI_BUILD_APPNAME8;

typedef struct SiFiveUAcpiBuildState {
    MemoryRegion *table_mr;
    MemoryRegion *rsdp_mr;
    MemoryRegion *linker_mr;
    bool patched;
} SiFiveUAcpiBuildState;

static void sifive_u_acpi_align_size(GArray *blob, unsigned int align)
{
    g_array_set_size(blob, ROUND_UP(acpi_data_len(blob), align));
}

static void sifive_u_madt_add_rintc(uint32_t uid, uint64_t hart_id,
                                    GArray *entry)
{
    build_append_int_noprefix(entry, 0x18, 1);
    build_append_int_noprefix(entry, 36, 1);
    build_append_int_noprefix(entry, 1, 1);
    build_append_int_noprefix(entry, 0, 1);
    build_append_int_noprefix(entry, 1, 4);
    build_append_int_noprefix(entry, hart_id, 8);
    build_append_int_noprefix(entry, uid, 4);
    build_append_int_noprefix(entry, 2 * uid + 1, 4);
    build_append_int_noprefix(entry, 0, 8);
    build_append_int_noprefix(entry, 0, 4);
}

static void sifive_u_dsdt_add_cpus(Aml *scope, SiFiveUState *s)
{
    MachineClass *mc = MACHINE_GET_CLASS(s);
    MachineState *ms = MACHINE(s);
    const CPUArchIdList *arch_ids = mc->possible_cpu_arch_ids(ms);
    int i;

    for (i = 0; i < arch_ids->len; i++) {
        g_autoptr(GArray) madt = g_array_new(false, true, 1);
        Aml *dev = aml_device("C%.03X", i);

        aml_append(dev, aml_name_decl("_HID", aml_string("ACPI0007")));
        aml_append(dev, aml_name_decl(
                       "_UID", aml_int(arch_ids->cpus[i].arch_id)));
        sifive_u_madt_add_rintc(i, arch_ids->cpus[i].arch_id, madt);
        aml_append(dev, aml_name_decl(
                       "_MAT", aml_buffer(madt->len,
                                          (uint8_t *)madt->data)));
        aml_append(scope, dev);
    }
}

static void sifive_u_dsdt_add_plic(Aml *scope)
{
    Aml *crs;
    Aml *dev = aml_device("IC00");

    aml_append(dev, aml_name_decl("_HID", aml_string("RSCV0001")));
    aml_append(dev, aml_name_decl("_UID", aml_int(0)));
    aml_append(dev, aml_name_decl("_GSB", aml_int(0)));
    crs = aml_resource_template();
    aml_append(crs, aml_memory32_fixed(
                   sifive_u_memmap[SIFIVE_U_DEV_PLIC].base,
                   sifive_u_memmap[SIFIVE_U_DEV_PLIC].size,
                   AML_READ_WRITE));
    aml_append(dev, aml_name_decl("_CRS", crs));
    aml_append(scope, dev);
}

static void sifive_u_build_cxl_root(Aml *scope, SiFiveUState *s)
{
    Aml *cxl_dev = aml_device("CXLM");
    PCIBus *bus;
    uint32_t nr = 0;

    aml_append(cxl_dev,
               aml_name_decl("_HID", aml_string("ACPI0017")));
    QLIST_FOREACH(bus, &s->pci_bus->child, sibling) {
        if (pci_bus_is_root(bus) && pci_bus_is_cxl(bus)) {
            nr++;
        }
    }

    if (nr) {
        Aml *dep_pkg = aml_package(nr);

        QLIST_FOREACH(bus, &s->pci_bus->child, sibling) {
            if (pci_bus_is_root(bus) && pci_bus_is_cxl(bus)) {
                aml_append(dep_pkg,
                           aml_name("\\_SB.PC%.02X", pci_bus_num(bus)));
            }
        }
        aml_append(cxl_dev, aml_name_decl("_DEP", dep_pkg));
    }

    {
        Aml *method = aml_method("_STA", 0, AML_NOTSERIALIZED);

        aml_append(method, aml_return(aml_int(0x0b)));
        aml_append(cxl_dev, method);
    }

    build_cxl_dsm_method(cxl_dev);
    aml_append(scope, cxl_dev);
}

static void sifive_u_build_dsdt(GArray *table_data, BIOSLinker *linker,
                                SiFiveUState *s)
{
    AcpiTable table = {
        .sig = "DSDT",
        .rev = 2,
        .oem_id = sifive_u_acpi_oem_id,
        .oem_table_id = sifive_u_acpi_oem_table_id,
    };
    Aml *dsdt;
    Aml *scope;

    acpi_table_begin(&table, table_data);
    dsdt = init_aml_allocator();
    scope = aml_scope("\\_SB");

    sifive_u_dsdt_add_cpus(scope, s);
    fw_cfg_acpi_dsdt_add(
        scope, &sifive_u_memmap[SIFIVE_U_DEV_FW_CFG]);
    sifive_u_dsdt_add_plic(scope);
    acpi_dsdt_add_gpex(scope, &s->gpex_host->gpex_cfg);
    sifive_u_build_cxl_root(scope, s);

    aml_append(dsdt, scope);
    g_array_append_vals(table_data, dsdt->buf->data, dsdt->buf->len);
    acpi_table_end(linker, &table);
    free_aml_allocator();
}

static void sifive_u_build_fadt(GArray *table_data, BIOSLinker *linker,
                                unsigned int dsdt_offset)
{
    AcpiFadtData fadt = {
        .rev = 6,
        .minor_ver = 6,
        .flags = 1 << ACPI_FADT_F_HW_REDUCED_ACPI,
        .xdsdt_tbl_offset = &dsdt_offset,
    };

    build_fadt(table_data, linker, &fadt, sifive_u_acpi_oem_id,
               sifive_u_acpi_oem_table_id);
}

static void sifive_u_build_madt(GArray *table_data, BIOSLinker *linker,
                                SiFiveUState *s)
{
    MachineClass *mc = MACHINE_GET_CLASS(s);
    MachineState *ms = MACHINE(s);
    const CPUArchIdList *arch_ids = mc->possible_cpu_arch_ids(ms);
    AcpiTable table = {
        .sig = "APIC",
        .rev = 7,
        .oem_id = sifive_u_acpi_oem_id,
        .oem_table_id = sifive_u_acpi_oem_table_id,
    };
    int i;

    acpi_table_begin(&table, table_data);
    build_append_int_noprefix(table_data, 0, 4);
    build_append_int_noprefix(table_data, 0, 4);

    for (i = 0; i < arch_ids->len; i++) {
        sifive_u_madt_add_rintc(i, arch_ids->cpus[i].arch_id,
                                table_data);
    }

    build_append_int_noprefix(table_data, 0x1b, 1);
    build_append_int_noprefix(table_data, 36, 1);
    build_append_int_noprefix(table_data, 1, 1);
    build_append_int_noprefix(table_data, 0, 1);
    build_append_int_noprefix(table_data, 0, 8);
    build_append_int_noprefix(table_data,
                              SIFIVE_U_PLIC_NUM_SOURCES - 1, 2);
    build_append_int_noprefix(table_data,
                              SIFIVE_U_PLIC_NUM_PRIORITIES, 2);
    build_append_int_noprefix(table_data, 0, 4);
    build_append_int_noprefix(
        table_data, sifive_u_memmap[SIFIVE_U_DEV_PLIC].size, 4);
    build_append_int_noprefix(
        table_data, sifive_u_memmap[SIFIVE_U_DEV_PLIC].base, 8);
    build_append_int_noprefix(table_data, 0, 4);

    acpi_table_end(linker, &table);
}

static void sifive_u_build_rhct(GArray *table_data, BIOSLinker *linker,
                                SiFiveUState *s)
{
    MachineState *ms = MACHINE(s);
    RISCVCPU *cpu = &s->soc.u_cpus.harts[0];
    AcpiTable table = {
        .sig = "RHCT",
        .rev = 1,
        .oem_id = sifive_u_acpi_oem_id,
        .oem_table_id = sifive_u_acpi_oem_table_id,
    };
    g_autofree char *isa = riscv_isa_string(cpu);
    uint32_t isa_offset;
    size_t len = 8 + strlen(isa) + 1;
    size_t aligned_len = (len % 2) ? len + 1 : len;
    int i;

    acpi_table_begin(&table, table_data);
    build_append_int_noprefix(table_data, 0, 4);
    build_append_int_noprefix(table_data, 1000000, 8);
    build_append_int_noprefix(table_data, 1 + ms->smp.cpus, 4);
    build_append_int_noprefix(
        table_data, SIFIVE_U_RHCT_NODE_ARRAY_OFFSET, 4);

    isa_offset = table_data->len - table.table_offset;
    build_append_int_noprefix(table_data, 0, 2);
    build_append_int_noprefix(table_data, aligned_len, 2);
    build_append_int_noprefix(table_data, 1, 2);
    build_append_int_noprefix(table_data, strlen(isa) + 1, 2);
    g_array_append_vals(table_data, isa, strlen(isa) + 1);
    if (aligned_len != len) {
        build_append_int_noprefix(table_data, 0, 1);
    }

    for (i = 0; i < ms->smp.cpus; i++) {
        build_append_int_noprefix(table_data, 0xffff, 2);
        build_append_int_noprefix(table_data, 16, 2);
        build_append_int_noprefix(table_data, 1, 2);
        build_append_int_noprefix(table_data, 1, 2);
        build_append_int_noprefix(table_data, i, 4);
        build_append_int_noprefix(table_data, isa_offset, 4);
    }

    acpi_table_end(linker, &table);
}

static void sifive_u_acpi_build(SiFiveUState *s, AcpiBuildTables *tables)
{
    GArray *table_offsets = g_array_new(false, true, sizeof(uint32_t));
    GArray *tables_blob = tables->table_data;
    unsigned int dsdt;
    unsigned int xsdt;
    AcpiMcfgInfo mcfg = {
        .base = sifive_u_memmap[SIFIVE_U_DEV_PCIE_ECAM].base,
        .size = sifive_u_memmap[SIFIVE_U_DEV_PCIE_ECAM].size,
    };

    bios_linker_loader_alloc(tables->linker, ACPI_BUILD_TABLE_FILE,
                             tables_blob, 64, false);

    dsdt = tables_blob->len;
    sifive_u_build_dsdt(tables_blob, tables->linker, s);

    acpi_add_table(table_offsets, tables_blob);
    sifive_u_build_fadt(tables_blob, tables->linker, dsdt);

    acpi_add_table(table_offsets, tables_blob);
    sifive_u_build_madt(tables_blob, tables->linker, s);

    acpi_add_table(table_offsets, tables_blob);
    sifive_u_build_rhct(tables_blob, tables->linker, s);

    acpi_add_table(table_offsets, tables_blob);
    build_mcfg(tables_blob, tables->linker, &mcfg,
               sifive_u_acpi_oem_id, sifive_u_acpi_oem_table_id);

    cxl_build_cedt(table_offsets, tables_blob, tables->linker,
                   sifive_u_acpi_oem_id, sifive_u_acpi_oem_table_id,
                   &s->cxl_devices_state);

    xsdt = tables_blob->len;
    build_xsdt(tables_blob, tables->linker, table_offsets,
               sifive_u_acpi_oem_id, sifive_u_acpi_oem_table_id);

    {
        AcpiRsdpData rsdp_data = {
            .revision = 2,
            .oem_id = sifive_u_acpi_oem_id,
            .xsdt_tbl_offset = &xsdt,
            .rsdt_tbl_offset = NULL,
        };

        build_rsdp(tables->rsdp, tables->linker, &rsdp_data);
    }

    if (tables_blob->len > SIFIVE_U_ACPI_TABLE_SIZE / 2) {
        warn_report("SiFive U ACPI table size %u exceeds %d bytes",
                    tables_blob->len, SIFIVE_U_ACPI_TABLE_SIZE / 2);
    }
    sifive_u_acpi_align_size(tables_blob, SIFIVE_U_ACPI_TABLE_SIZE);
    g_array_free(table_offsets, true);
}

static void sifive_u_acpi_ram_update(MemoryRegion *mr, GArray *data)
{
    uint32_t size = acpi_data_len(data);

    memory_region_ram_resize(mr, size, &error_abort);
    memcpy(memory_region_get_ram_ptr(mr), data->data, size);
    memory_region_set_dirty(mr, 0, size);
}

static void sifive_u_acpi_build_update(void *opaque)
{
    SiFiveUAcpiBuildState *build_state = opaque;
    AcpiBuildTables tables;

    if (!build_state || build_state->patched) {
        return;
    }

    build_state->patched = true;
    acpi_build_tables_init(&tables);
    sifive_u_acpi_build(RISCV_U_MACHINE(qdev_get_machine()), &tables);
    sifive_u_acpi_ram_update(build_state->table_mr, tables.table_data);
    sifive_u_acpi_ram_update(build_state->rsdp_mr, tables.rsdp);
    sifive_u_acpi_ram_update(build_state->linker_mr,
                             tables.linker->cmd_blob);
    acpi_build_tables_cleanup(&tables, true);
}

static void sifive_u_acpi_build_reset(void *opaque)
{
    SiFiveUAcpiBuildState *build_state = opaque;

    build_state->patched = false;
}

static const VMStateDescription vmstate_sifive_u_acpi_build = {
    .name = "sifive_u_acpi_build",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(patched, SiFiveUAcpiBuildState),
        VMSTATE_END_OF_LIST()
    },
};

void sifive_u_acpi_setup(SiFiveUState *s)
{
    SiFiveUAcpiBuildState *build_state;
    AcpiBuildTables tables;

    build_state = g_new0(SiFiveUAcpiBuildState, 1);
    acpi_build_tables_init(&tables);
    sifive_u_acpi_build(s, &tables);

    build_state->table_mr = acpi_add_rom_blob(
        sifive_u_acpi_build_update, build_state, tables.table_data,
        ACPI_BUILD_TABLE_FILE);
    assert(build_state->table_mr);

    build_state->linker_mr = acpi_add_rom_blob(
        sifive_u_acpi_build_update, build_state, tables.linker->cmd_blob,
        ACPI_BUILD_LOADER_FILE);
    build_state->rsdp_mr = acpi_add_rom_blob(
        sifive_u_acpi_build_update, build_state, tables.rsdp,
        ACPI_BUILD_RSDP_FILE);

    qemu_register_reset(sifive_u_acpi_build_reset, build_state);
    sifive_u_acpi_build_reset(build_state);
    vmstate_register(NULL, 0, &vmstate_sifive_u_acpi_build, build_state);
    acpi_build_tables_cleanup(&tables, false);
}
