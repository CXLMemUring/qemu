# SiFive U Synthetic PCIe/CXL QEMU Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an opt-in synthetic GPEX/PCIe/CXL platform, Type 2 and Type 3 support, `fw_cfg`, and ACPI/CEDT generation to the exact QEMU `sifive_u` machine.

**Architecture:** `-M sifive_u,cxl=on` creates a GPEX root bus and machine-owned `CXLState`, while the default `cxl=off` path remains unchanged. QEMU describes PCI and `fw_cfg` to U-Boot in DT, publishes ACPI0016/0017 and CEDT through `fw_cfg`, and leaves endpoints explicit on the command line.

**Tech Stack:** QEMU QOM/qdev, GPEX, PCIe/PXB/CXL, libfdt, AML/ACPI table builders, QTest and QEMU functional Python tests.

---

## File map

- Modify `qemu-cxl-type2/hw/riscv/Kconfig`: select GPEX, PXB, ACPI, and CXL dependencies for `SIFIVE_U`.
- Modify `qemu-cxl-type2/include/hw/riscv/sifive_u.h`: own synthetic PCIe, `fw_cfg`, ACPI, and CXL state and define resource/IRQ constants.
- Modify `qemu-cxl-type2/hw/riscv/sifive_u.c`: add the `cxl` machine property, address map, GPEX creation, DT nodes, `fw_cfg`, CXL windows, and machine-done hooks.
- Create `qemu-cxl-type2/include/hw/riscv/sifive_u-acpi.h`: declare the SiFive U ACPI setup interface.
- Create `qemu-cxl-type2/hw/riscv/sifive_u-acpi-build.c`: build the RISC-V tables, ACPI0016/0017 objects, `_DEP`, and CEDT.
- Modify `qemu-cxl-type2/hw/riscv/meson.build`: compile the SiFive U ACPI builder.
- Modify `qemu-cxl-type2/include/hw/pci-host/gpex.h`: add the dedicated CXL MMIO32 aperture to `GPEXConfig`.
- Modify `qemu-cxl-type2/hw/pci-host/gpex-acpi.c`: emit static CXL-host `_CRS`.
- Modify `qemu-cxl-type2/hw/cxl/cxl-host-stubs.c`: provide disabled-CXL QMP stubs for the fork's Zettai commands.
- Modify `qemu-cxl-type2/tests/functional/test_riscv64_sifive_u.py`: test default compatibility, CXL DT, rejection, and dual endpoint startup.
- Modify `qemu-cxl-type2/tests/functional/meson.build`: register the new test timeout/category.
- Create `qemu-cxl-type2/tests/qtest/sifive-u-cxl-test.c`: read SiFive U
  `fw_cfg` directly and validate ACPI/CEDT files without another firmware.
- Modify `qemu-cxl-type2/tests/qtest/meson.build`: register the SiFive U CXL
  QTest.
- Modify `qemu-cxl-type2/docs/system/riscv/sifive_u.rst`: document that the host bridge is synthetic and opt-in.

### Task 1: Repair the non-CXL link boundary

**Files:**
- Modify: `qemu-cxl-type2/hw/cxl/cxl-host-stubs.c`

- [ ] **Step 1: Reproduce the unresolved QMP symbols**

Run:

```bash
cd /root/cxl-u-boot/qemu-cxl-type2
ninja -C build-riscv-cxl qemu-system-riscv64
```

Expected: link failure naming `qmp_zettai_bind_vppb` and
`qmp_zettai_unbind_vppb`.

- [ ] **Step 2: Add explicit disabled-CXL QMP stubs**

Add the generated QAPI header and these functions:

```c
#include "qapi/qapi-commands-cxl.h"

void qmp_zettai_bind_vppb(const char *path, uint8_t vcs_id, uint8_t vppb_id,
                          uint8_t dsp_ppb_id, bool has_ld_id, uint16_t ld_id,
                          Error **errp)
{
    error_setg(errp, "CXL support is not available in this QEMU binary");
}

void qmp_zettai_unbind_vppb(const char *path, uint8_t vcs_id, uint8_t vppb_id,
                            bool has_option, uint16_t option, Error **errp)
{
    error_setg(errp, "CXL support is not available in this QEMU binary");
}
```

- [ ] **Step 3: Prove the baseline binary links**

Run:

```bash
ninja -C /root/cxl-u-boot/qemu-cxl-type2/build-riscv-cxl qemu-system-riscv64
```

Expected: target links with no undefined `qmp_zettai_*` symbols.

- [ ] **Step 4: Commit the isolated repair**

```bash
cd /root/cxl-u-boot/qemu-cxl-type2
git add hw/cxl/cxl-host-stubs.c
git commit -m "hw/cxl: stub Zettai QMP commands without CXL"
```

### Task 2: Define the SiFive U synthetic resource contract

**Files:**
- Modify: `qemu-cxl-type2/hw/riscv/Kconfig`
- Modify: `qemu-cxl-type2/include/hw/riscv/sifive_u.h`
- Modify: `qemu-cxl-type2/hw/riscv/sifive_u.c`
- Modify: `qemu-cxl-type2/tests/functional/test_riscv64_sifive_u.py`

- [ ] **Step 1: Add a failing machine-property test**

Add this test method:

```python
def test_sifive_u_cxl_property(self):
    self.set_machine('sifive_u')
    self.vm.add_args('-machine', 'cxl=on', '-display', 'none', '-S')
    self.vm.launch()
```

Run:

```bash
cd /root/cxl-u-boot/qemu-cxl-type2
build-riscv-cxl/pyvenv/bin/python3 tests/functional/test_riscv64_sifive_u.py \
  SifiveU.test_sifive_u_cxl_property
```

Expected: FAIL with `Property 'sifive_u-machine.cxl' not found`.

- [ ] **Step 2: Select the machine dependencies**

Extend `config SIFIVE_U` with:

```text
    select PCI
    select PCI_EXPRESS_GENERIC_BRIDGE
    select PXB
    select FW_CFG_DMA
    select ACPI
    select ACPI_PCI
    select ACPI_CXL
```

- [ ] **Step 3: Add state and constants**

Add the required includes and fields:

```c
#include "hw/cxl/cxl.h"
#include "hw/nvram/fw_cfg.h"
#include "hw/pci/pci.h"
#include "hw/pci-host/gpex.h"

typedef struct SiFiveUState {
    MachineState parent_obj;
    SiFiveUSoCState soc;
    int fdt_size;
    bool start_in_flash;
    uint32_t msel;
    uint32_t serial;

    GPEXHost *gpex_host;
    PCIBus *pci_bus;
    FWCfgState *fw_cfg;
    CXLState cxl_devices_state;
    Notifier machine_done;
} SiFiveUState;
```

Extend the memory-map enum and table with:

```c
SIFIVE_U_DEV_PCIE_ECAM,
SIFIVE_U_DEV_PCIE_PIO,
SIFIVE_U_DEV_PCIE_MMIO,
SIFIVE_U_DEV_FW_CFG,
SIFIVE_U_DEV_PCIE_MMIO_HIGH,
SIFIVE_U_DEV_CXL_HOST_REG,
SIFIVE_U_DEV_CXL_FMW,
```

```c
[SIFIVE_U_DEV_PCIE_ECAM]      = { 0x30000000, 256 * MiB },
[SIFIVE_U_DEV_PCIE_PIO]       = { 0x10080000,  64 * KiB },
[SIFIVE_U_DEV_PCIE_MMIO]      = { 0x40000000,   1 * GiB },
[SIFIVE_U_DEV_FW_CFG]         = { 0x10100000,  0x18 },
[SIFIVE_U_DEV_PCIE_MMIO_HIGH] = { 0x400000000ULL, 16 * GiB },
[SIFIVE_U_DEV_CXL_HOST_REG]   = { 0x800000000ULL,  1 * MiB },
[SIFIVE_U_DEV_CXL_FMW]        = { 0x1000000000ULL, 4 * GiB },
```

Define the reservation and four unused PLIC sources:

```c
#define SIFIVE_U_CXL_MMIO32_SIZE (256 * MiB)
#define SIFIVE_U_PCIE_IRQ_BASE 32
#define SIFIVE_U_PCIE_IRQ_COUNT PCI_NUM_PINS
```

- [ ] **Step 4: Reject a RAM/resource overlap**

Add and call before mapping any synthetic region:

```c
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
```

Add a functional test with `-M sifive_u,cxl=on -m 16G`; require nonzero exit
and the exact overlap message.

- [ ] **Step 5: Add the opt-in property**

Initialize CXL and add the property in instance initialization:

```c
cxl_machine_init(obj, &s->cxl_devices_state);
```

`cxl_machine_init()` creates the public `cxl` and `cxl-fmw` properties and
defaults `CXLState.is_enabled` to false. Do not register a second `cxl`
property. The observable property is `sifive_u-machine.cxl`.

- [ ] **Step 6: Build and run the property test**

Run:

```bash
cd /root/cxl-u-boot/qemu-cxl-type2
ninja -C build-riscv-cxl qemu-system-riscv64
build-riscv-cxl/pyvenv/bin/python3 tests/functional/test_riscv64_sifive_u.py \
  SifiveU.test_sifive_u_cxl_property
```

Expected: PASS and QEMU remains running until the test tears it down.

- [ ] **Step 7: Commit**

```bash
git add hw/riscv/Kconfig include/hw/riscv/sifive_u.h hw/riscv/sifive_u.c \
        tests/functional/test_riscv64_sifive_u.py
git commit -m "hw/riscv/sifive_u: define synthetic CXL resources"
```

### Task 3: Create GPEX, `fw_cfg`, and the U-Boot device tree

**Files:**
- Modify: `qemu-cxl-type2/hw/riscv/sifive_u.c`
- Modify: `qemu-cxl-type2/tests/functional/test_riscv64_sifive_u.py`

- [ ] **Step 1: Add failing DT assertions**

Add a helper that launches QEMU with `-machine dumpdtb=<tempfile>,cxl=on`,
runs `dtc -I dtb -O dts`, and asserts:

```python
self.assertIn('compatible = "pci-host-ecam-generic"', dts)
self.assertIn('reg = <0x00 0x30000000 0x00 0x10000000>', dts)
self.assertIn('bus-range = <0x00 0xff>', dts)
self.assertIn('qemu,fw-cfg-mmio', dts)
self.assertIn('qemu,synthetic-cxl-host', dts)
```

Also dump the default `sifive_u` DT and assert none of those strings appear.

Run the two methods and expect the CXL method to fail.

- [ ] **Step 2: Implement GPEX creation**

Add `sifive_u_create_gpex()` using the same aliases as `gpex_pcie_init()` in
`hw/riscv/virt.c`. Set:

```c
object_property_set_uint(OBJECT(dev), PCI_HOST_ECAM_BASE, 0x30000000, NULL);
object_property_set_int(OBJECT(dev), PCI_HOST_ECAM_SIZE, 256 * MiB, NULL);
object_property_set_uint(OBJECT(dev), PCI_HOST_PIO_BASE, 0x10080000, NULL);
object_property_set_int(OBJECT(dev), PCI_HOST_PIO_SIZE, 64 * KiB, NULL);
object_property_set_uint(OBJECT(dev), PCI_HOST_BELOW_4G_MMIO_BASE,
                         0x40000000, NULL);
object_property_set_int(OBJECT(dev), PCI_HOST_BELOW_4G_MMIO_SIZE,
                        768 * MiB, NULL);
object_property_set_uint(OBJECT(dev), PCI_HOST_ABOVE_4G_MMIO_BASE,
                         0x400000000ULL, NULL);
object_property_set_int(OBJECT(dev), PCI_HOST_ABOVE_4G_MMIO_SIZE,
                        16 * GiB, NULL);
```

Connect GPEX INTx pins to PLIC sources 32 through 35:

```c
for (i = 0; i < PCI_NUM_PINS; i++) {
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), i,
        qdev_get_gpio_in(DEVICE(s->soc.plic), SIFIVE_U_PCIE_IRQ_BASE + i));
    gpex_set_irq_num(GPEX_HOST(dev), i, SIFIVE_U_PCIE_IRQ_BASE + i);
}
s->gpex_host = GPEX_HOST(dev);
s->pci_bus = PCI_HOST_BRIDGE(dev)->bus;
```

- [ ] **Step 3: Create memory-mapped `fw_cfg`**

Add:

```c
static void sifive_u_create_fw_cfg(SiFiveUState *s)
{
    MachineState *ms = MACHINE(s);
    hwaddr base = sifive_u_memmap[SIFIVE_U_DEV_FW_CFG].base;

    s->fw_cfg = fw_cfg_init_mem_wide(base + 8, base, 8, base + 16,
                                     &address_space_memory);
    fw_cfg_add_i16(s->fw_cfg, FW_CFG_NB_CPUS, ms->smp.cpus);
    rom_set_fw(s->fw_cfg);
}
```

Call it only after checking `s->cxl_devices_state.is_enabled`.

- [ ] **Step 4: Emit the PCI and `fw_cfg` nodes**

Factor the current `create_fdt()` so the PLIC phandle can be passed to:

```c
static void sifive_u_cxl_fdt(SiFiveUState *s, uint32_t plic_phandle);
```

Emit `#address-cells = <3>`, `#size-cells = <2>`, ECAM `reg`, `bus-range`,
three `ranges` entries, and the four-pin `interrupt-map`/mask. The normal
MMIO32 range ends at `0x6fffffff`; the CXL reservation is not advertised to
U-Boot's primary PCI allocator. Add an empty
`qemu,synthetic-cxl-host` property.

Emit `qemu,fw-cfg-mmio` with:

```c
qemu_fdt_setprop_sized_cells(fdt, node, "reg",
    2, base, 2, 8, 2, base + 8, 2, 2, 2, base + 16, 2, 8);
```

- [ ] **Step 5: Guard all creation on CXL enablement**

Use:

```c
if (s->cxl_devices_state.is_enabled) {
    sifive_u_create_fw_cfg(s);
    sifive_u_create_gpex(s);
    sifive_u_cxl_fdt(s, plic_phandle);
}
```

The default path must not map any new region.

- [ ] **Step 6: Run DT and default-regression tests**

Run:

```bash
cd /root/cxl-u-boot/qemu-cxl-type2
ninja -C build-riscv-cxl qemu-system-riscv64
build-riscv-cxl/pyvenv/bin/python3 tests/functional/test_riscv64_sifive_u.py
```

Expected: both default SiFive U boot tests and the CXL DT tests PASS.

- [ ] **Step 7: Commit**

```bash
git add hw/riscv/sifive_u.c tests/functional/test_riscv64_sifive_u.py
git commit -m "hw/riscv/sifive_u: add synthetic GPEX and fw_cfg"
```

### Task 4: Wire machine-owned CXL state and dual endpoint topology

**Files:**
- Modify: `qemu-cxl-type2/hw/riscv/sifive_u.c`
- Modify: `qemu-cxl-type2/tests/functional/test_riscv64_sifive_u.py`

- [ ] **Step 1: Add a failing topology-start test**

Launch with:

```text
-M sifive_u,cxl=on
-M cxl-fmw.0.targets.0=cxl.1,cxl-fmw.0.size=4G
-object memory-backend-ram,id=t3mem,size=256M,share=on
-object memory-backend-ram,id=t3lsa,size=2M,share=on
-device pxb-cxl,bus=pcie.0,bus_nr=64,id=cxl.1
-device cxl-rp,bus=cxl.1,port=0,id=rp-t2,chassis=0,slot=0
-device cxl-type2,bus=rp-t2,gpu-mode=0,mem-size=256M,cache-size=64M,id=t2
-device cxl-rp,bus=cxl.1,port=1,id=rp-t3,chassis=0,slot=1
-device cxl-type3,bus=rp-t3,volatile-memdev=t3mem,lsa=t3lsa,id=t3
```

Query PCI through QMP and assert IDs `cxl.1`, `rp-t2`, `t2`, `rp-t3`, and
`t3` are present. Expected before implementation: QEMU exits because CXL has
no usable SiFive U host integration.

- [ ] **Step 2: Add host-register and FMW mapping**

Implement:

```c
static void sifive_u_create_cxl_regions(SiFiveUState *s)
{
    CXLState *cxl = &s->cxl_devices_state;
    MemoryRegion *sysmem = get_system_memory();

    if (!cxl->is_enabled)
        return;

    memory_region_init(&cxl->host_mr, OBJECT(s), "sifive-u-cxl-host-reg",
                       64 * KiB * 16);
    memory_region_add_subregion(sysmem, 0x800000000ULL, &cxl->host_mr);
    cxl_fmws_set_memmap(0x1000000000ULL, UINT64_MAX);
    cxl_fmws_update_mmio();
}
```

- [ ] **Step 3: Add machine-done hooks**

Register a notifier during machine init and implement:

```c
static void sifive_u_machine_done(Notifier *notifier, void *data)
{
    SiFiveUState *s = container_of(notifier, SiFiveUState, machine_done);

    cxl_hook_up_pxb_registers(s->pci_bus, &s->cxl_devices_state, &error_fatal);
    if (s->cxl_devices_state.is_enabled)
        cxl_fmws_link_targets(&error_fatal);
}
```

- [ ] **Step 4: Add negative tests**

Assert:

```text
-M sifive_u -device pxb-cxl,bus=pcie.0
```

fails because `pcie.0` does not exist, and an overlapping FMW fails before
vCPU execution. Match nonempty error text and a nonzero exit status.

- [ ] **Step 5: Run topology tests**

Run the focused methods. Expected: dual endpoint startup PASS; both negative
cases exit nonzero.

- [ ] **Step 6: Commit**

```bash
git add hw/riscv/sifive_u.c tests/functional/test_riscv64_sifive_u.py
git commit -m "hw/riscv/sifive_u: wire CXL host and fixed windows"
```

### Task 5: Build SiFive U ACPI, `_DEP`, and CEDT

**Files:**
- Create: `qemu-cxl-type2/include/hw/riscv/sifive_u-acpi.h`
- Create: `qemu-cxl-type2/hw/riscv/sifive_u-acpi-build.c`
- Modify: `qemu-cxl-type2/hw/riscv/meson.build`
- Modify: `qemu-cxl-type2/include/hw/pci-host/gpex.h`
- Modify: `qemu-cxl-type2/hw/pci-host/gpex-acpi.c`
- Modify: `qemu-cxl-type2/hw/riscv/sifive_u.c`

- [ ] **Step 1: Add the public interface**

Create:

```c
#ifndef HW_RISCV_SIFIVE_U_ACPI_H
#define HW_RISCV_SIFIVE_U_ACPI_H

typedef struct SiFiveUState SiFiveUState;

void sifive_u_acpi_setup(SiFiveUState *s);

#endif
```

- [ ] **Step 2: Add the CXL MMIO32 field and static `_CRS`**

Add:

```c
MemMapEntry cxl_mmio32;
```

to `GPEXConfig`. In the CXL branch of `acpi_dsdt_add_gpex()`, emit:

```c
if (is_cxl && cfg->cxl_mmio32.size) {
    crs = aml_resource_template();
    aml_append(crs, aml_dword_memory(AML_POS_DECODE, AML_MIN_FIXED,
        AML_MAX_FIXED, AML_NON_CACHEABLE, AML_READ_WRITE, 0,
        cfg->cxl_mmio32.base,
        cfg->cxl_mmio32.base + cfg->cxl_mmio32.size - 1,
        0, cfg->cxl_mmio32.size));
    aml_append(crs, aml_word_bus_number(AML_MIN_FIXED, AML_MAX_FIXED,
        AML_POS_DECODE, 0, bus_num, bus_num + 15, 0, 16));
} else {
    crs = build_crs(PCI_HOST_BRIDGE(BUS(bus)->parent), &crs_range_set,
                    cfg->pio.base, 0, 0, 0);
}
```

Set `gpex_cfg.cxl_mmio32` to `0x70000000/256 MiB`.

- [ ] **Step 3: Implement the SiFive U ACPI builder**

Adapt the reusable table construction from `virt-acpi-build.c`, with
`SiFiveUState` as the owner. The CXL root AML must be:

```c
static void build_cxl_root(Aml *scope, SiFiveUState *s)
{
    Aml *cxl_dev = aml_device("CXLM");
    Aml *dep_pkg;
    PCIBus *bus;
    uint32_t nr = 0;

    aml_append(cxl_dev, aml_name_decl("_HID", aml_string("ACPI0017")));
    QLIST_FOREACH(bus, &s->pci_bus->child, sibling) {
        if (pci_bus_is_root(bus) && pci_bus_is_cxl(bus))
            nr++;
    }
    if (nr) {
        dep_pkg = aml_package(nr);
        QLIST_FOREACH(bus, &s->pci_bus->child, sibling) {
            if (pci_bus_is_root(bus) && pci_bus_is_cxl(bus))
                aml_append(dep_pkg,
                           aml_name("\\_SB.PC%.02X", pci_bus_num(bus)));
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
```

Use an AML `_STA` method if the local builder requires method form. Call:

```c
acpi_dsdt_add_gpex(scope, &s->gpex_host->gpex_cfg);
cxl_build_cedt(table_offsets, tables_blob, tables->linker,
               ACPI_BUILD_APPNAME6, ACPI_BUILD_APPNAME8,
               &s->cxl_devices_state);
```

Publish `etc/acpi/tables`, `etc/table-loader`, and `etc/acpi/rsdp` through
`s->fw_cfg`, following `virt_acpi_setup()`.

The SiFive U builder emits exactly DSDT, FADT revision 6, MADT RINTC/PLIC,
RHCT, MCFG, CEDT, XSDT, and RSDP. Its DSDT contains CPU `ACPI0007` devices,
one PLIC device covering `0x0c000000/64 MiB`, the primary GPEX host, all
ACPI0016 CXL host bridges, and ACPI0017. Parameterize the copied PLIC helper
with `SIFIVE_U_PLIC_NUM_SOURCES` instead of `VIRT_IRQCHIP_NUM_SOURCES`.

Do not emit the `virt` 16550 `RSCV0003` device or SPCR: the SiFive UART model
is not register-compatible with 16550. Linux console output for this machine
uses OpenSBI DBCN through `CONFIG_HVC_RISCV_SBI` and `console=hvc0`.

- [ ] **Step 4: Compile only when SiFive U and ACPI are enabled**

Add:

```meson
riscv_ss.add(when: ['CONFIG_SIFIVE_U', 'CONFIG_ACPI'],
             if_true: files('sifive_u-acpi-build.c'))
```

Call `sifive_u_acpi_setup(s)` after `fw_cfg` creation and before machine
finalization.

- [ ] **Step 5: Build and inspect firmware files**

Run QEMU paused with the dual topology, use QMP `qom-list` to confirm the CXL
objects, and use U-Boot in the next plan to read the actual `fw_cfg` file list.
At this layer, `ninja` must succeed with no duplicate ACPI builder symbols.

- [ ] **Step 6: Commit**

```bash
git add include/hw/riscv/sifive_u-acpi.h \
        hw/riscv/sifive_u-acpi-build.c hw/riscv/meson.build \
        include/hw/pci-host/gpex.h hw/pci-host/gpex-acpi.c \
        hw/riscv/sifive_u.c
git commit -m "hw/riscv/sifive_u: publish CXL ACPI through fw_cfg"
```

### Task 6: Add direct `fw_cfg` ACPI and full QEMU regressions

**Files:**
- Create: `qemu-cxl-type2/tests/qtest/sifive-u-cxl-test.c`
- Modify: `qemu-cxl-type2/tests/qtest/meson.build`
- Modify: `qemu-cxl-type2/tests/functional/test_riscv64_sifive_u.py`
- Modify: `qemu-cxl-type2/tests/functional/meson.build`

- [ ] **Step 1: Add an MMIO `fw_cfg` reader**

Launch the Task 4 dual topology under qtest and access the SiFive U layout
directly:

```c
#define SIFIVE_U_FW_CFG_DATA 0x10100000ULL
#define SIFIVE_U_FW_CFG_SELECTOR (SIFIVE_U_FW_CFG_DATA + 8)

static void fw_cfg_select(QTestState *qts, uint16_t key)
{
    qtest_writew(qts, SIFIVE_U_FW_CFG_SELECTOR, cpu_to_be16(key));
}

static void fw_cfg_read(QTestState *qts, uint16_t key, void *buf, size_t len)
{
    uint8_t *p = buf;

    fw_cfg_select(qts, key);
    for (size_t i = 0; i < len; i++)
        p[i] = qtest_readb(qts, SIFIVE_U_FW_CFG_DATA);
}
```

Read `FW_CFG_FILE_DIR`, convert its count and entry sizes from big endian, and
provide:

```c
static GByteArray *fw_cfg_file(QTestState *qts, const char *name);
```

- [ ] **Step 2: Assert the ACPI files and CXL objects**

Require nonempty:

```text
etc/acpi/tables
etc/table-loader
etc/acpi/rsdp
```

In `etc/acpi/tables`, walk `AcpiTableHeader.length` from the DSDT start and
assert one each of `DSDT`, `FACP`, `APIC`, `RHCT`, `MCFG`, and `CEDT`.
Assert the blob contains the AML names `ACPI0016`, `ACPI0017`, `CXLM`, and
`_DEP`. Parse CEDT subtable headers and require at least one CHBS and one CFMWS
whose base/size are `0x1000000000/4 GiB`.

- [ ] **Step 3: Register and run the QTest**

Append `sifive-u-cxl-test` to `qtests_riscv64` when `CONFIG_SIFIVE_U`,
`CONFIG_CXL`, and `CONFIG_FW_CFG_DMA` are present.

Run:

```bash
cd /root/cxl-u-boot/qemu-cxl-type2
ninja -C build-riscv-cxl tests/qtest/sifive-u-cxl-test
build-riscv-cxl/tests/qtest/sifive-u-cxl-test
```

Expected: PASS without EDK2 or U-Boot.

- [ ] **Step 4: Run the complete QEMU gate**

```bash
ninja -C build-riscv-cxl
ninja -C build-riscv-cxl test
build-riscv-cxl/pyvenv/bin/python3 tests/functional/test_riscv64_sifive_u.py
```

Expected: build and focused tests PASS. Record unrelated baseline failures
separately and do not mask failures in the new tests.

- [ ] **Step 5: Commit**

```bash
git add tests/qtest/sifive-u-cxl-test.c tests/qtest/meson.build \
        tests/functional/test_riscv64_sifive_u.py \
        tests/functional/meson.build
git commit -m "tests/riscv: validate sifive_u CXL firmware tables"
```

### Task 7: Document the machine extension

**Files:**
- Modify: `qemu-cxl-type2/docs/system/riscv/sifive_u.rst`

- [ ] **Step 1: Document compatibility and the reference topology**

Add the exact statement:

```rst
Synthetic PCIe and CXL
----------------------

``-M sifive_u,cxl=on`` adds a QEMU-only GPEX PCIe root complex and CXL
platform facilities.  These devices are not present in physical HiFive
Unleashed hardware.  The default ``cxl=off`` machine remains unchanged.
```

Include the complete Task 4 command-line topology.

- [ ] **Step 2: Build documentation syntax**

Run:

```bash
cd /root/cxl-u-boot/qemu-cxl-type2
ninja -C build-riscv-cxl qemu-doc
```

Expected: no RST warnings from `sifive_u.rst`. If docs were configured off,
run `python3 scripts/checkpatch.pl` on the final commit range instead.

- [ ] **Step 3: Commit**

```bash
git add docs/system/riscv/sifive_u.rst
git commit -m "docs/riscv: describe synthetic sifive_u CXL"
```
