# SiFive U CXL U-Boot Firmware Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a SiFive U QEMU firmware that enumerates the synthetic PCIe hierarchy, safely inspects Type 2 and Type 3 CXL capabilities, loads QEMU ACPI tables, and launches Linux through EFI.

**Architecture:** A dedicated defconfig preserves the physical-board configuration while enabling generic ECAM, QFW MMIO, ACPI, and EFI. A read-only CXL library validates PCIe DVSEC/Register Locator data and component HDM registers; `cxl list` and `cxl info` expose it without programming decoders.

**Tech Stack:** U-Boot driver model PCI, generic ECAM, QEMU `fw_cfg`, ACPI table-loader, EFI loader, sandbox unit tests, pytest/pexpect serial tests.

---

## File map

- Create `u-boot/configs/sifive_unleashed_qemu_cxl_defconfig`: S-mode QEMU/DT/PCI/QFW/ACPI/EFI configuration.
- Modify `u-boot/drivers/pci/Kconfig`: define the read-only `CXL_PCI` library switch.
- Modify `u-boot/drivers/pci/Makefile`: compile the CXL discovery library.
- Create `u-boot/include/cxl.h`: constants, parsed structures, callback interface, and discovery API.
- Create `u-boot/drivers/pci/cxl.c`: guarded extended-capability traversal, DVSEC parsing, Register Locator validation, BAR mapping, and HDM reads.
- Modify `u-boot/cmd/Kconfig`: define `CMD_CXL`.
- Modify `u-boot/cmd/Makefile`: compile the command.
- Create `u-boot/cmd/cxl.c`: implement `cxl list` and `cxl info`.
- Create `u-boot/test/lib/cxl.c`: parser tests independent of live PCI hardware.
- Modify `u-boot/test/lib/Makefile`: register the parser tests.
- Create `u-boot/test/py/tests/test_sifive_u_cxl.py`: live QEMU command and ACPI/EFI assertions.
- Create `u-boot/doc/board/sifive/unleashed-qemu-cxl.rst`: build and command reference.

### Task 1: Add a dedicated QEMU CXL defconfig

**Files:**
- Create: `u-boot/configs/sifive_unleashed_qemu_cxl_defconfig`

- [ ] **Step 1: Generate the configuration from the proven S-mode build**

Run:

```bash
cd /root/cxl-u-boot/u-boot
make O=build_sifive_u_cxl sifive_unleashed_defconfig
scripts/config --file build_sifive_u_cxl/.config \
  -e OF_BOARD -d BINMAN_FDT -d SPL \
  -e PCI -e PCI_PNP -e SYS_PCI_64BIT -e PCIE_ECAM_GENERIC \
  -e CMD_PCI -e CMD_QFW -e QFW_MMIO \
  -e ACPI -e GENERATE_ACPI_TABLE -e CMD_ACPI \
  -e EFI_LOADER -e CMD_BOOTEFI -e CMD_EFIDEBUG \
  -e VIRTIO -e VIRTIO_PCI -e VIRTIO_BLK -e CMD_VIRTIO
make O=build_sifive_u_cxl olddefconfig
```

Expected before the new defconfig exists: the generated `.config` contains
all requested symbols but `CONFIG_CXL_PCI` and `CONFIG_CMD_CXL` are absent.

- [ ] **Step 2: Add the new defconfig**

Use the physical `sifive_unleashed_defconfig` content, then apply these exact
differences:

```text
CONFIG_OF_BOARD=y
# CONFIG_BINMAN_FDT is not set
# CONFIG_SPL is not set
CONFIG_PCI=y
CONFIG_PCI_PNP=y
CONFIG_SYS_PCI_64BIT=y
CONFIG_PCIE_ECAM_GENERIC=y
CONFIG_CMD_PCI=y
CONFIG_CMD_QFW=y
CONFIG_QFW_MMIO=y
CONFIG_ACPI=y
CONFIG_GENERATE_ACPI_TABLE=y
CONFIG_CMD_ACPI=y
CONFIG_EFI_LOADER=y
CONFIG_CMD_BOOTEFI=y
CONFIG_CMD_EFIDEBUG=y
CONFIG_VIRTIO=y
CONFIG_VIRTIO_PCI=y
CONFIG_VIRTIO_BLK=y
CONFIG_CMD_VIRTIO=y
```

Keep `CONFIG_RISCV_SMODE=y` and the existing `CONFIG_PREBOOT` that points
`fdt_addr` at `fdtcontroladdr`.

- [ ] **Step 3: Validate configuration semantics**

Run:

```bash
make O=build_sifive_u_cxl sifive_unleashed_qemu_cxl_defconfig
rg 'CONFIG_(OF_BOARD|PCI|SYS_PCI_64BIT|PCIE_ECAM_GENERIC|QFW_MMIO|GENERATE_ACPI_TABLE|CMD_ACPI|EFI_LOADER|VIRTIO_PCI|VIRTIO_BLK)=' \
  build_sifive_u_cxl/.config
```

Expected: every listed symbol is `y`; `CONFIG_SPL` is unset.

- [ ] **Step 4: Commit**

```bash
git add configs/sifive_unleashed_qemu_cxl_defconfig
git commit -m "configs: add SiFive U QEMU CXL firmware"
```

### Task 2: Define the bounded CXL parser API

**Files:**
- Modify: `u-boot/drivers/pci/Kconfig`
- Modify: `u-boot/drivers/pci/Makefile`
- Modify: `u-boot/configs/sifive_unleashed_qemu_cxl_defconfig`
- Create: `u-boot/include/cxl.h`
- Create: `u-boot/drivers/pci/cxl.c`
- Create: `u-boot/test/lib/cxl.c`
- Modify: `u-boot/test/lib/Makefile`

- [ ] **Step 1: Add failing synthetic config-space tests**

Create a 4 KiB byte array and a callback:

```c
static int cfg_read(void *ctx, uint off, void *value, size_t size)
{
    struct cxl_test_cfg *cfg = ctx;

    if (off + size > sizeof(cfg->data))
        return -ERANGE;
    memcpy(value, cfg->data + off, size);
    return 0;
}
```

Add tests for:

```c
ut_assertok(cxl_find_dvsec(&reader, PCI_VENDOR_ID_CXL,
                           CXL_DVSEC_PCIE_DEVICE, &dvsec));
ut_asserteq(0x100, dvsec.offset);
ut_asserteq(-ELOOP, cxl_find_dvsec(&loop_reader, PCI_VENDOR_ID_CXL,
                                   CXL_DVSEC_PCIE_DEVICE, &dvsec));
ut_asserteq(-EINVAL, cxl_find_dvsec(&bad_next_reader,
                                    PCI_VENDOR_ID_CXL,
                                    CXL_DVSEC_PCIE_DEVICE, &dvsec));
```

Register them with `LIB_TEST()` and `obj-$(CONFIG_CXL_PCI) += cxl.o`.

Run:

```bash
cd /root/cxl-u-boot/u-boot
make O=build-sandbox sandbox_defconfig
scripts/config --file build-sandbox/.config -e CXL_PCI
make O=build-sandbox olddefconfig
make O=build-sandbox -j"$(nproc)"
build-sandbox/u-boot -T -c "ut lib cxl"
```

Expected: compile failure because `cxl.h` and its API do not exist.

- [ ] **Step 2: Add Kconfig and build wiring**

Add:

```text
config CXL_PCI
    bool "Read-only CXL PCI capability discovery"
    depends on PCI
    help
      Parse CXL PCIe DVSEC and register locator capabilities and inspect
      component HDM decoder state without programming the device.
```

Add:

```make
obj-$(CONFIG_CXL_PCI) += cxl.o
```

Append to `sifive_unleashed_qemu_cxl_defconfig`:

```text
CONFIG_CXL_PCI=y
```

- [ ] **Step 3: Define exact constants and parsed types**

Create `include/cxl.h` with:

```c
#define PCI_EXT_CAP_ID_DVSEC                    0x23
#define PCI_VENDOR_ID_CXL                       0x1e98
#define PCI_DVSEC_HEADER1                       0x4
#define PCI_DVSEC_HEADER2                       0x8
#define PCI_DVSEC_HEADER1_VID(v)                ((v) & 0xffff)
#define PCI_DVSEC_HEADER1_REV(v)                (((v) >> 16) & 0xf)
#define PCI_DVSEC_HEADER1_LEN(v)                (((v) >> 20) & 0xfff)
#define PCI_DVSEC_HEADER2_ID(v)                 ((v) & 0xffff)
#define CXL_DVSEC_PCIE_DEVICE                   0
#define CXL_DVSEC_REG_LOCATOR                   8
#define CXL_DVSEC_CAP_OFFSET                    0x0a
#define CXL_DVSEC_CACHE_CAPABLE                 BIT(0)
#define CXL_DVSEC_IO_CAPABLE                    BIT(1)
#define CXL_DVSEC_MEM_CAPABLE                   BIT(2)
#define CXL_DVSEC_REG_LOCATOR_BLOCK1_OFFSET     0x0c
#define CXL_REGLOC_BIR(v)                       ((v) & GENMASK(2, 0))
#define CXL_REGLOC_RBI(v)                       (((v) >> 8) & 0xff)
#define CXL_REGLOC_OFF_LOW(v)                   ((v) & GENMASK(31, 16))
#define CXL_REGLOC_RBI_COMPONENT                1
#define CXL_REGLOC_RBI_MEMDEV                   3
#define CXL_HDM_DECODER_CAP_OFFSET              0
#define CXL_HDM_DECODER_CTRL_OFFSET             4
#define CXL_HDM_DECODER_COUNT(v) \
    (((v) & GENMASK(3, 0)) ? (((v) & GENMASK(3, 0)) * 2) : 1)

enum cxl_device_type {
    CXL_DEVICE_UNKNOWN,
    CXL_DEVICE_TYPE2,
    CXL_DEVICE_TYPE3,
};

struct cxl_cfg_reader {
    void *ctx;
    int (*read)(void *ctx, uint off, void *value, size_t size);
};

struct cxl_dvsec {
    uint offset;
    u16 vendor;
    u16 id;
    u8 revision;
    u16 length;
};

struct cxl_regloc {
    u8 bir;
    u8 rbi;
    u64 offset;
};

struct cxl_device_info {
    pci_dev_t bdf;
    u16 vendor;
    u16 device;
    u32 class_code;
    enum cxl_device_type type;
    struct cxl_dvsec device_dvsec;
    struct cxl_regloc component;
    struct cxl_regloc device_regs;
    phys_addr_t component_base;
    u8 hdm_decoder_count;
    u32 hdm_cap;
    u32 hdm_ctrl;
};
```

Declare:

```c
int cxl_find_dvsec(const struct cxl_cfg_reader *reader, u16 vendor, u16 id,
                   struct cxl_dvsec *result);
int cxl_parse_regloc(const struct cxl_cfg_reader *reader,
                     struct cxl_regloc *entries, size_t capacity,
                     size_t *count);
int cxl_probe_device(struct udevice *dev, struct cxl_device_info *info);
const char *cxl_type_name(enum cxl_device_type type);
```

- [ ] **Step 4: Implement guarded extended-capability traversal**

Use:

```c
int cxl_find_dvsec(const struct cxl_cfg_reader *r, u16 vendor, u16 id,
                   struct cxl_dvsec *out)
{
    DECLARE_BITMAP(visited, PCI_CFG_SPACE_EXP_SIZE / 4);
    uint pos = PCI_CFG_SPACE_SIZE;
    uint hops = 0;

    bitmap_zero(visited, PCI_CFG_SPACE_EXP_SIZE / 4);
    while (pos) {
        u32 hdr, h1, h2;
        uint next;
        int ret;

        if (pos < PCI_CFG_SPACE_SIZE || pos > PCI_CFG_SPACE_EXP_SIZE - 4 ||
            (pos & 3) || hops++ >= 960)
            return -EINVAL;
        if (test_and_set_bit(pos / 4, visited))
            return -ELOOP;
        ret = r->read(r->ctx, pos, &hdr, sizeof(hdr));
        if (ret)
            return ret;
        hdr = le32_to_cpu(hdr);
        if (hdr == 0 || hdr == 0xffffffff)
            return -ENOENT;
        next = PCI_EXT_CAP_NEXT(hdr);
        if (PCI_EXT_CAP_ID(hdr) == PCI_EXT_CAP_ID_DVSEC) {
            r->read(r->ctx, pos + PCI_DVSEC_HEADER1, &h1, sizeof(h1));
            r->read(r->ctx, pos + PCI_DVSEC_HEADER2, &h2, sizeof(h2));
            h1 = le32_to_cpu(h1);
            h2 = le32_to_cpu(h2);
            if (PCI_DVSEC_HEADER1_LEN(h1) < 12 ||
                pos + PCI_DVSEC_HEADER1_LEN(h1) > PCI_CFG_SPACE_EXP_SIZE)
                return -EBADMSG;
            if (PCI_DVSEC_HEADER1_VID(h1) == vendor &&
                PCI_DVSEC_HEADER2_ID(h2) == id) {
                *out = (struct cxl_dvsec) {
                    .offset = pos,
                    .vendor = vendor,
                    .id = id,
                    .revision = PCI_DVSEC_HEADER1_REV(h1),
                    .length = PCI_DVSEC_HEADER1_LEN(h1),
                };
                return 0;
            }
        }
        pos = next;
    }
    return -ENOENT;
}
```

Propagate callback failures for both DVSEC header reads instead of ignoring
them.

- [ ] **Step 5: Run the parser tests**

Run the sandbox command from Step 1. Expected: valid, loop, misaligned-next,
short-length, and out-of-bounds cases PASS.

- [ ] **Step 6: Commit**

```bash
git add drivers/pci/Kconfig drivers/pci/Makefile \
        configs/sifive_unleashed_qemu_cxl_defconfig include/cxl.h \
        drivers/pci/cxl.c test/lib/cxl.c test/lib/Makefile
git commit -m "pci: add bounded CXL DVSEC parser"
```

### Task 3: Parse Register Locator entries and map component registers

**Files:**
- Modify: `u-boot/drivers/pci/cxl.c`
- Modify: `u-boot/test/lib/cxl.c`

- [ ] **Step 1: Add failing Register Locator tests**

Construct a DVSEC ID 8 with two 8-byte entries:

```c
put_unaligned_le32(0 |
                   (CXL_REGLOC_RBI_COMPONENT << 8) | 0x00010000,
                   &cfg.data[0x10c]);
put_unaligned_le32(0, &cfg.data[0x110]);
put_unaligned_le32(2 |
                   (CXL_REGLOC_RBI_MEMDEV << 8) | 0x00020000,
                   &cfg.data[0x114]);
put_unaligned_le32(0, &cfg.data[0x118]);
```

Assert two decoded entries, BIR values 0 and 2, and offsets `0x10000` and
`0x20000`. Add failures for BIR > 5, non-multiple-of-8 payload, and capacity
overflow.

- [ ] **Step 2: Implement Register Locator parsing**

Find DVSEC ID 8, then for each entry:

```c
entries[i].bir = CXL_REGLOC_BIR(lo);
entries[i].rbi = CXL_REGLOC_RBI(lo);
entries[i].offset = ((u64)hi << 32) | CXL_REGLOC_OFF_LOW(lo);
```

Reject invalid BIR, truncated payload, duplicate component blocks, and offsets
that wrap when added to a BAR.

- [ ] **Step 3: Implement live PCI reads and BAR validation**

Use `dm_pci_read_config{8,16,32}` in the reader. Resolve a BAR without writing
all ones to it:

```c
static int cxl_bar_base(struct udevice *dev, u8 bir, u64 *base)
{
    u32 lo, hi = 0;

    if (bir > 5)
        return -EINVAL;
    dm_pci_read_config32(dev, PCI_BASE_ADDRESS_0 + bir * 4, &lo);
    if (lo & PCI_BASE_ADDRESS_SPACE_IO)
        return -ENXIO;
    if ((lo & PCI_BASE_ADDRESS_MEM_TYPE_MASK) == PCI_BASE_ADDRESS_MEM_TYPE_64) {
        if (bir == 5)
            return -EINVAL;
        dm_pci_read_config32(dev, PCI_BASE_ADDRESS_0 + (bir + 1) * 4, &hi);
    }
    *base = ((u64)hi << 32) | (lo & PCI_BASE_ADDRESS_MEM_MASK);
    return *base ? 0 : -ENXIO;
}
```

U-Boot does not retain a generic read-only BAR-size value after PCI
autoconfiguration, and probing the size would require the forbidden
all-ones BAR write. Check `offset + len` for integer overflow, then map through
the PCI uclass so the complete access must fall inside a declared host memory
range:

```c
ptr = dm_pci_map_bar(dev, PCI_BASE_ADDRESS_0 + bir * 4,
                     offset, len, PCI_REGION_TYPE,
                     PCI_REGION_MEM);
if (!ptr)
    return -ERANGE;
```

Do not call `map_sysmem()` directly. The live command remains read-only; the
sandbox reader tests provide the stricter malformed-DVSEC and synthetic-BAR
boundary cases that cannot be discovered from a generic live PCI function
without destructive BAR probing.

- [ ] **Step 4: Read HDM capability without writes**

Read component capability headers to locate the HDM block, then cache only:

```c
info->hdm_cap = readl(hdm + CXL_HDM_DECODER_CAP_OFFSET);
info->hdm_ctrl = readl(hdm + CXL_HDM_DECODER_CTRL_OFFSET);
info->hdm_decoder_count = CXL_HDM_DECODER_COUNT(info->hdm_cap);
```

Do not call `writel()`, `dm_pci_write_config*()`, or modify decoder state in
this library.

- [ ] **Step 5: Classify Type 2 and Type 3**

Read class code and Device DVSEC capability. Classify:

```c
if ((cap & CXL_DVSEC_CACHE_CAPABLE) && (cap & CXL_DVSEC_MEM_CAPABLE))
    info->type = CXL_DEVICE_TYPE2;
else if (!(cap & CXL_DVSEC_CACHE_CAPABLE) &&
         (cap & CXL_DVSEC_MEM_CAPABLE))
    info->type = CXL_DEVICE_TYPE3;
else
    info->type = CXL_DEVICE_UNKNOWN;
```

Return `-EUCLEAN` for a class-code/DVSEC mismatch while retaining the parsed
raw fields for diagnostic output.

- [ ] **Step 6: Run sandbox tests and a write-symbol audit**

```bash
build-sandbox/u-boot -T -c "ut lib cxl"
! rg -n 'writel|dm_pci_write_config|pci_bus_write_config' drivers/pci/cxl.c
```

Expected: all tests PASS; the audit produces no match.

- [ ] **Step 7: Commit**

```bash
git add drivers/pci/cxl.c test/lib/cxl.c
git commit -m "pci/cxl: inspect register locators and HDM state"
```

### Task 4: Add `cxl list` and `cxl info`

**Files:**
- Modify: `u-boot/cmd/Kconfig`
- Modify: `u-boot/cmd/Makefile`
- Modify: `u-boot/configs/sifive_unleashed_qemu_cxl_defconfig`
- Create: `u-boot/cmd/cxl.c`
- Create: `u-boot/test/py/tests/test_sifive_u_cxl.py`

- [ ] **Step 1: Add failing command tests**

In the live QEMU test, wait for the prompt and assert:

```python
assert 'Unknown command' in console.run_command('cxl list')
```

After implementation, replace it with:

```python
out = console.run_command('cxl list')
assert 'Type 2' in out
assert 'Type 3' in out
assert 'HDM' in out
```

- [ ] **Step 2: Add command Kconfig and Makefile entries**

Add:

```text
config CMD_CXL
    bool "cxl - inspect CXL PCI devices"
    depends on CXL_PCI
```

and:

```make
obj-$(CONFIG_CMD_CXL) += cxl.o
```

Append:

```text
CONFIG_CMD_CXL=y
```

to `sifive_unleashed_qemu_cxl_defconfig`.

- [ ] **Step 3: Implement BDF parsing**

Accept only `bb.dd.f` in hexadecimal:

```c
static int parse_bdf(const char *arg, pci_dev_t *bdf)
{
    uint bus, dev, fn;
    char tail;

    if (sscanf(arg, "%x.%x.%x%c", &bus, &dev, &fn, &tail) != 3 ||
        bus > 0xff || dev > 0x1f || fn > 7)
        return -EINVAL;
    *bdf = PCI_BDF(bus, dev, fn);
    return 0;
}
```

- [ ] **Step 4: Implement `cxl list`**

Probe all devices in `UCLASS_PCI_GENERIC` after `pci_init()`. Print:

```text
BDF       vendor:device  type    rev  component-regs    device-regs       HDM
40.00.0   8086:0d92      Type 2  1    000000007xxxxxxx  00000000--------  4
41.00.0   1e98:xxxx      Type 3  1    000000007xxxxxxx  000000007xxxxxxx  4
```

Warnings for malformed devices go to the console and do not stop scanning
other devices. Return failure only if no valid CXL endpoint is found.

- [ ] **Step 5: Implement `cxl info`**

Print vendor/device/class, Device DVSEC capability bits, every Register Locator
entry, component base, HDM capability/control, and each decoder's base/size/
control raw value. Validate all addresses before reading and return
`CMD_RET_FAILURE` for malformed input, missing BDF, non-CXL device, or mapping
failure.

- [ ] **Step 6: Register the command**

Use:

```c
U_BOOT_CMD_WITH_SUBCMDS(cxl, "CXL device inspection",
    U_BOOT_SUBCMD_MKENT(list, 1, 1, do_cxl_list),
    U_BOOT_SUBCMD_MKENT(info, 2, 1, do_cxl_info));
```

- [ ] **Step 7: Build and run live command tests**

```bash
cd /root/cxl-u-boot/u-boot
make O=build_sifive_u_cxl \
  DTC=/usr/bin/dtc PYTHON3=/usr/bin/python3 -j"$(nproc)"
pytest -q test/py/tests/test_sifive_u_cxl.py -k 'pci or cxl'
```

Expected: PCI enumeration, Type 2, Type 3, `cxl list`, and both `cxl info`
commands PASS; malformed BDF and a non-CXL BDF return failure.

- [ ] **Step 8: Commit**

```bash
git add cmd/Kconfig cmd/Makefile cmd/cxl.c \
        configs/sifive_unleashed_qemu_cxl_defconfig \
        test/py/tests/test_sifive_u_cxl.py
git commit -m "cmd: add read-only CXL inspection"
```

### Task 5: Prove QFW ACPI loading and EFI registration

**Files:**
- Modify: `u-boot/test/py/tests/test_sifive_u_cxl.py`

- [ ] **Step 1: Add firmware-table assertions**

Run and assert:

```text
qfw list
```

contains `etc/acpi/tables`, `etc/table-loader`, and `etc/acpi/rsdp`.

Assert:

```text
acpi list
```

contains `RSDP`, `XSDT`, `DSDT`, and `CEDT`.

Assert:

```text
efidebug tables
```

contains the ACPI 2.0 table GUID
`8868e871-e4f1-11d3-bc22-0080c73c8881`.

- [ ] **Step 2: Add explicit failure checks**

Boot once with a test QEMU build that omits `etc/table-loader`; assert U-Boot
prints `error: can't find etc/table-loader`, `acpi list` has no CEDT, and the
test exits failed before `bootefi`.

- [ ] **Step 3: Run the ACPI/EFI test**

```bash
cd /root/cxl-u-boot/u-boot
pytest -q test/py/tests/test_sifive_u_cxl.py -k 'qfw or acpi or efi'
```

Expected: standard path PASS; injected missing-table path fails closed.

- [ ] **Step 4: Commit**

```bash
git add test/py/tests/test_sifive_u_cxl.py
git commit -m "test: verify SiFive U CXL ACPI EFI handoff"
```

### Task 6: Document and run the U-Boot gate

**Files:**
- Create: `u-boot/doc/board/sifive/unleashed-qemu-cxl.rst`

- [ ] **Step 1: Document build and commands**

Include:

```bash
make O=build_sifive_u_cxl sifive_unleashed_qemu_cxl_defconfig
make O=build_sifive_u_cxl DTC=/usr/bin/dtc PYTHON3=/usr/bin/python3 -j$(nproc)
```

Document `pci enum`, `pci`, `qfw list`, `acpi list`, `efidebug tables`,
`cxl list`, `cxl info <bb.dd.f>`, and `bootefi`.

- [ ] **Step 2: Run all relevant tests**

```bash
cd /root/cxl-u-boot/u-boot
make O=build-sandbox -j"$(nproc)"
build-sandbox/u-boot -T -c "ut lib cxl"
make O=build_sifive_u_cxl DTC=/usr/bin/dtc PYTHON3=/usr/bin/python3 \
  -j"$(nproc)"
pytest -q test/py/tests/test_sifive_u_cxl.py
sha256sum build_sifive_u_cxl/u-boot.bin
```

Expected: sandbox tests, firmware build, and live tests PASS; hash is captured.

- [ ] **Step 3: Commit**

```bash
git add doc/board/sifive/unleashed-qemu-cxl.rst
git commit -m "doc: describe SiFive U CXL U-Boot flow"
```
