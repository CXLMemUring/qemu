# SiFive U CXL RISC-V Linux Guest Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a RISC-V EFI Linux image and initramfs that consume U-Boot's ACPI handoff, enumerate both CXL endpoint types, create a Type 3 region, verify a memory pattern, and online the region as system RAM.

**Architecture:** Linux receives RSDP from U-Boot's EFI system table and uses CEDT rather than a private DT CXL binding. A one-line ACPI PCI supplier completion fix enforces ACPI0017 `_DEP` ordering; a pinned Buildroot external tree supplies RISC-V `cxl`, `daxctl`, `lspci`, udev, and a deterministic guest proof service.

**Tech Stack:** RISC-V Linux EFI stub, ACPI PCI/CXL, Buildroot 2026.05.1, ndctl 83 tools, BusyBox/eudev, device DAX, shell and a small mmap pattern utility.

---

## File map

- Modify `linux-cxl-type2/drivers/acpi/pci_root.c`: clear ACPI consumers after a PCI root supplier is active.
- Create `linux-cxl-type2/tools/testing/cxl/sifive_u/kernel.config`: EFI/ACPI/PCI/CXL/DAX kernel fragment.
- Create `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/external.desc`: name the Buildroot external tree.
- Create `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/Config.in`: include the local proof package.
- Create `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/external.mk`: include local package makefiles.
- Create `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/configs/sifive_u_cxl_defconfig`: RISC-V initramfs with ndctl and pciutils.
- Create `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/package/cxl-pattern/Config.in`: select the helper.
- Create `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/package/cxl-pattern/cxl-pattern.mk`: compile/install it.
- Create `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/package/cxl-pattern/src/cxl-pattern.c`: bounded DAX mmap/write/read proof.
- Create `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/board/overlay/etc/init.d/S99cxl-proof`: run topology, region, pattern, and online checks.
- Create `linux-cxl-type2/tools/testing/cxl/sifive_u/build-guest.sh`: fetch the pinned Buildroot tag and build rootfs plus local kernel.

### Task 1: Apply the ACPI PCI dependency completion fix

**Files:**
- Modify: `linux-cxl-type2/drivers/acpi/pci_root.c`

- [ ] **Step 1: Record the missing call**

Run:

```bash
cd /root/cxl-u-boot/linux-cxl-type2
sed -n '735,775p' drivers/acpi/pci_root.c
```

Expected: `pci_bus_add_devices(root->bus);` is followed by unlock and
`return 1;`, with no `acpi_dev_clear_dependencies(device)`.

- [ ] **Step 2: Add the supplier completion call**

Change the successful tail to:

```c
pci_lock_rescan_remove();
pci_bus_add_devices(root->bus);
pci_unlock_rescan_remove();

/*
 * Notify _DEP consumers only after the PCI namespace for this root bridge
 * is complete. ACPI0017 depends on every ACPI0016 CXL host bridge.
 */
acpi_dev_clear_dependencies(device);
return 1;
```

Do not call it on an error path.

- [ ] **Step 3: Compile the changed object**

Run:

```bash
make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- defconfig
scripts/config -e ACPI -e PCI
make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- olddefconfig
make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- -j"$(nproc)" \
  drivers/acpi/pci_root.o
```

Expected: object compiles without implicit declarations or unused warnings.

- [ ] **Step 4: Commit**

```bash
git add drivers/acpi/pci_root.c
git commit -m "ACPI: PCI: clear dependencies after root enumeration"
```

### Task 2: Define the reproducible RISC-V kernel configuration

**Files:**
- Create: `linux-cxl-type2/tools/testing/cxl/sifive_u/kernel.config`

- [ ] **Step 1: Write the configuration fragment**

Create:

```text
CONFIG_64BIT=y
CONFIG_RISCV=y
CONFIG_MMU=y
CONFIG_SMP=y
CONFIG_EFI=y
CONFIG_EFI_STUB=y
CONFIG_ACPI=y
CONFIG_ACPI_TABLE_LIB=y
CONFIG_ACPI_NUMA=y
CONFIG_NUMA=y
CONFIG_PCI=y
CONFIG_PCI_HOST_GENERIC=y
CONFIG_PCI_ECAM=y
CONFIG_DEVTMPFS=y
CONFIG_DEVTMPFS_MOUNT=y
CONFIG_CXL_BUS=y
CONFIG_CXL_PCI=y
CONFIG_CXL_ACPI=y
CONFIG_CXL_MEM=y
CONFIG_CXL_PORT=y
CONFIG_CXL_REGION=y
CONFIG_CXL_PMEM=y
CONFIG_CXL_TYPE2_ACCEL=y
CONFIG_LIBNVDIMM=y
CONFIG_DEV_DAX=y
CONFIG_DEV_DAX_CXL=y
CONFIG_DEV_DAX_KMEM=y
CONFIG_MEMORY_HOTPLUG=y
CONFIG_MEMORY_HOTREMOVE=y
CONFIG_ZONE_DEVICE=y
CONFIG_BLK_DEV_INITRD=y
CONFIG_SERIAL_SIFIVE=y
CONFIG_SERIAL_SIFIVE_CONSOLE=y
CONFIG_HVC_RISCV_SBI=y
CONFIG_EARLY_PRINTK=y
CONFIG_DEBUG_FS=y
```

- [ ] **Step 2: Merge and validate symbols**

Run:

```bash
cd /root/cxl-u-boot/linux-cxl-type2
make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- defconfig
scripts/kconfig/merge_config.sh -m .config \
  tools/testing/cxl/sifive_u/kernel.config
make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- olddefconfig
scripts/config --state EFI_STUB
scripts/config --state ACPI
scripts/config --state CXL_ACPI
scripts/config --state CXL_REGION
scripts/config --state CXL_TYPE2_ACCEL
scripts/config --state DEV_DAX_KMEM
```

Expected: every query prints `y`. If a dependency changes one symbol, inspect
its Kconfig, add the concrete missing dependency to the fragment, and rerun
until all six are `y`.

- [ ] **Step 3: Build the EFI-stubbed image**

```bash
make ARCH=riscv CROSS_COMPILE=riscv64-linux-gnu- -j"$(nproc)" Image
file arch/riscv/boot/Image
```

Expected: successful RISC-V kernel build; EFI stub support is present in
`.config`.

- [ ] **Step 4: Commit**

```bash
git add tools/testing/cxl/sifive_u/kernel.config
git commit -m "tools/cxl: add SiFive U RISC-V guest config"
```

### Task 3: Add the pinned Buildroot external tree

**Files:**
- Create: `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/external.desc`
- Create: `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/Config.in`
- Create: `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/external.mk`
- Create: `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/configs/sifive_u_cxl_defconfig`

- [ ] **Step 1: Add the external-tree metadata**

Create `external.desc`:

```text
name: SIFIVE_U_CXL
desc: SiFive U synthetic CXL guest proof
```

Create `Config.in`:

```text
source "$BR2_EXTERNAL_SIFIVE_U_CXL_PATH/package/cxl-pattern/Config.in"
```

Create `external.mk`:

```make
include $(sort $(wildcard $(BR2_EXTERNAL_SIFIVE_U_CXL_PATH)/package/*/*.mk))
```

- [ ] **Step 2: Add the exact Buildroot defconfig**

Create:

```text
BR2_riscv=y
BR2_RISCV_64=y
BR2_riscv_g=y
BR2_TOOLCHAIN_BUILDROOT_GLIBC=y
BR2_ROOTFS_DEVICE_CREATION_DYNAMIC_EUDEV=y
BR2_INIT_BUSYBOX=y
BR2_TARGET_GENERIC_GETTY_PORT="hvc0"
BR2_PACKAGE_NDCTL=y
BR2_PACKAGE_PCIUTILS=y
BR2_PACKAGE_CXL_PATTERN=y
BR2_TARGET_ROOTFS_CPIO=y
BR2_TARGET_ROOTFS_CPIO_GZIP=y
BR2_ROOTFS_OVERLAY="$(BR2_EXTERNAL_SIFIVE_U_CXL_PATH)/board/overlay"
# BR2_TARGET_ROOTFS_TAR is not set
```

- [ ] **Step 3: Validate against Buildroot 2026.05.1**

Run:

```bash
cd /tmp/buildroot-2026.05.1
make BR2_EXTERNAL=/root/cxl-u-boot/linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external \
  O=/tmp/sifive-u-cxl-br sifive_u_cxl_defconfig
make O=/tmp/sifive-u-cxl-br olddefconfig
rg 'BR2_PACKAGE_(NDCTL|PCIUTILS)|BR2_ROOTFS_DEVICE_CREATION_DYNAMIC_EUDEV|BR2_TARGET_ROOTFS_CPIO' \
  /tmp/sifive-u-cxl-br/.config
```

Expected: ndctl, pciutils, eudev, and CPIO are enabled.

- [ ] **Step 4: Commit**

```bash
git add tools/testing/cxl/sifive_u/br2-external
git commit -m "tools/cxl: define RISC-V CXL proof rootfs"
```

### Task 4: Add the bounded DAX pattern helper

**Files:**
- Create: `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/package/cxl-pattern/Config.in`
- Create: `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/package/cxl-pattern/cxl-pattern.mk`
- Create: `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/package/cxl-pattern/src/cxl-pattern.c`

- [ ] **Step 1: Define the package**

Create `Config.in`:

```text
config BR2_PACKAGE_CXL_PATTERN
    bool "cxl-pattern"
    help
      Write and verify a deterministic pattern in the first 2 MiB of a
      device-DAX mapping.
```

Create `cxl-pattern.mk`:

```make
CXL_PATTERN_VERSION = 1
CXL_PATTERN_SITE = $(BR2_EXTERNAL_SIFIVE_U_CXL_PATH)/package/cxl-pattern/src
CXL_PATTERN_SITE_METHOD = local

define CXL_PATTERN_BUILD_CMDS
	$(TARGET_CC) $(TARGET_CFLAGS) -Wall -Wextra -Werror \
		-o $(@D)/cxl-pattern $(@D)/cxl-pattern.c
endef

define CXL_PATTERN_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/cxl-pattern \
		$(TARGET_DIR)/usr/bin/cxl-pattern
endef

$(eval $(generic-package))
```

- [ ] **Step 2: Implement the exact 2 MiB proof**

The program accepts one device path, uses `length = 2 * 1024 * 1024`, opens
with `O_RDWR | O_SYNC`, mmaps shared read/write, fills every byte with:

```c
static uint8_t expected(size_t i)
{
    return (uint8_t)((i * 131u + 0x5a) & 0xff);
}
```

After `msync()`, read back every byte. On mismatch print:

```text
CXL_PATTERN_FAIL offset=<offset> expected=<hex> actual=<hex>
```

and exit 1. On success print:

```text
CXL_PATTERN_PASS bytes=2097152
```

Always `munmap()` and close the fd.

- [ ] **Step 3: Cross-build only the package**

Run:

```bash
make -C /tmp/buildroot-2026.05.1 O=/tmp/sifive-u-cxl-br cxl-pattern
file /tmp/sifive-u-cxl-br/build/cxl-pattern-1/cxl-pattern
```

Expected: ELF 64-bit RISC-V executable.

- [ ] **Step 4: Commit**

```bash
git add tools/testing/cxl/sifive_u/br2-external/package/cxl-pattern
git commit -m "tools/cxl: add bounded device-DAX pattern check"
```

### Task 5: Add the automatic guest proof service

**Files:**
- Create: `linux-cxl-type2/tools/testing/cxl/sifive_u/br2-external/board/overlay/etc/init.d/S99cxl-proof`

- [ ] **Step 1: Implement fail-closed shell helpers**

Start the script with:

```sh
#!/bin/sh
set -eu

fail()
{
    echo "CXL_PROOF_FAIL stage=$1"
    poweroff -f
    exit 1
}

echo "CXL_PROOF_BEGIN"
dmesg > /var/log/dmesg-cxl.log
```

- [ ] **Step 2: Prove firmware and PCI discovery**

Add:

```sh
grep -q 'ACPI:.*CEDT' /var/log/dmesg-cxl.log || fail acpi-cedt
if grep -q 'unresolved.*_DEP' /var/log/dmesg-cxl.log; then
    fail acpi-dep
fi
lspci -nn
cxl list -M -m -D -d -R -r
test -d /sys/bus/cxl/devices/mem0 || fail mem0
test -d /sys/bus/acpi/devices/ACPI0017:00 || fail acpi0017-sysfs
test -d /sys/bus/pci/devices/0000:41:00.0 || fail type2-pci
```

Use the BDF printed by the final QEMU/U-Boot topology test if bridge numbering
differs; keep the asserted value fixed in the committed script.

- [ ] **Step 3: Create a Type 3 region and verify DAX**

Add:

```sh
cxl enable-memdev mem0 || fail enable-memdev
cxl create-region -m -t ram -d decoder0.0 -w 1 mem0 -s 256M ||
    fail create-region
test -e /dev/dax0.0 || fail dax-device
cxl-pattern /dev/dax0.0 || fail pattern
```

- [ ] **Step 4: Online memory and prove the delta**

Add:

```sh
before=$(awk '/MemTotal:/ {print $2}' /proc/meminfo)
daxctl reconfigure-device --mode=system-ram --no-online dax0.0 ||
    fail reconfigure-system-ram
daxctl online-memory dax0.0 || fail online-memory
after=$(awk '/MemTotal:/ {print $2}' /proc/meminfo)
delta=$((after - before))
test "$delta" -ge 200000 || fail memtotal-delta
echo "CXL_PROOF_PASS mem_kib_before=$before mem_kib_after=$after delta_kib=$delta"
sync
poweroff -f
```

- [ ] **Step 5: Build the CPIO image**

Run:

```bash
make -C /tmp/buildroot-2026.05.1 O=/tmp/sifive-u-cxl-br -j"$(nproc)"
file /tmp/sifive-u-cxl-br/images/rootfs.cpio.gz
```

Expected: gzip-compressed CPIO and installed RISC-V `cxl`, `daxctl`,
`lspci`, and `cxl-pattern`.

- [ ] **Step 6: Commit**

```bash
git add tools/testing/cxl/sifive_u/br2-external/board/overlay/etc/init.d/S99cxl-proof
git commit -m "tools/cxl: automate guest CXL region proof"
```

### Task 6: Add the one-command guest builder

**Files:**
- Create: `linux-cxl-type2/tools/testing/cxl/sifive_u/build-guest.sh`

- [ ] **Step 1: Implement pinned source acquisition**

Use:

```bash
#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
kernel_dir=$(cd -- "$script_dir/../../../.." && pwd)
buildroot_dir=${BUILDROOT_DIR:-/tmp/buildroot-2026.05.1}
buildroot_out=${BUILDROOT_OUT:-/tmp/sifive-u-cxl-br}
buildroot_commit=cb857ba4c87a93e5265a9e4a3f32071abf39e14a

if [[ ! -d "$buildroot_dir/.git" ]]; then
    git clone https://gitlab.com/buildroot.org/buildroot.git "$buildroot_dir"
fi
git -C "$buildroot_dir" fetch --depth 1 origin "$buildroot_commit"
git -C "$buildroot_dir" checkout --detach "$buildroot_commit"
```

- [ ] **Step 2: Build rootfs and use its toolchain for Linux**

Continue with:

```bash
make -C "$buildroot_dir" \
    BR2_EXTERNAL="$script_dir/br2-external" \
    O="$buildroot_out" sifive_u_cxl_defconfig
make -C "$buildroot_dir" O="$buildroot_out" -j"$(nproc)"

cross="$buildroot_out/host/bin/riscv64-buildroot-linux-gnu-"
make -C "$kernel_dir" ARCH=riscv CROSS_COMPILE="$cross" defconfig
(
    cd "$kernel_dir"
    scripts/kconfig/merge_config.sh -m .config "$script_dir/kernel.config"
)
make -C "$kernel_dir" ARCH=riscv CROSS_COMPILE="$cross" olddefconfig
make -C "$kernel_dir" ARCH=riscv CROSS_COMPILE="$cross" \
    -j"$(nproc)" Image

sha256sum "$kernel_dir/arch/riscv/boot/Image" \
          "$buildroot_out/images/rootfs.cpio.gz"
```

- [ ] **Step 3: Run the builder**

```bash
cd /root/cxl-u-boot/linux-cxl-type2
tools/testing/cxl/sifive_u/build-guest.sh
```

Expected: local kernel `Image`, Buildroot CPIO, and two hashes.

- [ ] **Step 4: Commit**

```bash
git add tools/testing/cxl/sifive_u/build-guest.sh
git commit -m "tools/cxl: build reproducible SiFive U guest"
```
