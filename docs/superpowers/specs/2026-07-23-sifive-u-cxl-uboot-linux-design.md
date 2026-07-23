# SiFive U CXL Host Bridge, U-Boot, and Linux Boot Design

**Date:** 2026-07-23  
**Status:** Approved architecture; awaiting final specification review  
**Owners:** `qemu-cxl-type2`, `u-boot`, and `linux-cxl-type2`

## 1. Objective

Extend QEMU's existing SiFive HiFive Unleashed machine without renaming or
replacing it, so the following machine selection remains exact:

```text
qemu-system-riscv64 -M sifive_u
```

The new opt-in form:

```text
qemu-system-riscv64 -M sifive_u,cxl=on
```

must add a synthetic PCIe/CXL hierarchy that does not claim to model physical
HiFive Unleashed hardware. The complete boot chain is:

```text
OpenSBI -> U-Boot EFI -> Linux ACPI/CXL
```

The first implementation must expose both:

- one simulated CXL Type 2 accelerator with device memory; and
- one RAM-backed CXL Type 3 memory device with an LSA backend.

Success means more than PCI enumeration. U-Boot must inspect the CXL devices
and hand QEMU-generated ACPI tables to a RISC-V Linux EFI payload. Linux must
enumerate the CXL host bridges and both endpoints, and the Type 3 path must
support creating and validating a CXL region.

## 2. Constraints

1. The machine type remains `sifive_u`. A new `virt`-derived machine is not an
   acceptable substitute.
2. `cxl=off` is the default. With it, the existing SiFive U memory map,
   generated device tree, boot behavior, and accepted device set remain
   unchanged.
3. CXL devices supplied while `cxl=off` fail at QEMU startup with an actionable
   error instead of creating a partial topology.
4. The host bridge is explicitly synthetic. QEMU documentation and the
   generated device tree identify it as a QEMU extension, not real FU540 or
   HiFive Unleashed hardware.
5. Type 2 must run in self-contained simulation mode. It must not require VFIO,
   a physical GPU, hetGPU, or CXLMemSim.
6. U-Boot CXL discovery is read-only in the first implementation. It must not
   program HDM decoders or add CXL capacity to U-Boot's DRAM allocator.
7. Linux receives CXL topology through ACPI, including CEDT. Device tree
   remains the firmware description used to start U-Boot and to discover the
   synthetic PCIe and `fw_cfg` devices.

## 3. Reference Baselines

The design is based on these local revisions:

- QEMU: `c63f34b42b31502ef5c15e91adb42b965acc6ef2`
- U-Boot: `c53b344475734d0d29f522b7b1d80c5b8204442d`
- Linux: `974621ca7a275580045480fb0f35aa4b2d2c5f46`

It adapts the architecture of the QEMU patch series
`hw/riscv/virt: Add CXL support to RISC-V virt machine`, including:

- the machine-owned `CXLState`;
- CXL host-register and fixed-memory-window setup;
- ACPI0016 CXL host bridges, ACPI0017 CXL root object, and CEDT;
- `_DEP` relationships that delay the ACPI0017 CXL root object until every
  ACPI0016 host bridge is attached; and
- a dedicated 32-bit non-prefetchable MMIO aperture for CXL component BARs.

Those patches target `virt` and EDK2. This design ports the mechanisms to the
SiFive U machine and uses U-Boot's existing QEMU `fw_cfg` ACPI loader and EFI
configuration-table support.

## 4. System Architecture

### 4.1 Boot and description paths

QEMU generates two complementary platform descriptions:

```text
QEMU sifive_u,cxl=on
  |
  +-- DTB -> OpenSBI/U-Boot
  |          |
  |          +-- GPEX PCIe ECAM/ranges/interrupt-map
  |          +-- qemu,fw-cfg-mmio
  |
  +-- fw_cfg ACPI files
             |
             +-- RSDP/XSDT
             +-- DSDT: PCI0 + ACPI0016 CXL host bridges + ACPI0017
             +-- CEDT: CHBS + CFMWS
             |
             +-- U-Boot table-loader relocation/checksum processing
                         |
                         +-- EFI ACPI configuration table -> Linux
```

The DT PCI node enables U-Boot PCI enumeration. The ACPI hierarchy is the
authoritative Linux CXL description. Linux is started as a RISC-V EFI image
using `bootefi`; it discovers RSDP through the EFI system table.

### 4.2 QEMU object ownership

`SiFiveUState` gains, behind the `cxl` machine property:

- a GPEX root-complex device and its root `PCIBus`;
- `CXLState`;
- a memory-mapped `FWCfgState`;
- ACPI build state and machine-done notifier state; and
- explicit resources for ECAM, PCI MMIO, CXL component MMIO, CXL host
  registers, and CXL fixed memory windows.

The generic QEMU CXL framework remains the source of endpoint, root-port,
host-bridge, HDM, and CFMWS behavior. SiFive U supplies only the machine
integration and platform descriptions.

### 4.3 Device topology

The reference launch topology is:

```text
GPEX PCIe root bus (pcie.0)
  |
  +-- pxb-cxl (cxl.1, bus 0x40)
        |
        +-- cxl-rp (rp-t2)
        |     |
        |     +-- cxl-type2 (gpu-mode=none)
        |
        +-- cxl-rp (rp-t3)
              |
              +-- cxl-type3
                    +-- volatile or persistent RAM backend
                    +-- LSA backend
```

`pxb-cxl`, both root ports, and both endpoints remain explicit command-line
devices. `-M sifive_u,cxl=on` creates the root complex and CXL platform
facilities, not hidden endpoints. This keeps topology tests deterministic and
allows later multi-host-bridge and interleave testing.

## 5. Address and Interrupt Layout

The implementation must centralize all synthetic resources in the SiFive U
memory-map enum and reject overlaps during machine initialization. The initial
layout is:

| Resource | Guest physical range | Size |
| --- | ---: | ---: |
| PCI I/O aperture | `0x1008_0000..0x1008_ffff` | 64 KiB |
| PCIe ECAM | `0x3000_0000..0x3fff_ffff` | 256 MiB |
| General PCI MMIO32 | `0x4000_0000..0x6fff_ffff` | 768 MiB |
| CXL component MMIO32 | `0x7000_0000..0x7fff_ffff` | 256 MiB |
| PCI MMIO64 | `0x4_0000_0000..0x7_ffff_ffff` | 16 GiB |
| CXL host registers | immediately above PCI MMIO64 | implementation-sized |
| Default CFMWS | `0x10_0000_0000..0x10_ffff_ffff` | 4 GiB |

The 256 MiB CXL MMIO32 reservation is intentional. CXL component register BARs
are 64-bit non-prefetchable BARs, while PCI bridge non-prefetchable windows are
limited to a 32-bit aperture. The normal PCI MMIO32 window exposed through DT
and ACPI must be shortened so it cannot consume this reservation.

The GPEX interrupt-map routes PCI INTx pins to unused SiFive U PLIC sources.
The exact source numbers are chosen only after checking all existing SiFive U
interrupt assignments. They are constants, covered by compile-time or startup
validation, and emitted consistently into the DTB. MSI/MSI-X support is not a
prerequisite for the first boot proof.

All resources are absent when `cxl=off`.

## 6. QEMU Changes

### 6.1 Build configuration

RISC-V `sifive_u` CXL support selects the generic dependencies required by
GPEX, PXB, and ACPI CXL. The build must support both configurations:

- a non-CXL RISC-V binary that links and runs; and
- a CXL-enabled RISC-V binary containing Type 2 and Type 3.

The current fork generates `zettai-bind-vppb` and `zettai-unbind-vppb` QAPI
dispatch code even when `CONFIG_CXL=n`, while their implementations are
compiled only with CXL. The configuration or stub boundary must be corrected
so non-CXL targets never fail at link time.

### 6.2 Machine property and validation

Add a boolean `cxl` property to `sifive_u`, defaulting to false.

When false:

- skip GPEX, `fw_cfg`, ACPI, CXL host registers, and FMW creation;
- retain the current DT exactly except for unavoidable deterministic ordering
  differences; and
- reject `pxb-cxl`, `cxl-rp`, `cxl-type2`, and `cxl-type3` devices.

When true:

1. create and realize the GPEX host;
2. expose ECAM and PCI apertures;
3. create `fw_cfg` before firmware tables are finalized;
4. initialize machine-owned CXL state;
5. create the host-register aperture and map requested CFMWS ranges;
6. add GPEX and `fw_cfg` nodes to the generated DT;
7. build ACPI tables after the user-specified PCI/CXL graph is complete; and
8. hook PXB CXL host registers and FMW targets in the machine-done callback.

Initialization fails closed on address overlap, duplicate bus number, missing
CXL host target, invalid FMW size/alignment, or a CXL graph attached to the
wrong parent bus.

### 6.3 Device tree

With CXL enabled, QEMU adds:

- a standards-compatible GPEX PCI host node with ECAM `reg`, `bus-range`,
  `ranges`, and PLIC `interrupt-map`; and
- a `qemu,fw-cfg-mmio` node including DMA registers.

The root model and compatible strings continue to identify the SiFive
Unleashed machine. A QEMU-specific property on the PCI node documents that the
bridge is synthetic. Linux is not expected to infer CXL host topology from
this DT node.

### 6.4 ACPI and CEDT

Port the reusable RISC-V `virt` ACPI builders instead of duplicating CXL AML.
The SiFive U table set must provide the minimum complete RISC-V EFI boot
description plus:

- `PNP0A08` for the primary GPEX root;
- `ACPI0016` for each `pxb-cxl` host bridge;
- `ACPI0017` for the CXL root object;
- `_DEP` on ACPI0017 containing every ACPI0016 host bridge;
- static `_CRS` resources for each CXL host bridge, including its reserved
  component MMIO32 range;
- CEDT CHBS entries matching the emulated CXL host-register blocks; and
- CEDT CFMWS entries matching the QEMU fixed memory windows and targets.

ACPI tables, table-loader commands, and RSDP are published through standard
QEMU `fw_cfg` filenames so U-Boot's generic loader can relocate, patch, and
checksum them.

## 7. U-Boot Changes

### 7.1 Board configuration

Create a dedicated QEMU-oriented SiFive U CXL configuration derived from
`sifive_unleashed_defconfig`. It keeps S-mode direct boot and the QEMU-provided
DTB, and enables:

- driver-model PCI and PCIe ECAM support;
- PCI enumeration and `pci` command;
- 64-bit PCI resource handling;
- QEMU MMIO `fw_cfg`;
- ACPI table loading from QEMU;
- EFI loader, `bootefi`, and EFI ACPI registration; and
- the new CXL inspection command.

This configuration does not replace the physical-board defconfig.

### 7.2 PCI host driver binding

The QEMU DT node binds to U-Boot's generic PCI ECAM/GPEX support. During
`pci enum`, U-Boot assigns bridge windows and endpoint BARs within the DT
apertures. Allocation must preserve the CXL component MMIO32 reservation.

The implementation must prove that the primary root, PXB CXL host bridge,
both CXL root ports, and both endpoints have stable BDFs and valid BARs before
CXL-specific parsing starts.

### 7.3 CXL discovery library

Add a small CXL PCI discovery layer that:

1. walks the PCI Express extended-capability list;
2. detects CXL Designated Vendor-Specific Extended Capabilities;
3. validates capability length, next-pointer alignment, bounds, and cycles;
4. parses the CXL capability and status fields needed to classify Type 2 and
   Type 3 devices;
5. parses Register Locator entries and validates referenced BAR numbers and
   offsets;
6. locates component/device register blocks; and
7. reads HDM decoder capability and decoder state without changing it.

Classification cross-checks PCI class code and DVSEC type. Mismatches and
unknown revisions produce warnings and preserve raw values; malformed
capability chains fail for that device without aborting enumeration of other
devices.

### 7.4 Command interface

Add:

```text
cxl list
cxl info <bus.dev.fn>
```

`cxl list` prints one concise row per discovered endpoint:

```text
BDF  vendor:device  type  DVSEC-rev  component-regs  device-regs  HDM-decoders
```

`cxl info` prints validated DVSEC, Register Locator, register-block, and HDM
decoder details. A malformed BDF, non-CXL device, inaccessible BAR, or invalid
capability returns a nonzero command status.

No command in the first implementation writes component registers or HDM
decoder controls.

### 7.5 ACPI-to-EFI handoff

At U-Boot's final initialization event:

1. the QFW ACPI loader reads `etc/table-loader`;
2. it allocates and relocates the QEMU ACPI table blobs;
3. it applies pointer and checksum commands;
4. it records the relocated RSDP and ACPI memory ranges; and
5. EFI loader registers RSDP under `EFI_ACPI_TABLE_GUID`.

Before launching Linux, the test script checks:

- `qfw list` includes the ACPI files;
- `acpi list` includes CEDT;
- `efidebug tables` exposes the ACPI configuration table; and
- `bootefi` starts the RISC-V EFI kernel.

Failure to load or register ACPI is fatal to the CXL Linux boot script.

## 8. Linux Changes

Linux remains as close to upstream behavior as possible. The required local
change is the ACPI dependency-order fix used by the referenced QEMU series:
after an ACPI PCI root is successfully scanned and its devices are added,
`acpi_pci_root_add()` calls `acpi_dev_clear_dependencies(device)`.

This clears the dependency held by ACPI0017 on each newly active ACPI0016
supplier and prevents the CXL root driver from probing before all CXL host
bridges exist.

The kernel configuration enables, at minimum:

- RISC-V EFI and ACPI boot;
- PCI/PCIe ECAM;
- CXL bus, ACPI, port, memory, and PMEM support;
- device DAX and CXL region support; and
- an initramfs containing `cxl`, `daxctl`, `lspci`, and table-inspection tools.

No private DT CXL root binding will be added as a fallback. If ACPI handoff
fails, the test fails rather than silently using a different topology model.

## 9. Error Handling and Safety

- Extended-capability traversal has a finite hop limit and visited-offset
  bitmap to prevent loops.
- All DVSEC and Register Locator accesses are range-checked before MMIO.
- Register blocks outside the assigned BAR are reported and not mapped.
- QEMU rejects machine resource overlap before vCPU execution.
- FMW configuration is validated for size, alignment, target count, and target
  existence.
- Type 2 starts with `gpu-mode=none`; auto-detection is prohibited in the
  reproducible launch script.
- CXL memory is not included in U-Boot's normal RAM map.
- Linux region creation uses only the RAM-backed emulated Type 3 device.
- A failed Linux region or memory-online operation is reported as a failed
  proof, not a partial success.

## 10. Verification

### 10.1 QEMU

- Build `riscv64-softmmu` with CXL enabled.
- Build/link a non-CXL RISC-V target to guard the QAPI stub/configuration
  boundary.
- Add QTests for:
  - default `sifive_u` behavior;
  - `sifive_u,cxl=on` DT PCI and `fw_cfg` nodes;
  - CXL-off device rejection;
  - address overlap rejection;
  - Type 2 plus Type 3 topology; and
  - ACPI table signatures, ACPI0016/0017 objects, `_DEP`, CEDT CHBS, and CFMWS.

### 10.2 U-Boot

- Add parser unit tests for valid Type 2/Type 3 DVSECs and malformed chains,
  loops, lengths, BAR indices, and offsets.
- Boot the existing non-CXL command and prove its banner/model remain intact.
- Boot the exact CXL machine and prove:
  - `pci enum` succeeds;
  - `pci` shows PXB, two root ports, Type 2, and Type 3;
  - `cxl list` classifies both endpoints;
  - `cxl info` reads valid component and HDM capability state;
  - `acpi list` shows CEDT; and
  - EFI exposes the ACPI table GUID.

### 10.3 Linux

Boot Linux only through U-Boot EFI on the exact machine:

```text
qemu-system-riscv64 -M sifive_u,cxl=on ...
```

Required proof:

- Linux reports EFI and ACPI initialization;
- ACPI lists ACPI0017 and all expected ACPI0016 roots;
- `lspci -nn` shows Type 2 and Type 3 endpoints;
- `cxl list -M -m -D -d -R -r` shows the root decoder, ports, and Type 3 memdev;
- kernel logs have no unresolved `_DEP`, CEDT target, or CXL probe errors;
- a Type 3 region can be created and enabled;
- the resulting DAX device can be inspected and, if configured for
  system-ram, onlined and exercised; and
- a memory pattern written through the selected Type 3 mapping reads back
  correctly.

The Type 2 proof boundary is enumeration, DVSEC/register discovery, driver
binding where supported by the local model, and access to its simulated device
memory. It does not claim GPU execution or physical cache coherency.

### 10.4 Reproducible artifacts

Provide scripts for build and launch rather than relying on shell history.
Capture:

- exact source revisions and dirty-state summaries;
- compiler and tool versions;
- complete QEMU command line;
- SHA-256 hashes of QEMU, U-Boot, OpenSBI, kernel, DTB, and initramfs;
- U-Boot serial log;
- Linux serial log;
- decoded DT and ACPI tables; and
- machine-readable PCI/CXL topology output.

## 11. Completion Criteria

The task is complete only when all of the following hold:

1. `qemu-system-riscv64 -M sifive_u` still boots the existing U-Boot image.
2. `qemu-system-riscv64 -M sifive_u,cxl=on` boots the CXL-enabled U-Boot image.
3. U-Boot enumerates and correctly identifies both Type 2 and Type 3.
4. U-Boot loads QEMU ACPI tables and exposes them through EFI.
5. That same U-Boot instance boots the RISC-V Linux EFI image.
6. Linux enumerates the ACPI CXL host topology and both endpoints.
7. Linux successfully creates and validates a Type 3 CXL region.
8. Negative and malformed-input tests pass.
9. The exact launch and evidence are preserved in reproducible artifacts.

Build success, U-Boot PCI output alone, direct QEMU `-kernel` Linux boot, or a
separate EDK2 boot does not satisfy this specification.
