# SiFive U CXL U-Boot-to-Linux Integration Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build all three trees and prove the exact `qemu-system-riscv64 -M sifive_u` U-Boot-to-Linux chain with Type 2 and Type 3 CXL devices.

**Architecture:** A FAT EFI system partition carries the RISC-V EFI-stubbed kernel and initramfs. QEMU boots OpenSBI then U-Boot with the generated DT; U-Boot inspects CXL, loads QEMU ACPI through QFW, creates an EFI boot option with LoadFile2 initrd, and starts Linux, whose guest service produces a fail-closed CXL region proof.

**Tech Stack:** Bash build scripts, GPT/FAT image tools, QEMU RISC-V, OpenSBI, U-Boot EFI boot manager, pexpect, JSON evidence manifest, SHA-256.

---

## File map

- Create `qemu-cxl-type2/scripts/sifive_u_cxl/make-esp.sh`: create a deterministic GPT/FAT boot disk containing `Image` and `rootfs.cpio.gz`.
- Create `qemu-cxl-type2/scripts/sifive_u_cxl/build-all.sh`: build QEMU, U-Boot, Linux/rootfs, and the EFI disk.
- Create `qemu-cxl-type2/scripts/sifive_u_cxl/run.py`: launch the exact machine, drive U-Boot, capture serial, and wait for guest proof.
- Create `qemu-cxl-type2/scripts/sifive_u_cxl/verify.py`: parse logs and write bounded machine-readable evidence.
- Create `qemu-cxl-type2/scripts/sifive_u_cxl/run-regression.sh`: prove default `sifive_u` remains unchanged.
- Create `qemu-cxl-type2/scripts/sifive_u_cxl/README.md`: one-command reproduction and proof interpretation.
- Create `qemu-cxl-type2/scripts/sifive_u_cxl/.gitignore`: exclude generated images, logs, and manifests.

### Task 1: Create the EFI system partition

**Files:**
- Create: `qemu-cxl-type2/scripts/sifive_u_cxl/make-esp.sh`
- Create: `qemu-cxl-type2/scripts/sifive_u_cxl/.gitignore`

- [ ] **Step 1: Write strict input validation**

Use:

```bash
#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 3 ]]; then
    echo "usage: $0 <Image> <rootfs.cpio.gz> <output.img>" >&2
    exit 2
fi

kernel=$(realpath "$1")
initrd=$(realpath "$2")
output=$(realpath -m "$3")
[[ -s "$kernel" ]] || { echo "missing kernel: $kernel" >&2; exit 1; }
[[ -s "$initrd" ]] || { echo "missing initrd: $initrd" >&2; exit 1; }
```

- [ ] **Step 2: Build a 512 MiB GPT/FAT disk**

Continue with:

```bash
truncate -s 512M "$output"
sgdisk --clear \
       --new=1:2048:0 \
       --typecode=1:ef00 \
       --change-name=1:CXL-ESP "$output"
mformat -i "$output@@1048576" -F -v CXLBOOT ::
mcopy -i "$output@@1048576" "$kernel" ::/Image
mcopy -i "$output@@1048576" "$initrd" ::/rootfs.cpio.gz
mdir -i "$output@@1048576" ::
sha256sum "$output"
```

Expected: partition 1 begins at 1 MiB and both files are listed.

- [ ] **Step 3: Exclude generated state**

Create:

```text
out/
```

- [ ] **Step 4: Run and inspect**

```bash
cd /root/cxl-u-boot/qemu-cxl-type2
scripts/sifive_u_cxl/make-esp.sh \
  ../linux-cxl-type2/arch/riscv/boot/Image \
  /tmp/sifive-u-cxl-br/images/rootfs.cpio.gz \
  scripts/sifive_u_cxl/out/cxl-esp.img
mdir -i scripts/sifive_u_cxl/out/cxl-esp.img@@1048576 ::
```

Expected: `Image` and `rootfs.cpio.gz`.

- [ ] **Step 5: Commit**

```bash
git add scripts/sifive_u_cxl/make-esp.sh scripts/sifive_u_cxl/.gitignore
git commit -m "scripts: create SiFive U CXL EFI disk"
```

### Task 2: Add the complete build driver

**Files:**
- Create: `qemu-cxl-type2/scripts/sifive_u_cxl/build-all.sh`

- [ ] **Step 1: Define fixed paths and fail-closed prerequisites**

Use:

```bash
#!/usr/bin/env bash
set -euo pipefail

qemu_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
workspace=$(cd -- "$qemu_dir/.." && pwd)
uboot_dir="$workspace/u-boot"
linux_dir="$workspace/linux-cxl-type2"
out="$qemu_dir/scripts/sifive_u_cxl/out"
mkdir -p "$out"

for tool in ninja make git dtc iasl mformat mcopy sgdisk sha256sum; do
    command -v "$tool" >/dev/null ||
        { echo "missing tool: $tool" >&2; exit 1; }
done
```

- [ ] **Step 2: Build QEMU**

Add:

```bash
if [[ ! -f "$qemu_dir/build-riscv-cxl/build.ninja" ]]; then
    mkdir -p "$qemu_dir/build-riscv-cxl"
    (
        cd "$qemu_dir/build-riscv-cxl"
        ../configure --target-list=riscv64-softmmu \
            --disable-docs --disable-werror
    )
fi
ninja -C "$qemu_dir/build-riscv-cxl" qemu-system-riscv64
```

- [ ] **Step 3: Build U-Boot with the external DTC workaround**

Add:

```bash
make -C "$uboot_dir" O=build_sifive_u_cxl \
    sifive_unleashed_qemu_cxl_defconfig
make -C "$uboot_dir" O=build_sifive_u_cxl \
    DTC=/usr/bin/dtc PYTHON3=/usr/bin/python3 -j"$(nproc)"
```

- [ ] **Step 4: Build Linux and rootfs**

Add:

```bash
"$linux_dir/tools/testing/cxl/sifive_u/build-guest.sh"
"$qemu_dir/scripts/sifive_u_cxl/make-esp.sh" \
    "$linux_dir/arch/riscv/boot/Image" \
    /tmp/sifive-u-cxl-br/images/rootfs.cpio.gz \
    "$out/cxl-esp.img"
```

- [ ] **Step 5: Record inputs before runtime**

Add:

```bash
{
    git -C "$qemu_dir" rev-parse HEAD
    git -C "$uboot_dir" rev-parse HEAD
    git -C "$linux_dir" rev-parse HEAD
} > "$out/source-revisions.txt"

sha256sum \
    "$qemu_dir/build-riscv-cxl/qemu-system-riscv64" \
    "$uboot_dir/build_sifive_u_cxl/u-boot.bin" \
    "$qemu_dir/pc-bios/opensbi-riscv64-generic-fw_dynamic.bin" \
    "$linux_dir/arch/riscv/boot/Image" \
    /tmp/sifive-u-cxl-br/images/rootfs.cpio.gz \
    "$out/cxl-esp.img" > "$out/SHA256SUMS"
```

- [ ] **Step 6: Run the build driver**

Expected: all six artifacts exist and `SHA256SUMS` has six lines.

- [ ] **Step 7: Commit**

```bash
git add scripts/sifive_u_cxl/build-all.sh
git commit -m "scripts: build SiFive U CXL boot stack"
```

### Task 3: Automate U-Boot EFI launch on the exact machine

**Files:**
- Create: `qemu-cxl-type2/scripts/sifive_u_cxl/run.py`

- [ ] **Step 1: Construct the immutable QEMU argument list**

Use a Python list whose first machine arguments are exactly:

```python
cmd = [
    str(qemu),
    '-M', 'sifive_u,cxl=on',
    '-M', 'cxl-fmw.0.targets.0=cxl.1,cxl-fmw.0.size=4G',
    '-smp', '5',
    '-m', '2G',
    '-nographic',
    '-no-reboot',
    '-bios', str(opensbi),
    '-kernel', str(uboot),
    '-drive', f'if=none,file={esp},format=raw,id=esp',
    '-device', 'virtio-blk-pci,drive=esp,bus=pcie.0,id=bootdisk',
    '-object', 'memory-backend-ram,id=t3mem,size=256M,share=on',
    '-object', 'memory-backend-ram,id=t3lsa,size=2M,share=on',
    '-device', 'pxb-cxl,bus=pcie.0,bus_nr=64,id=cxl.1',
    '-device', 'cxl-rp,bus=cxl.1,port=0,id=rp-t2,chassis=0,slot=0',
    '-device', 'cxl-type2,bus=rp-t2,gpu-mode=0,mem-size=256M,cache-size=64M,id=t2',
    '-device', 'cxl-rp,bus=cxl.1,port=1,id=rp-t3,chassis=0,slot=1',
    '-device', 'cxl-type3,bus=rp-t3,volatile-memdev=t3mem,lsa=t3lsa,id=t3',
]
```

Serialize this list to `out/qemu-command.json` before launching.

- [ ] **Step 2: Capture the full serial stream**

Use `pexpect.spawn(cmd[0], cmd[1:], encoding='utf-8', timeout=180)` and set:

```python
child.logfile_read = open(serial_log, 'w', encoding='utf-8')
child.expect_exact('U-Boot ')
child.expect_exact('=> ')
```

Fail with the last 80 serial lines on timeout or EOF.

- [ ] **Step 3: Run the U-Boot proof commands**

Send each command and require the prompt:

```text
version
pci enum
pci
cxl list
cxl info 41.00.0
cxl info 42.00.0
qfw list
acpi list
efidebug tables
virtio scan
fatls virtio 0:1 /
```

Require `Type 2`, `Type 3`, `CEDT`, the ACPI table GUID, `Image`, and
`rootfs.cpio.gz` in the accumulated output.

- [ ] **Step 4: Create and run the EFI boot option**

Send:

```text
efidebug boot add -b 0001 CXL-Linux virtio 0:1 /Image -i virtio 0:1 /rootfs.cpio.gz -s 'console=hvc0 earlycon=sbi loglevel=7 rdinit=/sbin/init'
efidebug boot order 0001
bootefi bootmgr
```

Expected Linux milestones:

```text
EFI v
ACPI:
CXL_PROOF_BEGIN
CXL_PATTERN_PASS bytes=2097152
CXL_PROOF_PASS
```

- [ ] **Step 5: Require clean guest shutdown**

Wait for QEMU EOF after `CXL_PROOF_PASS` and require exit status 0. A timeout,
EOF before the pass marker, or any `CXL_PROOF_FAIL` marker is failure.

- [ ] **Step 6: Run the automation**

```bash
cd /root/cxl-u-boot/qemu-cxl-type2
python3 scripts/sifive_u_cxl/run.py
```

Expected: clean exit and `out/serial-cxl.log`.

- [ ] **Step 7: Commit**

```bash
git add scripts/sifive_u_cxl/run.py
git commit -m "scripts: boot SiFive U CXL through U-Boot EFI"
```

### Task 4: Preserve the default `sifive_u` regression

**Files:**
- Create: `qemu-cxl-type2/scripts/sifive_u_cxl/run-regression.sh`

- [ ] **Step 1: Boot the unchanged default command**

Use:

```bash
#!/usr/bin/env bash
set -euo pipefail
qemu_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
workspace=$(cd -- "$qemu_dir/.." && pwd)
out="$qemu_dir/scripts/sifive_u_cxl/out"

timeout 30s "$qemu_dir/build-riscv-cxl/qemu-system-riscv64" \
  -M sifive_u -smp 5 -m 2G -display none -serial stdio \
  -kernel "$workspace/u-boot/build_sifive_u_extdtc/u-boot.bin" \
  > "$out/serial-default.log" 2>&1 || test $? -eq 124

grep -q 'SiFive HiFive Unleashed A00' "$out/serial-default.log"
grep -q 'U-Boot 2024.07' "$out/serial-default.log"
! grep -q 'pci@30000000' "$out/serial-default.log"
```

- [ ] **Step 2: Prove CXL devices are rejected with CXL off**

Run:

```bash
if "$qemu_dir/build-riscv-cxl/qemu-system-riscv64" \
    -M sifive_u -display none \
    -device pxb-cxl,bus=pcie.0,id=cxl.1 \
    > "$out/cxl-off-rejection.log" 2>&1; then
    echo "CXL-off launch unexpectedly succeeded" >&2
    exit 1
fi
grep -Eq 'Bus .pcie.0. not found|CXL.*off' "$out/cxl-off-rejection.log"
```

- [ ] **Step 3: Run and commit**

```bash
scripts/sifive_u_cxl/run-regression.sh
git add scripts/sifive_u_cxl/run-regression.sh
git commit -m "scripts: guard default SiFive U behavior"
```

### Task 5: Produce machine-readable evidence

**Files:**
- Create: `qemu-cxl-type2/scripts/sifive_u_cxl/verify.py`

- [ ] **Step 1: Parse required proof markers**

Read `serial-cxl.log` and fail unless all are present:

```python
required = {
    'opensbi': 'OpenSBI v',
    'uboot': 'U-Boot 2024.07',
    'type2': 'Type 2',
    'type3': 'Type 3',
    'cedt': 'CEDT',
    'efi': 'EFI v',
    'pattern': 'CXL_PATTERN_PASS bytes=2097152',
    'region': 'CXL_PROOF_PASS',
}
```

Fail if the log contains `CXL_PROOF_FAIL`, unresolved `_DEP`, or a CXL probe
error.

- [ ] **Step 2: Write bounded JSON**

Write `out/evidence.json`:

```json
{
  "machine": "sifive_u",
  "cxl_enabled": true,
  "boot_chain": ["OpenSBI", "U-Boot EFI", "Linux ACPI"],
  "endpoints": [
    {"bdf": "41:00.0", "type": 2},
    {"bdf": "42:00.0", "type": 3}
  ],
  "type3_region": {"size_bytes": 268435456, "pattern_bytes": 2097152},
  "proof": "pass"
}
```

Populate hashes and source revisions from the generated files rather than
hard-coding them.

- [ ] **Step 3: Run the full evidence gate**

```bash
scripts/sifive_u_cxl/build-all.sh
scripts/sifive_u_cxl/run-regression.sh
python3 scripts/sifive_u_cxl/run.py
python3 scripts/sifive_u_cxl/verify.py
python3 -m json.tool scripts/sifive_u_cxl/out/evidence.json
sha256sum -c scripts/sifive_u_cxl/out/SHA256SUMS
```

Expected: every command exits 0 and JSON reports `"proof": "pass"`.

- [ ] **Step 4: Commit**

```bash
git add scripts/sifive_u_cxl/verify.py
git commit -m "scripts: validate SiFive U CXL boot evidence"
```

### Task 6: Document the one-command reproduction

**Files:**
- Create: `qemu-cxl-type2/scripts/sifive_u_cxl/README.md`

- [ ] **Step 1: Document prerequisites and commands**

Document:

```bash
scripts/sifive_u_cxl/build-all.sh
scripts/sifive_u_cxl/run-regression.sh
python3 scripts/sifive_u_cxl/run.py
python3 scripts/sifive_u_cxl/verify.py
```

State explicitly that the bridge is synthetic, `cxl=off` is unchanged, Type 2
does not prove GPU execution, and completion requires Linux Type 3 region plus
memory proof.

- [ ] **Step 2: Link the design and three implementation plans**

Link:

```text
docs/superpowers/specs/2026-07-23-sifive-u-cxl-uboot-linux-design.md
docs/superpowers/plans/2026-07-23-sifive-u-cxl-qemu.md
docs/superpowers/plans/2026-07-23-sifive-u-cxl-uboot.md
docs/superpowers/plans/2026-07-23-sifive-u-cxl-linux.md
```

- [ ] **Step 3: Commit**

```bash
git add scripts/sifive_u_cxl/README.md
git commit -m "docs: add SiFive U CXL reproduction guide"
```
