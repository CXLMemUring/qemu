#!/usr/bin/env python3
#
# Functional test that boots a Linux kernel on a Sifive U machine
# and checks the console
#
# Copyright (c) Linaro Ltd.
#
# Author:
#  Philippe Mathieu-Daudé
#
# SPDX-License-Identifier: GPL-2.0-or-later

import os
import subprocess
import time

from qemu_test import Asset, LinuxKernelTest
from qemu_test import skipIfMissingCommands


class SifiveU(LinuxKernelTest):

    ASSET_KERNEL = Asset(
        'https://storage.tuxboot.com/buildroot/20241119/riscv64/Image',
        '2bd8132a3bf21570290042324fff48c987f42f2a00c08de979f43f0662ebadba')
    ASSET_ROOTFS = Asset(
        ('https://github.com/groeck/linux-build-test/raw/'
         '9819da19e6eef291686fdd7b029ea00e764dc62f/rootfs/riscv64/'
         'rootfs.ext2.gz'),
        'b6ed95610310b7956f9bf20c4c9c0c05fea647900df441da9dfe767d24e8b28b')

    def test_sifive_u_cxl_property(self):
        self.set_machine('sifive_u')
        self.vm.add_args('-machine', 'cxl=on', '-display', 'none', '-S')
        self.vm.set_qmp_monitor(enabled=False)
        self.vm.launch()
        time.sleep(0.1)
        self.assertTrue(self.vm.is_running())

    def test_sifive_u_cxl_rejects_ram_overlap(self):
        self.set_machine('sifive_u')
        self.vm.add_args('-machine', 'cxl=on', '-m', '16G',
                         '-display', 'none')
        self.vm.set_qmp_monitor(enabled=False)
        self.vm.launch()
        self.vm.wait(timeout=5)
        self.assertEqual(self.vm.exitcode(), 1)
        self.assertIn(
            'sifive_u CXL: RAM overlaps synthetic PCI MMIO64',
            self.vm.get_log())

    def _dump_dts(self, cxl):
        dtb = self.scratch_file('sifive-u.dtb')

        self.set_machine('sifive_u')
        machine = f'dumpdtb={dtb}'
        if cxl:
            machine += ',cxl=on'
        self.vm.add_args('-machine', machine, '-display', 'none')
        self.vm.set_qmp_monitor(enabled=False)
        self.vm.launch()
        self.vm.wait(timeout=5)
        self.assertEqual(self.vm.exitcode(), 0)
        return subprocess.check_output(
            ['dtc', '-I', 'dtb', '-O', 'dts', dtb],
            text=True, stderr=subprocess.DEVNULL)

    def test_sifive_u_default_dtb_has_no_synthetic_pcie(self):
        dts = self._dump_dts(False)

        self.assertNotIn('pci-host-ecam-generic', dts)
        self.assertNotIn('qemu,fw-cfg-mmio', dts)
        self.assertNotIn('qemu,synthetic-cxl-host', dts)

    def test_sifive_u_cxl_dtb_has_synthetic_pcie(self):
        dts = self._dump_dts(True)

        self.assertIn('compatible = "pci-host-ecam-generic"', dts)
        self.assertIn('reg = <0x00 0x30000000 0x00 0x10000000>', dts)
        self.assertIn('bus-range = <0x00 0xff>', dts)
        self.assertIn('qemu,fw-cfg-mmio', dts)
        self.assertIn('qemu,synthetic-cxl-host', dts)

    def do_test_riscv64_sifive_u_mmc_spi(self, connect_card):
        self.set_machine('sifive_u')
        kernel_path = self.ASSET_KERNEL.fetch()
        rootfs_path = self.uncompress(self.ASSET_ROOTFS)

        self.vm.set_console()
        kernel_command_line = (self.KERNEL_COMMON_COMMAND_LINE +
                               'earlycon=sbi console=ttySIF0 '
                               'root=/dev/mmcblk0 ')
        self.vm.add_args('-kernel', kernel_path,
                         '-append', kernel_command_line,
                         '-no-reboot')
        if connect_card:
            kernel_command_line += 'panic=-1 noreboot rootwait '
            self.vm.add_args('-drive', f'file={rootfs_path},if=sd,format=raw')
            pattern = 'Boot successful.'
        else:
            kernel_command_line += 'panic=0 noreboot '
            pattern = 'Cannot open root device "mmcblk0" or unknown-block(0,0)'

        self.vm.launch()
        self.wait_for_console_pattern(pattern)

        os.remove(rootfs_path)

    def test_riscv64_sifive_u_nommc_spi(self):
        self.do_test_riscv64_sifive_u_mmc_spi(False)

    def test_riscv64_sifive_u_mmc_spi(self):
        self.do_test_riscv64_sifive_u_mmc_spi(True)


if __name__ == '__main__':
    LinuxKernelTest.main()
