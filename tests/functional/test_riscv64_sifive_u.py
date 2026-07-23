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
        self.assertIn('0x43000000', dts)

    def _add_dual_cxl_topology(self):
        self.vm.add_args(
            '-machine', 'cxl=on',
            '-machine',
            'cxl-fmw.0.targets.0=cxl.1,cxl-fmw.0.size=4G',
            '-object',
            'memory-backend-ram,id=t3mem,size=256M,share=on',
            '-object',
            'memory-backend-ram,id=t3lsa,size=2M,share=on',
            '-device', 'pxb-cxl,bus=pcie.0,bus_nr=64,id=cxl.1',
            '-device',
            'cxl-rp,bus=cxl.1,port=0,id=rp-t2,chassis=0,slot=0',
            '-device',
            ('cxl-type2,bus=rp-t2,gpu-mode=0,mem-size=256M,'
             'cache-size=64M,id=t2'),
            '-device',
            'cxl-rp,bus=cxl.1,port=1,id=rp-t3,chassis=0,slot=1',
            '-device',
            ('cxl-type3,bus=rp-t3,volatile-memdev=t3mem,'
             'lsa=t3lsa,id=t3'))

    def test_sifive_u_cxl_dual_topology_starts(self):
        self.set_machine('sifive_u')
        self._add_dual_cxl_topology()
        self.vm.add_args('-display', 'none', '-S')
        self.vm.set_qmp_monitor(enabled=False)
        self.vm.launch()
        time.sleep(0.5)
        self.assertTrue(self.vm.is_running(), self.vm.get_log())

    def test_sifive_u_cxl_rejects_oversized_fmw(self):
        self.set_machine('sifive_u')
        self.vm.add_args(
            '-machine', 'cxl=on',
            '-machine',
            'cxl-fmw.0.targets.0=cxl.1,cxl-fmw.0.size=8G',
            '-device', 'pxb-cxl,bus=pcie.0,bus_nr=64,id=cxl.1',
            '-display', 'none')
        self.vm.set_qmp_monitor(enabled=False)
        self.vm.launch()
        self.vm.wait(timeout=5)
        self.assertEqual(self.vm.exitcode(), 1)
        self.assertIn(
            'sifive_u CXL: fixed windows exceed 4 GiB aperture',
            self.vm.get_log())

    def test_sifive_u_cxl_off_rejects_pxb(self):
        self.set_machine('sifive_u')
        self.vm.add_args(
            '-device', 'pxb-cxl,bus=pcie.0',
            '-display', 'none')
        self.vm.set_qmp_monitor(enabled=False)
        self.vm.launch()
        self.vm.wait(timeout=5)
        self.assertNotEqual(self.vm.exitcode(), 0)
        self.assertIn("Bus 'pcie.0' not found", self.vm.get_log())

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
