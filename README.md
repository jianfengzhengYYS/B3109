# B3109
project for B3109 chip
This directory is special for b3109 project. b3109 is a rf chip made by beijing boruiwei technology.

OS Requirement:
   Ubuntu 16.04

Tools:
   Vivado 2018.3
   Petalinux 2018.3

This directory includes 3 parts code:
1) hdl: FPGA code for Xilinx demo board.
2) linux: Linux code for Zynq/ZynqMPSoC chips.
3) meta-aid
4) Petalinux Project

Notes:
1) Create petalinux project: remember setting external linux source code directory. 
2) meta-adi code (version: 2019-R1) bugs: 
     * device-tree.bbappend file: change "echo -e" as "echo "
     * jesd-status_dev.bb file: change SRC_URI = "git://github.com/analogdevicesinc/jesd-eye-scan-gtk.git;branch=${BRANCH}" as SRC_URI = "git://github.com/analogdevicesinc/jesd-eye-scan-gtk.git;protocol=https;branch=${BRANCH}"
     * libiio.inc file: change SRC_URI = "git://github.com/analogdevicesinc/libiio.git;branch=${BRANCH}" as SRC_URI = "git://github.com/analogdevicesinc/libiio.git;protocol=https;branch=${BRANCH}"
