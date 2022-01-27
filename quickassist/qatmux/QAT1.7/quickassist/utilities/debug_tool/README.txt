/******************************************************************************
 *
 *   BSD LICENSE
 * 
 *   Copyright(c) 2007-2018 Intel Corporation. All rights reserved.
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 *  version: QAT1.7.L.4.6.0-00025
 *
 *****************************************************************************/

===============================================================================
The aim of this script is to gather data that should help to debug an issue.
===============================================================================

Running the script
===============================================================================

1) Install the Intel® QuickAssist Technology Software for Hardware Version 1.7
   package at <INSTALL DIR>
2) Make sure that the driver is up
     a) #export ICP_ROOT=<INSTALL DIR>
     b) #cd $ICP_ROOT/build
     c) #./adf_ctl status
        root@system:/QAT/build# ./adf_ctl status
        Checking status of all devices.
        There is 4 QAT acceleration device(s) in the system:
         qat_dev0 - type: c6xx,  inst_id: 0,  node_id: 0,  bsf: 03:00.0,  #accel: 5 #engines: 10 state: up
         qat_dev1 - type: c6xx,  inst_id: 1,  node_id: 0,  bsf: 05:00.0,  #accel: 5 #engines: 10 state: up
         qat_dev2 - type: c6xx,  inst_id: 2,  node_id: 0,  bsf: 07:00.0,  #accel: 5 #engines: 10 state: up
         qat_dev3 - type: d15xx, inst_id: 0,  node_id: 0,  bsf: 0b:00.0,  #accel: 5 #engines: 10 state: up
        root@system:/QAT/build
    e) Ensure all the devices are up
3) Go to the directory containing the debug dump script
      #cd $ICP_ROOT/quickassist/utilities/debug_tool
4) Run the test case triggers the issue until the issue is recreated
5) Execute the icp_dump script
     #./icp_dump.sh <full path debug file>

     The resulting debug tar ball will be created and stored in location passed in above.

6) Provide the created debug tarball to your Intel® representative

The following is a list of all the details that will be gathered by the script
and stored in the tar ball:
* output of lspci -vvv
* output of lstopo-no-graphics -v
* /sys/kernel/debug/qat_*/* directories
* output of adf_ctl status
* config files /etc/*_dev*.conf
* firmware md5sum
* environment variables
* output of uname -a
* output of /var/log/messages
* kernel config files
* device sku
* BIOS version
* amount of Memory on the system
* /proc/cmdline
* kernel modules loaded
* version of the qat driver installed

Output on a system with c6xx and dh895xcc devices has the following tree
structure
=====================================================================
ICP_debug
├── adf_ctl_status.txt
├── BIOS.txt
├── cmdline.txt
├── config_files
│   ├── c6xx_dev0.conf
│   ├── c6xx_dev1.conf
│   ├── c6xx_dev2.conf
│   └── dh895xcc_dev0.conf
├── environment.txt
├── firmware_files
│   ├── qat_895xcc_md5sum.txt
│   ├── qat_895xcc_mmp_md5sum.txt
│   ├── qat_c62x_md5sum.txt
│   └── qat_c62x_mmp_md5sum.txt
├── hwloc.txt
├── Kernel_config_files
│   ├── config-3.10.0-514.el7.x86_64
│   ├── config-3.10.0-693.11.6.el7.x86_64
│   └── config-3.10.0-693.17.1.el7.x86_64
├── lsmod.txt
├── lspci.txt
├── memory.txt
├── messages.txt
├── qat_c6xx_85:00.0
│   ├── cnv_errors
│   ├── dev_cfg
│   ├── frequency
│   ├── fw_counters
│   ├── heartbeat
│   ├── heartbeat_failed
│   ├── heartbeat_sent
│   ├── transport
│   │   ├── bank_00
│   │   │   ├── config
│   │   │   ├── ring_00
│   │   │   ├── ring_02
│   │   │   ├── ring_08
│   │   │   └── ring_10
│   │   ├── bank_01
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   ├── ring_06
│   │   │   ├── ring_10
│   │   │   └── ring_14
│   │   ├── bank_02
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   ├── ring_06
│   │   │   ├── ring_10
│   │   │   └── ring_14
│   │   ├── bank_03
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   └── ring_10
│   │   ├── bank_04
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   └── ring_10
│   │   ├── bank_05
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   └── ring_10
│   │   ├── bank_06
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   └── ring_10
│   │   ├── bank_07
│   │   │   └── config
│   │   ├── bank_08
│   │   │   └── config
│   │   ├── bank_09
│   │   │   └── config
│   │   ├── bank_10
│   │   │   └── config
│   │   ├── bank_11
│   │   │   └── config
│   │   ├── bank_12
│   │   │   └── config
│   │   ├── bank_13
│   │   │   └── config
│   │   ├── bank_14
│   │   │   └── config
│   │   └── bank_15
│   │       └── config
│   └── version
│       ├── fw
│       ├── hw
│       └── mmp
├── qat_c6xx_87:00.0
│   ├── cnv_errors
│   ├── dev_cfg
│   ├── frequency
│   ├── fw_counters
│   ├── heartbeat
│   ├── heartbeat_failed
│   ├── heartbeat_sent
│   ├── transport
│   │   ├── bank_00
│   │   │   ├── config
│   │   │   ├── ring_00
│   │   │   ├── ring_02
│   │   │   ├── ring_08
│   │   │   └── ring_10
│   │   ├── bank_01
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   ├── ring_06
│   │   │   ├── ring_10
│   │   │   └── ring_14
│   │   ├── bank_02
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   ├── ring_06
│   │   │   ├── ring_10
│   │   │   └── ring_14
│   │   ├── bank_03
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   └── ring_10
│   │   ├── bank_04
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   └── ring_10
│   │   ├── bank_05
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   └── ring_10
│   │   ├── bank_06
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   └── ring_10
│   │   ├── bank_07
│   │   │   └── config
│   │   ├── bank_08
│   │   │   └── config
│   │   ├── bank_09
│   │   │   └── config
│   │   ├── bank_10
│   │   │   └── config
│   │   ├── bank_11
│   │   │   └── config
│   │   ├── bank_12
│   │   │   └── config
│   │   ├── bank_13
│   │   │   └── config
│   │   ├── bank_14
│   │   │   └── config
│   │   └── bank_15
│   │       └── config
│   └── version
│       ├── fw
│       ├── hw
│       └── mmp
├── qat_c6xx_89:00.0
│   ├── cnv_errors
│   ├── dev_cfg
│   ├── frequency
│   ├── fw_counters
│   ├── heartbeat
│   ├── heartbeat_failed
│   ├── heartbeat_sent
│   ├── transport
│   │   ├── bank_00
│   │   │   ├── config
│   │   │   ├── ring_00
│   │   │   ├── ring_02
│   │   │   ├── ring_08
│   │   │   └── ring_10
│   │   ├── bank_01
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   ├── ring_06
│   │   │   ├── ring_10
│   │   │   └── ring_14
│   │   ├── bank_02
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   ├── ring_06
│   │   │   ├── ring_10
│   │   │   └── ring_14
│   │   ├── bank_03
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   └── ring_10
│   │   ├── bank_04
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   └── ring_10
│   │   ├── bank_05
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   └── ring_10
│   │   ├── bank_06
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   └── ring_10
│   │   ├── bank_07
│   │   │   └── config
│   │   ├── bank_08
│   │   │   └── config
│   │   ├── bank_09
│   │   │   └── config
│   │   ├── bank_10
│   │   │   └── config
│   │   ├── bank_11
│   │   │   └── config
│   │   ├── bank_12
│   │   │   └── config
│   │   ├── bank_13
│   │   │   └── config
│   │   ├── bank_14
│   │   │   └── config
│   │   └── bank_15
│   │       └── config
│   └── version
│       ├── fw
│       ├── hw
│       └── mmp
├── qat_dh895xcc_02:00.0
│   ├── cnv_errors
│   ├── dev_cfg
│   ├── fw_counters
│   ├── heartbeat
│   ├── heartbeat_failed
│   ├── heartbeat_sent
│   ├── transport
│   │   ├── bank_00
│   │   │   ├── config
│   │   │   ├── ring_00
│   │   │   ├── ring_02
│   │   │   ├── ring_08
│   │   │   └── ring_10
│   │   ├── bank_01
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   ├── ring_06
│   │   │   ├── ring_10
│   │   │   └── ring_14
│   │   ├── bank_02
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   ├── ring_06
│   │   │   ├── ring_10
│   │   │   └── ring_14
│   │   ├── bank_03
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   └── ring_10
│   │   ├── bank_04
│   │   │   ├── config
│   │   │   ├── ring_02
│   │   │   └── ring_10
│   │   ├── bank_05
│   │   │   └── config
│   │   ├── bank_06
│   │   │   └── config
│   │   ├── bank_07
│   │   │   └── config
│   │   ├── bank_08
│   │   │   └── config
│   │   ├── bank_09
│   │   │   └── config
│   │   ├── bank_10
│   │   │   └── config
│   │   ├── bank_11
│   │   │   └── config
│   │   ├── bank_12
│   │   │   └── config
│   │   ├── bank_13
│   │   │   └── config
│   │   ├── bank_14
│   │   │   └── config
│   │   ├── bank_15
│   │   │   └── config
│   │   ├── bank_16
│   │   │   └── config
│   │   ├── bank_17
│   │   │   └── config
│   │   ├── bank_18
│   │   │   └── config
│   │   ├── bank_19
│   │   │   └── config
│   │   ├── bank_20
│   │   │   └── config
│   │   ├── bank_21
│   │   │   └── config
│   │   ├── bank_22
│   │   │   └── config
│   │   ├── bank_23
│   │   │   └── config
│   │   ├── bank_24
│   │   │   └── config
│   │   ├── bank_25
│   │   │   └── config
│   │   ├── bank_26
│   │   │   └── config
│   │   ├── bank_27
│   │   │   └── config
│   │   ├── bank_28
│   │   │   └── config
│   │   ├── bank_29
│   │   │   └── config
│   │   ├── bank_30
│   │   │   └── config
│   │   └── bank_31
│   │       └── config
│   └── version
│       ├── fw
│       ├── hw
│       └── mmp
├── Sku_details.txt
├── swVersion.txt
└── uname.txt

95 directories, 218 files

Legal/Disclaimers
===================

INFORMATION IN THIS DOCUMENT IS PROVIDED IN CONNECTION WITH INTEL(R) PRODUCTS.
NO LICENSE, EXPRESS OR IMPLIED, BY ESTOPPEL OR OTHERWISE, TO ANY INTELLECTUAL
PROPERTY RIGHTS IS GRANTED BY THIS DOCUMENT. EXCEPT AS PROVIDED IN INTEL'S
TERMS AND CONDITIONS OF SALE FOR SUCH PRODUCTS, INTEL ASSUMES NO LIABILITY
WHATSOEVER, AND INTEL DISCLAIMS ANY EXPRESS OR IMPLIED WARRANTY, RELATING TO
SALE AND/OR USE OF INTEL PRODUCTS INCLUDING LIABILITY OR WARRANTIES RELATING
TO FITNESS FOR A PARTICULAR PURPOSE, MERCHANTABILITY, OR INFRINGEMENT OF ANY
PATENT, COPYRIGHT OR OTHER INTELLECTUAL PROPERTY RIGHT. Intel products are
not intended for use in medical, life saving, life sustaining, critical control
 or safety systems, or in nuclear facility applications.

Intel may make changes to specifications and product descriptions at any time,
without notice.

(C) Intel Corporation 2018

* Other names and brands may be claimed as the property of others.

===============================================================================

