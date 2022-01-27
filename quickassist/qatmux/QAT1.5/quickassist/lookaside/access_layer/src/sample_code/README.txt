/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or 
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
 * 
 *   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
 * 
 *   This program is free software; you can redistribute it and/or modify 
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 * 
 *   This program is distributed in the hope that it will be useful, but 
 *   WITHOUT ANY WARRANTY; without even the implied warranty of 
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 *   General Public License for more details.
 * 
 *   You should have received a copy of the GNU General Public License 
 *   along with this program; if not, write to the Free Software 
 *   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *   The full GNU General Public License is included in this distribution 
 *   in the file called LICENSE.GPL.
 * 
 *   Contact Information:
 *   Intel Corporation
 * 
 *   BSD LICENSE 
 * 
 *   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
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
 * 
 *  version: QAT1.5.L.1.11.0-36
 *
 *****************************************************************************/

===============================================================================

Intel® DH895xCC Sample Code for Security Applications on Intel®
QuickAssist® Technology

Intel® DH89xxCC Sample Code for Security Applications on Intel®
QuickAssist® Technology

Intel® C2xxx Sample Code for Security Applications on Intel®
QuickAssist® Technology

April, 2015

===============================================================================

Reference
=========


 - Intel® Communications Chipset 89xx Series Software for Linux*
   Getting Started Guide

 - Intel® Atom¿ Processor C2000 Product Family for Communications Infrastructure Software for Linux*
   Getting Started Guide

===============================================================================
     

Installing and Running the Security Sample Performance Tests

===============================================================================

1) General 
This is how to generate and run sample code for security on Linux. 

Note that Performance sample code is available only for Linux.

The installer script also contains an option for building the sample code. The
output of which is created in

<INSTALL_PATH>/quickassist/lookaside/access_layer/src/sample_code/build

The installer automatically performs steps 2 & 3 below

===============================================================================

2) The Calgary Corpus

The Performance sample code requires the Calgary corpus to be extract to the 
/lib/firmware folder

a concatenated version of the calgary corpus is located in :

<INSTALL_PATH>/quickassist/lookaside/access_layer/src/sample_code/performance/
compression/calgary

to /lib/firmware


3) Compilation of the Performance Module

The following environment variables need to be set in order to build the modules.

Note: assuming sources were untarred in /QAT directory

setenv ICP_ROOT /QAT
setenv ICP_BUILDSYSTEM_PATH $ICP_ROOT/quickassist/build_system
setenv ICP_ENV_DIR $ICP_ROOT/quickassist/build_system/build_files/env_files
setenv LD_LIBRARY_PATH $ICP_BUILD_OUTPUT

note if you are building onthe Fedora16 kernel you can also 

setenv USE_ZLIB 1

this setting allows the sample code to compress the calgary corpus using default
zlib setting then decompress using the QuickAssist® driver.

Other kernel version may not support the same version of zlib as included in 
FC16 and this can cause compile errors
================================================================================

3) Building Sample Code

To change the Linux's target kernel sources, the sample code to be compiled for, 
from their default location 

/usr/src/kernels/3.1.0-7.fc16.x86_64

set the KERNEL_SOURCE_ROOT environmental variable to point the location of 
desired target kernel sources(e.g. setenv KERNEL_SOURCE_ROOT /usr/src/linux/) 
prior to compilation of Performance Module.

To create the Linux kernel object use the following command from within the 
directories containing the Makefiles:

$ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/

make clean && make perf_all

The generated Linux kernel object, user space executable and kernel module to 
support memory managed for the user space application  is located at:
   
$ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/build/

===============================================================================

4) Performance Module Load/Unload Instructions

To run this performance sample code, on Linux use the following command for 
kernel space: 

insmod cpa_sample_code.ko 

or in user space execute the user space app

insmod qaeMemDrv.ko
./cpa_sample_code

NOTE: all security components must be initialized before attempting to run the 
module.

The following tests will then run:

Cipher Encrypt AES128-CBC: 100000* operations per crypto instance of packet 
sizes 64, 128, 256, 204, 512, 1024, 1152, 2048, 4096, IMIX**

Algorithm Chaining- AES128-CBC HMAC-SHA1: 100000 operations per crypto instance 
of packet sizes 64, 128, 256, 204, 512, 1024, 1152, 2048, 4096, IMIX**

Cipher Encrypt AES256-CBC: 100000* operations per crypto instance of packet 
sizes 64, 128, 256, 204, 512, 1024, 1152, 2048, 4096, IMIX**
A
lgorithm Chaining- AES256-CBC HMAC-SHA512: 100000 operations per crypto 
instance of packet sizes 64, 128, 256, 204, 512, 1024, 1152, 2048, 4096, IMIX**

Algorithm Chaining- AES256-CBC HMAC-AES-XCBC: 100000 operations per crypto 
instance of packet sizes 64, 128, 256, 204, 512, 1024, 1152, 2048, 4096, IMIX**

RSA CRT Decrypt 1024, 2048 and 4096 bit: 10000 operations each

DH 180bit exponent with 1024, 2048 and 4096 bit modulus

DSA L/N Pair of 1024/160 bit: 10000 operations

ECDSA 192 bit binary nist curve: 10000 operations

Deflate Compression/Decompression Level 1 & 3 on the the Calgary Corpus using 
8182 byte buffers. 64k decompression of zlib compresses data if the USE_ZLIB 
option is set

Once the test has completed, "Sample Code Complete" is
displayed. The module can then be unloaded.
   
To unload the kernel module on Linux use the following command:

rmmod cpa_sample_code.ko

*Note the number of operations sent is on a per thread basis. 
The number of threads is controlled by the lower of: 
	the number of cores
	the number of crypto instances
	
by default there are 4 crypto instances enabled on Stargo and 8 on Shumway,

**IMIX is a mixture of packet sizes 40%-64Byte 20%-752Byte 35% 1504Byte 
5%-8892Byte. of the total submissions


The throughput and operations per second output maybe inaccurate for less than 
100000 and 10000 submissions per thread for symmetric and asymmetric respectively. 

By default the sample code is set to submit the minimum number of submissions 
required for an accurate output.

If sample code is to be used to get reliable performance measures one of the 
following needs to be applied prior to running sample code tests:

o	Intel SpeedStep® Technology needs to be disabled in BIOS



===============================================================================

5) Performance Module Control

The cpa_sample_code.ko module and cpa_sample_code user space application 
supports four optional parameters for controlling which tests run and how many 
iterations of each test are executed.

These optional parameters are passed during module loading.

Example:
insmod cpa_sample_code.ko cySymLoops=150 cyAsymLoops=100 dcLoops=100 runTests=31
configFileVer=2

cySymLoops is the number of iterations of Symmetric operations to be executed, 
which affects the running time of the following tests
    a) All Algorithm Chaining tests
    b) All Cipher Encrypt tests
    c) Hash HMAC-SHA1

cyAsymLoops is the number of iterations of Asymmetric operations to be executed, 
which affects the running time of the following tests
    a) RSA CRT Decrypt
    b) DSA
    c) ECDSA
    d) DH

dcLoops is the number of iterations of Compression operations to be executed.

The runTests parameter is a bit masked variable used to control which tests are
 to be executed.

    runTests=1      		Run symmetric tests.
    runTests=2      		Run RSA test.
    runTests=4     		Run DSA test.
    runTests=8     		Run ECDSA test.
    runTests=16    		Run DH test
    runTests=32    		Run Stateless Compression test.
    runTests=63    		Run all tests. (default)
    runTests=32 runStateful=1   Run both stateful and stateless compression test	

The current default is runTests=63, run all tests.    

configFileVer is the type of configuration file used. The default is 2, if the 
wrong version of config file is used the sample code will issue an error
message and fail to find any logical instances.

6) Known Issues

This is sample code and all invalid cases are not fully covered.



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

(C) Intel Corporation 2008  

* Other names and brands may be claimed as the property of others.

===============================================================================
