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
Overview
===============================================================================
The me_load_test_app reuses the acceleration driver to execute four distinct 
tests, that are replicated across the available logical Intel(r) QuickAssist 
technology (QA) instances:
    1.  Symmetric(cipher): AES-CBC-256 Cipher Test w/ 2k buffers
    2.  Symmetric(auth): SHA-512 Hash Test w/ 2k buffers
    3.  Asymmetric(pke): RSA(1024 bit)
    4.  Compression(dc): L3 Dynamic w/ 4k buffers
    5.  TRNG(trng): 128bit True Random Number Generator
    6.  [alternative to 1, 2]
        Algorithm Chainning:(algchain): AES-CBC-256 + SHA-512

The application package includes a number of configuration files
(dh895xcc_qa_dev<n>.conf/dh89xxcc_qa_dev<n>.conf/c2xxx_qa_dev<n>.conf) in the config_files directory.
These files define the number of QA instances to use on each platform. These instances are mapped to
specific accelerators and execution engines to ensure an even load of the 
individual test threads across the underlying hardware and allow for TDP 
measurements.

===============================================================================
Installing the Accel Load Test Application
===============================================================================
This README assumes that the user has followed the relevant documentation, that
is, the Getting Started Guide (GSG) for building and installing the
acceleration software and the user space sample code.

This application includes a "setup_accel_load_test.sh" bash script for setting 
the environment variables necessary to build the application.

This script assumes that the user installed the driver package to the 
/DH895xCC_Release directory, if an alternate directory was used, please update
the ICP_ROOT=/ variable location in the setup_me_load_test.sh script.


Execute the script with the "source" command:
    source setup_load_test_env.sh

To build the application, execute the following commands from a bash shell:
    rm -rf build/ && make clean && make

The resulting accel_load_test application will be created in the 
./build/linux_2.6/user_space/ sub-directory.

Dependending on the platform being used, copy the corresponding configuration
files from config_files/ to /etc/.

for e.g.

(dh895xcc_qa_dev{n}.conf) from config_files/ to /etc/.
(dh89xxcc_qa_dev{n}.conf) from config_files/ to /etc/.
(cxxx_qa_dev{n}.conf) from config_files/ to /etc/.

Configuration files for SKU2 are also included, where a single accelerator
is available on each device.
If using a SKU2 on the system, copy and rename the dh895xcc_qa_dev{n}.conf.SKU2
to /etc/dh895xcc_qa_dev{n}.conf, where {n} is the device number.

To ensure that the acceleration driver picks up the custom configuration, it is 
necessary to execute the following:
    /DH895xCC_Release/build/adf_ctl down
followed by:
    /DH895xCC_Release/build/adf_ctl up

===============================================================================
Running the Application
===============================================================================

Load the modules using following commands,
    insmod /DH895xCC_Release/quickassist/lookaside/access_layer/src/sample_code/build/qaeMemDrv.ko

Depending on the platform being used, arguments in command need to change

The application can be run with arguments:
    ./build/linux_2.6/user_space/accel_load_test numDc=12 numPke=6 numCipher=12 numAuth=12 numTrng=2 numLoops=n
    ./build/linux_2.6/user_space/accel_load_test numDc=6 numPke=6 numCipher=6 numAuth=6 numTrng=2 numLoops=n
    ./build/linux_2.6/user_space/accel_load_test numDc=0 numPke=2 numCipher=6 numAuth=6 numTrng=0 numLoops=n

Besides, this application provides other arguments, for instance, numAlgchain(
alternative for Cipher and Auth), symPacketSize, dcPacketSize, and numLoops. 
Each will have its default value if not specified by user input.
	
The application is designed to run over a number of hours if the numLoops is 
set large enough, but it can be stopped by sending a SIGINT(Ctrl-c) signal.


Notes:
1. There are 6 CPM slices in one accelerator, and for each slice, it only 
supports 2 data compression, 5 public key encryption, 2 cipher and 2 
authentication service. The max number of true random number generator is 2, of 
the entire accelerator. The command above shows a example about how to maximize
the workload for the accelerator. However, if one specific number is set to 0,
the corresponding service will be disabled and thus, not run.
 
2. The performance statistics are not printed after Ctrl-c. They are only 
printed if the application is left to run to completion. 

3. If the input parameter 'numLoops' is not set, this application will last
for quite a long time. The practical way is to set 'numLoops' to 5000, 10000, 
or 50000, and the application will finish within dozens of minutes.

4. Any performance statistics printed should not be taken out of context as
individual threads are not synchronised and therefore skew results. Some 
threads will return before others. If thread statistics are printed while 
measurements are being taken, it is adivsable to restart the application to 
ensure the hardware is under sufficient load.


To verify that operations are being executed on each accelerator, the device
statistics can be queried:

For example, for the first SKU1/SKU4 device on the system:
    cat /proc/icp_dh895xcc_dev0/qat0 
    cat /proc/icp_dh895xcc_dev0/qat1

For the second SKU1/SKU4 device on the system (if present):
    cat /proc/icp_dh895xcc_dev1/qat0
    cat /proc/icp_dh895xcc_dev1/qat1

It is recommended to wait 2-3 minutes for after the "instances initialization
completed" message is printed before taking measurements, as the memory
allocation for the compression threads can delay the execution of 
operations and may skew measurments taken earlier.
As it may take some time for the application to initialize, it is 
recommended that the device statistics are queried first to see 
operations in-flight before taking measurements.


A sample output looks like the following:

Sample Output:
./build/linux_2.6/user_space/accel_load_test numDc=12 numPke=6 numCipher=12 numAuth=12 numTrng=2
Initializing user space "LoadTest" instances...
"LoadTest" instances initialization completed
Creating Crypto Tests across 64 logical instances
Creating RSA(mod 1024 Type 2) thread: instance   0, core  0, device  0, accelerator  0, engine  0
Creating RSA(mod 1024 Type 2) thread: instance   1, core  0, device  0, accelerator  0, engine  0
Creating RSA(mod 1024 Type 2) thread: instance   2, core  1, device  0, accelerator  0, engine  0
Creating RSA(mod 1024 Type 2) thread: instance   3, core  1, device  0, accelerator  0, engine  0
Creating RSA(mod 1024 Type 2) thread: instance   4, core  2, device  0, accelerator  0, engine  0
Creating RSA(mod 1024 Type 2) thread: instance   5, core  2, device  0, accelerator  0, engine  0
Creating Cipher(AES_CBC) thread:      instance   6, core  3, device  0, accelerator  0, engine  0
Creating Cipher(AES_CBC) thread:      instance   7, core  3, device  0, accelerator  0, engine  0
Creating Cipher(AES_CBC) thread:      instance   8, core  4, device  0, accelerator  0, engine  0
Creating Cipher(AES_CBC) thread:      instance   9, core  4, device  0, accelerator  0, engine  0
Creating Cipher(AES_CBC) thread:      instance  10, core  5, device  0, accelerator  0, engine  0
Creating Cipher(AES_CBC) thread:      instance  11, core  5, device  0, accelerator  0, engine  0
Creating Cipher(AES_CBC) thread:      instance  12, core  6, device  0, accelerator  0, engine  0
Creating Cipher(AES_CBC) thread:      instance  13, core  6, device  0, accelerator  0, engine  0
Creating Cipher(AES_CBC) thread:      instance  14, core  7, device  0, accelerator  0, engine  0
Creating Cipher(AES_CBC) thread:      instance  15, core  7, device  0, accelerator  0, engine  0
Creating Cipher(AES_CBC) thread:      instance  16, core  8, device  0, accelerator  0, engine  0
Creating Cipher(AES_CBC) thread:      instance  17, core  8, device  0, accelerator  0, engine  0
Creating Hash(Auth SHA512) thread:    instance  18, core  9, device  0, accelerator  0, engine  0
Creating Hash(Auth SHA512) thread:    instance  19, core  9, device  0, accelerator  0, engine  0
Creating Hash(Auth SHA512) thread:    instance  20, core 10, device  0, accelerator  0, engine  0
Creating Hash(Auth SHA512) thread:    instance  21, core 10, device  0, accelerator  0, engine  0
Creating Hash(Auth SHA512) thread:    instance  22, core 11, device  0, accelerator  0, engine  0
Creating Hash(Auth SHA512) thread:    instance  23, core 11, device  0, accelerator  0, engine  0
Creating Hash(Auth SHA512) thread:    instance  24, core 12, device  0, accelerator  0, engine  0
Creating Hash(Auth SHA512) thread:    instance  25, core 12, device  0, accelerator  0, engine  0
Creating Hash(Auth SHA512) thread:    instance  26, core 13, device  0, accelerator  0, engine  0
Creating Hash(Auth SHA512) thread:    instance  27, core 13, device  0, accelerator  0, engine  0
Creating Hash(Auth SHA512) thread:    instance  28, core 14, device  0, accelerator  0, engine  0
Creating Hash(Auth SHA512) thread:    instance  29, core 14, device  0, accelerator  0, engine  0
Creating Nrbg thread:                 instance  30, core 15, device  0, accelerator  0, engine  0
Creating Nrbg thread:                 instance  31, core 15, device  0, accelerator  0, engine  0
Creating Compression Tests across 64 logical instances
Creating Compression thread:          instance   0, core  0, device  0, accelerator  0
Creating Compression thread:          instance   1, core  0, device  0, accelerator  0
Creating Compression thread:          instance   2, core  1, device  0, accelerator  0
Creating Compression thread:          instance   3, core  1, device  0, accelerator  0
Creating Compression thread:          instance   4, core  2, device  0, accelerator  0
Creating Compression thread:          instance   5, core  2, device  0, accelerator  0
Creating Compression thread:          instance   6, core  3, device  0, accelerator  0
Creating Compression thread:          instance   7, core  3, device  0, accelerator  0
Creating Compression thread:          instance   8, core  4, device  0, accelerator  0
Creating Compression thread:          instance   9, core  4, device  0, accelerator  0
Creating Compression thread:          instance  10, core  5, device  0, accelerator  0
Creating Compression thread:          instance  11, core  5, device  0, accelerator  0
Starting test execution, press Ctl-c to exit


===============================================================================
Running the Ethernet Loop-back Test
===============================================================================

For TDP testing that involves the embedded Ethernet device(s) (if available), 
the following test can be run from the command line using "ethtool".
For each interface, run "ethtool -t <interface-name>". This test generally lasts
for 15 seconds and can be looped over using a shell script, for example, using 
bash:

    #for i in {1..300}
    #do
    #   ethtool -t <interface>
    #done

