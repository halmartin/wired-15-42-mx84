******************************************************************************
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
The aim of this script is to gather data that should help to debug an issue that
the customer is seeing. 
===============================================================================

Running the script
===============================================================================

1) Install the qat driver
2) Make sure that the driver is up
3) Set the ICP_BUILD_OUTPUT to the path to Build output directory, if in the mux
case point the build output folder to that of the QAT1.6 build
4) Run the test case that is causing the issue until the issue is recreated
5) Run ./icp_dump <full path to location of where to store output of debug file>
6) tar ball will be created and stored in location passed in
7) Once the tar ball has been created it needs to be sent on to Intel so the
issue can be looked into


The following is a list of all the details that will be gathered by the script
and stored in the tar ball:
    The output of lspci -vvv
    /proc/icp_*_dev* directories
    The config files that are used /etc/*_qa_dev*.conf
    The environment used
    The output of uname -a
    The output of /var/log/messages
    The kernel config files
    Md5Sum of the firmware used
    Sku version of device
    Bios version of device
    Amount of Memory on the device
    The contents of /proc/cmdline
   

===============================================================================
