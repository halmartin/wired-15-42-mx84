/**
 **************************************************************************
 * @file halMmapUsr.c
 *
 * @description
 *      This file provides implementation of MEv2 HAL library
 *
 * @par 
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
 **************************************************************************/ 

#include "halAe_platform.h"
#include "core_io.h"
#include "icp_firml_handle.h"
#include "icp_firml_interface.h"
#include "halAeChip.h"
#include "halAeDrv.h"

extern int DriverFd;

/**
 *****************************************************************************
 * @ingroup icp_hal
 * 
 * @description
 *     Map the physical address to virtual address
 *
 * @param mTab - IN
 * @param 
 *
 * @retval  
 * 
 * 
 *****************************************************************************/
void halAe_DrvMmap(icp_firml_handle_t * handle)
{
    /* remap dram virtual addresses */
    handle->sysMemInfo.dramDesc[0].dramBaseAddr_v = 
        (unsigned long)mmap(0, (unsigned long)handle->sysMemInfo.dramDesc[0].dramSize, PROT_READ | PROT_WRITE, 
                           MAP_SHARED, DriverFd, (unsigned long)handle->sysMemInfo.dramDesc[0].dramBaseAddr);
                           
DBGINFO(("DRAM0: base = 0x%llx, size = 0x%llx, virt = 0x%llx \n", handle->sysMemInfo.dramDesc[0].dramBaseAddr, handle->sysMemInfo.dramDesc[0].dramSize, handle->sysMemInfo.dramDesc[0].dramBaseAddr_v ));
                           
    handle->sysMemInfo.dramDesc[1].dramBaseAddr_v = 
        (unsigned long)mmap(0, (unsigned long)handle->sysMemInfo.dramDesc[1].dramSize, PROT_READ | PROT_WRITE, 
                           MAP_SHARED, DriverFd, (unsigned long)handle->sysMemInfo.dramDesc[1].dramBaseAddr);
DBGINFO(("DRAM1: base = 0x%llx, size = 0x%llx, virt = 0x%llx \n", handle->sysMemInfo.dramDesc[1].dramBaseAddr, handle->sysMemInfo.dramDesc[1].dramSize, handle->sysMemInfo.dramDesc[1].dramBaseAddr_v ));

    /* remap sram virtual addresses */
    handle->sysMemInfo.sramDesc.sramBaseAddr_v = 
        (unsigned long)mmap(0, (unsigned long)handle->sysMemInfo.sramDesc.sramSize, PROT_READ | PROT_WRITE, 
                           MAP_SHARED, DriverFd, (unsigned long)handle->sysMemInfo.sramDesc.sramBaseAddr);

DBGINFO(("SRAM: base = 0x%llx, size = 0x%x, virt = 0x%llx \n", handle->sysMemInfo.sramDesc.sramBaseAddr, handle->sysMemInfo.sramDesc.sramSize, handle->sysMemInfo.sramDesc.sramBaseAddr_v ));

    /* remap PCI BAR virtual addresses */
    handle->sysMemInfo.pciBars[ICP_AC_PESRAM_BAR].virtAddr = handle->sysMemInfo.sramDesc.sramBaseAddr_v;

DBGINFO(("PESRAM BAR: base = 0x%llx, size = 0x%llx, virt = 0x%llx \n", handle->sysMemInfo.pciBars[ICP_AC_PESRAM_BAR].baseAddr, handle->sysMemInfo.pciBars[ICP_AC_PESRAM_BAR].size, handle->sysMemInfo.pciBars[ICP_AC_PESRAM_BAR].virtAddr ));

    handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].virtAddr =
        (unsigned long)mmap(0, (unsigned long)handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].size, PROT_READ | PROT_WRITE, 
                           MAP_SHARED, DriverFd, (unsigned long)handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].baseAddr);

DBGINFO(("PMISC BAR: base = 0x%llx, size = 0x%llx, virt = 0x%llx \n", handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].baseAddr, handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].size, handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].virtAddr ));

    handle->sysMemInfo.pciBars[ICP_AC_PETRINGCSR_BAR].virtAddr =
        (unsigned long)mmap(0, (unsigned long)handle->sysMemInfo.pciBars[ICP_AC_PETRINGCSR_BAR].size, PROT_READ | PROT_WRITE, 
                           MAP_SHARED, DriverFd, (unsigned long)handle->sysMemInfo.pciBars[ICP_AC_PETRINGCSR_BAR].baseAddr);

DBGINFO(("PETRINGCSR BAR: base = 0x%llx, size = 0x%llx, virt = 0x%llx \n", handle->sysMemInfo.pciBars[ICP_AC_PETRINGCSR_BAR].baseAddr, handle->sysMemInfo.pciBars[ICP_AC_PETRINGCSR_BAR].size, handle->sysMemInfo.pciBars[ICP_AC_PETRINGCSR_BAR].virtAddr ));

}

void 
exportVirAddr(icp_firml_handle_t * handle)
{
    unsigned int cap_offset, ae_offset, pmu_offset, scratch_offset, ssu_offset, ep_offset;
    GET_OFFSET(handle->sysMemInfo.deviceId, cap_offset, ae_offset, pmu_offset, scratch_offset, ssu_offset, ep_offset);	
    /* Map the physical address to user space virtual address via mmap*/
    halAe_DrvMmap(handle);

    handle->Hal_dram_ch0_virtAddr = handle->sysMemInfo.dramDesc[0].dramBaseAddr_v;
    handle->Hal_dram_ch1_virtAddr = handle->sysMemInfo.dramDesc[1].dramBaseAddr_v;
    handle->Hal_sram_virtAddr = handle->sysMemInfo.sramDesc.sramBaseAddr_v;           

    handle->Hal_cap_global_ctl_csr_virtAddr = SADD(handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].virtAddr, cap_offset);
    handle->Hal_cap_ae_xfer_csr_virtAddr = SADD(handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].virtAddr, ae_offset);
    handle->Hal_cap_ae_local_csr_virtAddr = SADD(handle->Hal_cap_ae_xfer_csr_virtAddr, LOCAL_TO_XFER_REG_OFFSET);
    handle->Hal_cap_pmu_csr_virtAddr = SADD(handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].virtAddr, pmu_offset);
    handle->Hal_cap_hash_csr_virtAddr = SADD(handle->Hal_cap_global_ctl_csr_virtAddr, HASH_TO_GLOBAL_REG_OFFSET);
    handle->Hal_scratch_rd_wr_swap_virtAddr = SADD(handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].virtAddr, scratch_offset);
    handle->Hal_ae_fastaccess_csr_virtAddr = 0; 

    handle->Hal_ssu_csr_virtAddr = SADD(handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].virtAddr, ssu_offset);
    handle->Hal_ep_csr_virtAddr = SADD(handle->sysMemInfo.pciBars[ICP_AC_PMISC_BAR].virtAddr, ep_offset);
    handle->Hal_eagletail_ring_csr_virtAddr = handle->sysMemInfo.pciBars[ICP_AC_PETRINGCSR_BAR].virtAddr;

    return;
}

int halAe_readRam_IA(uint64 addr, unsigned int *value)
{
    unsigned long addr_v = 0;
    unsigned long size = getpagesize();
    int rt=0;
    DBGINFO(("Read From IA\n")); 
   
    addr_v = (unsigned long)mmap(0, size, PROT_READ, MAP_SHARED, DriverFd, (unsigned long)(addr & ~(size-1)));   
    
    if(addr_v == -1)
    {
        ERRINFO(("Error while maping memory, size=0x%lx, addr=0x%llx, addr_v=0x%lx, errno=0x%x\n", size, addr, addr_v, errno)); 
#if (defined(_DBG_PRINT))
        perror("mmap failed");
#endif
        return (HALAE_FAIL);
    }
    
    DBGINFO(("size=0x%lx, addr=0x%llx, addr_v=0x%lx\n", size, addr, addr_v));
    /*get the value from the virtual address*/
    *value = *((unsigned int *)(addr_v + ((unsigned long)addr & (size-1))));
    rt=munmap((caddr_t)addr_v, size);     //make sure the address is unmapped
    if(rt == -1)
    {
         ERRINFO(("Error while unmaping memory, size=0x%lx, addr=0x%llx, addr_v=0x%lx, errno=0x%x\n", size, addr, addr_v, errno)); 
#if (defined(_DBG_PRINT))
        perror("unmap failed");
#endif
        return (HALAE_FAIL);
    }
    return (HALAE_SUCCESS);
}                              

int halAe_writeRam_IA(uint64 addr, unsigned int value)
{
    unsigned long addr_v = 0;
    unsigned long size = getpagesize();
    int rt=0;
    DBGINFO(("Write From IA\n")); 
   
    addr_v = (unsigned long)mmap(0, size, PROT_WRITE, MAP_SHARED, DriverFd, (unsigned long)(addr & ~(size-1)));   
    if(addr_v == -1)
    {
        ERRINFO(("Error while maping memory, size=0x%lx, addr=0x%llx, addr_v=0x%lx, errno=0x%x\n", size, addr, addr_v, errno)); 
#if (defined(_DBG_PRINT))
        perror("mmap failed");
#endif
        return (HALAE_FAIL);
    }
   
    DBGINFO(("size=0x%lx, addr=0x%llx, addr_v=0x%lx\n", size, addr, addr_v));
    /*get the value from the virtual address*/
    *((unsigned int *)(addr_v + ((unsigned long)addr & (size-1)))) = value;
    rt=munmap((caddr_t)addr_v, size);    //make sure the address is unmapped
    if(rt == -1)
    {
         ERRINFO(("Error while unmaping memory, size=0x%lx, addr=0x%llx, addr_v=0x%lx, errno=0x%x\n", size, addr, addr_v, errno)); 
#if (defined(_DBG_PRINT))
        perror("unmap failed");
#endif
        return (HALAE_FAIL);
    }

    return (HALAE_SUCCESS);    
}

