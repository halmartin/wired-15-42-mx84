/**
 * @file kernel_space/OsalAtomicAddSub.S (linux)
 *
 * @brief OS-specific Atomic API's implementation.
 *
 *
 * @par
 * GPL LICENSE SUMMARY
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
 *  version: QAT1.5.L.1.11.0-36
 */

.file "OsalAtomicAddSub.S"
.text

.global  osal_atomic_add
osal_atomic_add:
    push %ebp
    push %ebx
    push %esi
    push %edi
    movl %eax, %esi
    movl %edx, %ebp
    movl %ecx, %edi
    movl (%edi), %eax
    movl 4(%edi), %edx
    lab1:
    movl %eax, %ebx
    movl %edx, %ecx
    addl %esi, %ebx
    adcl %ebp, %ecx
    lock cmpxchg8b (%edi)
    jnz lab1
    movl %ebx, %eax
    movl %ecx, %edx
    pop  %edi
    pop  %esi
    pop  %ebx
    pop  %ebp
    ret

.global  osal_atomic_sub
osal_atomic_sub:
    push %ebp
    push %ebx
    push %esi
    push %edi
    movl %eax, %esi
    movl %edx, %ebp
    movl %ecx, %edi
    movl (%edi), %eax
    movl 4(%edi), %edx
    lab2:
    movl %eax, %ebx
    movl %edx, %ecx
    subl %esi, %ebx
    sbbl %ebp, %ecx
    lock cmpxchg8b (%edi)
    jnz lab2
    movl %ebx, %eax
    movl %ecx, %edx
    pop  %edi
    pop  %esi
    pop  %ebx
    pop  %ebp
    ret

.global  osal_atomic_inc
osal_atomic_inc:
    push %ebx
    push %edi
    movl %eax, %edi
    movl (%edi), %eax
    movl 4(%edi), %edx
    lab3:
    movl %eax, %ebx
    movl %edx, %ecx
    addl $1, %ebx
    adcl $0, %ecx
    lock cmpxchg8b (%edi)
    jnz lab3
    movl %ebx, %eax
    movl %ecx, %edx
    pop  %edi
    pop  %ebx
    ret

.global  osal_atomic_dec
osal_atomic_dec:
    push %ebx
    push %edi
    movl %eax, %edi
    movl (%edi), %eax
    movl 4(%edi), %edx
    lab4:
    movl %eax, %ebx
    movl %edx, %ecx
    subl $1, %ebx
    sbbl $0, %ecx
    lock cmpxchg8b (%edi)
    jnz lab4
    movl %ebx, %eax
    movl %ecx, %edx
    pop  %edi
    pop  %ebx
    ret

