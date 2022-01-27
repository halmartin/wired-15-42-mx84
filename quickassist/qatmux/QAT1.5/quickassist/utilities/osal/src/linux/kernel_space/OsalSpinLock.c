/**
 * @file kernel_space/OsalSpinLock.c (linux)
 *
 * @brief Implementation for spinlocks
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

#include "Osal.h"

OSAL_PUBLIC OSAL_STATUS
osalLockInit(OsalLock *pLock, OsalLockType lockType)
{
    OSAL_LOCAL_ENSURE(pLock,
                "OsalLockInit(): Null spinlock pointer",
                OSAL_FAIL);

    /* Spinlock type is ignored in case of Linux */
    spin_lock_init (pLock);

    return OSAL_SUCCESS;
}

OSAL_PUBLIC OSAL_STATUS
osalLock(OsalLock *pLock)
{
    OSAL_LOCAL_ENSURE(pLock,
            "OsalLock():   Null spinlock pointer",
            OSAL_FAIL);

    spin_lock (pLock);

    return OSAL_SUCCESS;
}

OSAL_PUBLIC OSAL_STATUS
osalUnlock(OsalLock *pLock)
{
    OSAL_LOCAL_ENSURE(pLock,
            "OsalUnlock():   Null spinlock pointer",
            OSAL_FAIL);

    spin_unlock (pLock);

    return OSAL_SUCCESS;
}

OSAL_PUBLIC OSAL_STATUS
osalLockDestroy(OsalLock *pLock)
{
    OSAL_LOCAL_ENSURE(pLock,
            "OsalLockDestroy():   Null spinlock pointer",
            OSAL_FAIL);

    pLock = NULL;
    return OSAL_SUCCESS;
}

OSAL_PUBLIC OSAL_STATUS osalLockBh(OsalLock *slock)
{
    /* SpinLock  NULL pointer check. */
    OSAL_LOCAL_ENSURE(slock,
             "osalLockBh():   Null spinlock pointer",
             OSAL_FAIL);

    spin_lock_bh(slock);
    return OSAL_SUCCESS;

}


OSAL_PUBLIC OSAL_STATUS osalUnlockBh(OsalLock *slock)
{
    /* SpinLock  NULL pointer check. */
    OSAL_LOCAL_ENSURE(slock,
              "osalUnlockBh():   Null spinlock pointer",
               OSAL_FAIL);

    spin_unlock_bh(slock);
    return OSAL_SUCCESS;
}



OSAL_PUBLIC OSAL_STATUS
osalLockIrqSave(OsalLock *pLock, unsigned long *pFlags)
{
    /* SpinLock  NULL pointer check. */
    OSAL_LOCAL_ENSURE(pLock,
            "OsalLockIrqSave():   Null spinlock pointer",
            OSAL_FAIL);

    OSAL_LOCAL_ENSURE(pFlags,
            "OsalLockIrqSave():   Null flags  pointer",
            OSAL_FAIL);

    spin_lock_irqsave(pLock, *pFlags);
    return OSAL_SUCCESS;
}

OSAL_PUBLIC OSAL_STATUS
osalUnlockIrqRestore(OsalLock *pLock, unsigned long *pFlags)
{
    /* SpinLock  NULL pointer check. */
    OSAL_LOCAL_ENSURE(pLock,
            "OsalUnlockIrqRestore(): Null spinlock pointer",
            OSAL_FAIL);

    OSAL_LOCAL_ENSURE(pFlags,
            "OsalUnlockIrqRestore(): Null flags  pointer",
            OSAL_FAIL);

    spin_unlock_irqrestore(pLock, *pFlags);
    return OSAL_SUCCESS;
}

