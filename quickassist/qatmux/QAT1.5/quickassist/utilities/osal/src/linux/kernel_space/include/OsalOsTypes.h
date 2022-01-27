/**
 * @file OsalOsTypes.h
 *
 * @brief Linux-specific data type
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

#ifndef OSAL_OS_TYPES_H
#define OSAL_OS_TYPES_H

#ifdef __cplusplus
extern "C"{
#endif

#include <linux/types.h>
#include <linux/version.h>
#include <linux/compiler.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/param.h>


#include <linux/slab.h>
/* Include PCI to include PCI definitions*/
#ifdef ICP_SRIOV
#ifndef CONFIG_PCI_IOV
#error "Please enable PCI IOV support in kernel configuration [make menuconfig]"
#endif
#endif
#include <linux/pci.h>
#if KERNEL_VERSION(2,6,0) <= LINUX_VERSION_CODE

#include <linux/sched.h>
#include <linux/kthread.h>

#endif /* KERNEL_VERSION_2.6 */
 
#if KERNEL_VERSION(3,10,0) <= LINUX_VERSION_CODE
#include <linux/sched/rt.h>
#endif /* KERNEL_VERSION(3,10,0) */

#include <linux/spinlock.h>

#if LINUX_VERSION_CODE  < KERNEL_VERSION(2,6,18)
#include <linux/interrupt.h>
#endif  /*< KERNEL_VERSION(2,6,18)*/

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
#include <asm/semaphore.h>
#else
#include <linux/semaphore.h>
#endif

#ifdef USE_NATIVE_OS_TIMER_API
#include <linux/timer.h>
#endif

#include <linux/wait.h>
#include <asm/io.h>

#ifndef OSAL_PUBLIC
#define OSAL_PUBLIC
#endif /* OSAL_PUBLIC */

/* Zlib inflate overflow error. Same error code
 * as in the external API definition.
 */
#ifndef OSAL_DC_OVERFLOW
#define OSAL_DC_OVERFLOW       (-11)
#endif

/* Zlib deflate window size */
#ifndef DEFLATE_DEF_WINBITS
#define DEFLATE_DEF_WINBITS    (15)
#endif

/* XOR value required for the CRC calculation in user space */
#ifndef OSAL_XOR_VALUE
#define OSAL_XOR_VALUE         (0xffffffff)
#endif

/* Zlib CRC 32 checksum type definition.
 * Same definition as in the external API.
 */
#ifndef OSAL_DC_CRC32
#define OSAL_DC_CRC32          (1)
#endif

/* Zlib ADLER 32 checksum type definition./
 * Same definition as in the external API.
 */
#ifndef OSAL_DC_ADLER32
#define OSAL_DC_ADLER32        (2)
#endif

#ifndef __ACTYPES_H__
typedef uint8_t      UINT8;    /**< 8-bit unsigned integer */
typedef uint16_t     UINT16;   /**< 16-bit unsigned integer */
typedef uint32_t     UINT32;   /**< 32-bit unsigned integer */
typedef uint64_t     UINT64;   /**< 64-bit unsigned integer */
typedef int64_t      INT64;    /**< 64-bit signed integer */
typedef int32_t      INT32;    /**< 32-bit signed integer */
#endif /* __ACTYPES_H__ */
typedef int16_t      INT16;    /**< 16-bit signed integer */

typedef int8_t       INT8;    /**< 8-bit signed integer */
typedef UINT32     ULONG;   /**< alias for UINT32 */
typedef UINT16     USHORT;  /**< alias for UINT16 */
typedef UINT8      UCHAR;   /**< alias for UINT8 */
typedef UINT32     BOOL;    /**< alias for UINT32 */
typedef INT8       CHAR;    /**< alias for INT8*/
typedef void       VOID;

typedef struct chr_drv_info_s {
    struct module          *owner;
    unsigned               major;
    unsigned               min_minor;
    unsigned               max_minor;
    char                   *name;
    struct file_operations *file_ops;
    struct cdev            drv_cdev;
    struct class           *drv_class;
    struct device          *drv_class_dev;
    unsigned               num_devices;
} chr_drv_info_t;

#define OSAL_OS_PAGE_SIZE	4096

/* Default stack limit is 10 KB */
#define OSAL_OS_THREAD_DEFAULT_STACK_SIZE  (10240)

/* Maximum stack limit is 32 MB */
#define OSAL_OS_THREAD_MAX_STACK_SIZE      (33554432)  /* 32 MBytes */

/* Thread minimum priority */
#define OSAL_OS_MIN_THREAD_PRIORITY        (0)

/* Default thread priority */
#define OSAL_OS_DEFAULT_THREAD_PRIORITY    (MAX_RT_PRIO-1)

/* Thread maximum priority (0 - 255). 0 - highest priority */
#define OSAL_OS_MAX_THREAD_PRIORITY          (MAX_PRIO)

/* Maximum input value for priority */
#define OSAL_PRIO_SET_MAX_VALID_VAL        (255)

/* Maximum supported priority value in ThreadPrioritySet */
#define OSAL_PRIO_SET_MAX_VAL              (39)

/* Difference of actual nice value and input value */
#define OSAL_NICE_VAL_DIFFERENCE           (20)

/* Default scheduling policy */
#define OSAL_OS_THREAD_DEFAULT_SCHED_POLICY SCHED_RR

/* Thread scheduling policy - Round Robin */
#define OSAL_THREAD_SCHED_RR                 SCHED_RR

/* Thread scheduling policy - FiFo */
#define OSAL_THREAD_SCHED_FIFO               SCHED_FIFO

 /* Thread scheduling policy - Other */
#define OSAL_THREAD_SCHED_OTHER              SCHED_OTHER

#define OSAL_OS_WAIT_FOREVER               (-1)
#define OSAL_OS_WAIT_NONE                  0

#define OSAL_OS_MMU_VIRT_TO_PHYS(addr) ((UINT64)((addr) ? virt_to_phys((void*)(addr)) : 0))

#define OSAL_OS_MMU_PHYS_TO_VIRT(addr)  ((addr) ? phys_to_virt((unsigned long)(addr)) : 0)

/* Thread handle is a task_struct pointer */
typedef struct task_struct *OsalThread;

/* Semaphore handle */
typedef struct semaphore *OsalSemaphore;

/* Mutex handle */
typedef struct semaphore *OsalMutex;

#ifdef _OSAL_OEM_FAST_MUTEX

typedef int OsalOsFastMutex;

#else /* ! _OSAL_OEM_FAST_MUTEX -> Generic */

/*
 * Fast mutex handle - fast mutex operations are implemented
 * using the linux atomic instructions.
 */
typedef atomic_t OsalFastMutex;

#endif /* _OSAL_OEM_FAST_MUTEX */

typedef spinlock_t OsalLock;

/* Dummy typedef for OsThreadAttr - This is not used in linux currently.
   This needs to be defined appropriately when it is planned to be used */
typedef int OsalOsThreadAttr;

typedef void (*voidFnVoidPtr) (void *);
typedef void (*voidFnVoid) (void);

#ifdef USE_NATIVE_OS_TIMER_API
typedef void (*voidFnULongPtr)(unsigned long);

typedef struct
{
    BOOL  inUse;             /* status of timer active or cancel */
    BOOL  isRepeating;       /* Timer is repeating type */
    voidFnVoidPtr  callback;  /* Function to be called back after period ms */
    UINT32 priority;          /* priority */
    void  *callbackParam;     /* parameter to be passed to callback function*/
    UINT32 period;            /* period in mili seconds */
    struct timer_list timer;  /* Linux OS timer struct */
} OsalTimerRec;

typedef  OsalTimerRec *OsalOsTimer;
#endif /* USE_NATIVE_OS_TIMER_API */

/**
 *
 * @brief Private data structure
 *
 *  linux data struct to store the information on the
 *  memory allocated. This structure is stored at the beginning of
 *  the allocated chunck of memory
 *  size is the no of byte passed to the memory allocation functions
 *  mSize is the real size of the memory required to the OS
 *
 *  +--------------------------+--------------------------------+
 *  | ixOsalMemAllocInfoStruct | memory returned to user (size) |
 *  +--------------------------+--------------------------------+
 *  ^                          ^
 *  mAllocMemPtr               Ptr returned to the caller of MemAlloc*
 *
 */
typedef struct _sMemAllocInfo
{
    VOID*               mAllocMemPtr;   /* memory addr returned by the kernel */
    UINT32              mSize;          /* allocated size */
} OsalMemAllocInfoStruct;

#if LINUX_VERSION_CODE  >= KERNEL_VERSION(2,6,31) || defined(CONFIG_64BIT)
typedef atomic64_t OsalAtomic;
#else
typedef struct { uint64_t val __attribute__ ((aligned (8))); } atomic_val;
typedef atomic_val OsalAtomic;
#endif

/**
 * @ingroup IxOsalTypes
 * @enum OsalSpinLockType
 * @brief This is an emum for OSAL SpinLock types.
*/
typedef enum
{
    TYPE_IGNORE = 0,                /**< Spin Lock type Ignore */
    NON_QUEUED = 1,                 /**< Non Queued Spin Lock type */
    QUEUED = 2,                     /**< Queued Spin Lock type  */
    NON_QUEUED_AT_DPC_LEVEL = 3,    /**< Non Queued Spin Lock type at DPC level */
    QUEUED_AT_DPC_LEVEL             /**< Queued Spin Lock type at DPC level */
}OsalLockType;

/* String manipulation definitions */
#define OSAL_OS_GET_STRING_LENGTH(str) strlen(str)

#define OSAL_MEM_ASSERT(c) \
if(!(c)) \
{ \
    BUG(); \
}

/*
 * s - memory size
 * a - memory alignment
*/
#define OSAL_MEM_PADDING(s, a) ( ( a - (s % a)) % a  )

#define OSAL_OS_NW_TO_HOST_16(uData)  be16_to_cpu(uData)
#define OSAL_OS_NW_TO_HOST_32(uData)  be32_to_cpu(uData)
#define OSAL_OS_NW_TO_HOST_64(uData)  be64_to_cpu(uData)

#define OSAL_OS_HOST_TO_NW_16(uData)  cpu_to_be16(uData)
#define OSAL_OS_HOST_TO_NW_32(uData)  cpu_to_be32(uData)
#define OSAL_OS_HOST_TO_NW_64(uData)  cpu_to_be64(uData)

#define OSAL_OS_UDIV64_32(dividend, divisor)  \
({                                            \
         UINT64 _div = dividend;              \
         do_div(_div, divisor);               \
         _div;                                \
})

#define OSAL_OS_UMOD64_32(dividend, divisor)  \
({                                            \
         UINT64 _div = dividend;              \
         do_div(_div, divisor);               \
 })

#ifndef BANK_E_REGISTER_SIZE
#define BANK_E_REGISTER_SIZE        (16)
#endif

/* Bank E is 4Kbyte size */
#ifndef BANK_E_SIZE
#define BANK_E_SIZE (0x1000)
#endif

#ifndef BANK_E_1K_SIZE
#define BANK_E_1K_SIZE	            (1024)
#endif

#ifndef BANK_E_2K_SIZE
#define BANK_E_2K_SIZE	            (2048)
#endif

#ifndef BANK_E_3K_SIZE
#define BANK_E_3K_SIZE	            (3072)
#endif

#ifndef CODE_128
#define CODE_128                    (128)
#endif

#ifndef BANK_E_LIT_MAP_SIZE
#define BANK_E_LIT_MAP_SIZE         (256)
#endif

/* maximum number of distance codes */
#ifndef BANK_E_MAXDCODES
#define BANK_E_MAXDCODES            (30)
#endif

/* maximum number of literal/length codes */
#ifndef BANK_E_MAXLCODES
#define BANK_E_MAXLCODES            (286)
#endif

#ifdef BANKE_WORKAROUND
/**
 * @ingroup IxOsalTypes
 * @typedef huffman_s
 * @brief Huffman code decoding tables.  count[1..MAXBITS] is the number of
 *        symbols of each length, which for a canonical code are stepped
 *        through in order. symbol[] are the symbol values in canonical order,
 *        where the number of entries is the sum of the counts in count[].
 *        The huffman_table_t structure defined here is a redefiniton of the
 *        huffman_t structure defined in dc_flattree.h
 */
typedef struct huffman_table_s
{
	INT16 count[BANK_E_MAXLCODES];   /* number of symbols of each length */
	INT16 symbol[BANK_E_MAXLCODES];  /* canonically ordered symbols */
	INT16 lit[BANK_E_MAXLCODES];     /* canonically ordered literals */
	UINT8 bit_width[BANK_E_MAXLCODES];/* bit width for symbol */
}huffman_table_t;

/**
 * @ingroup IxOsalTypes
 * @typedef osalz_stream_s
 * @brief Duplicate type for z_stream defined in zlib.h,this is required not to include zlib.h in osal
 */
typedef struct osalz_stream_s {
    const UINT8 *next_in;     /* next input byte */
    UINT32    avail_in;  /* number of bytes available at next_in */
    UINT32    total_in;  /* total number of input bytes read so far */

    UINT8    *next_out; /* next output byte should be put there */
    UINT32    avail_out; /* remaining free space at next_out */
    UINT32    total_out; /* total number of bytes output so far */

    const char *msg;  /* last error message, NULL if no error */
    struct internal_state *state; /* not visible by applications */

    void *workspace; /* memory allocated for this stream */

    INT16    data_type;  /* best guess about the data type: binary or text */
    UINT32   adler;      /* adler32 value of the uncompressed data */
    UINT32   reserved;   /* reserved for future use */
} osalz_stream;

#endif /* BANKE_WORKAROUND */

#ifdef __cplusplus
}
#endif


#endif /* OSAL_OS_TYPES_H */

