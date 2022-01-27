#########################################################################
#
# @par
# GPL LICENSE SUMMARY
# 
#   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
# 
#   This program is free software; you can redistribute it and/or modify 
#   it under the terms of version 2 of the GNU General Public License as
#   published by the Free Software Foundation.
# 
#   This program is distributed in the hope that it will be useful, but 
#   WITHOUT ANY WARRANTY; without even the implied warranty of 
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
#   General Public License for more details.
# 
#   You should have received a copy of the GNU General Public License 
#   along with this program; if not, write to the Free Software 
#   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
#   The full GNU General Public License is included in this distribution 
#   in the file called LICENSE.GPL.
# 
#   Contact Information:
#   Intel Corporation
# 
#  version: QAT1.5.L.1.11.0-36
############################################################################


####################Common variables and definitions########################
# Ensure The ICP_ENV_DIR environmental var is defined.
ifndef ICP_ENV_DIR
$(error ICP_ENV_DIR is undefined. Please set the path to your environment makefile \
        "-> setenv ICP_ENV_DIR <path>")
endif

#Add your project environment Makefile
include $(ICP_ENV_DIR)/environment.mk

#include the makefile with all the default and common Make variable definitions
include $(ICP_BUILDSYSTEM_PATH)/build_files/common.mk
ifdef BANKE_WORKAROUND
INCLUDES += -I$(OSAL_DIR)/src/linux/kernel_space/osalzlib/include 
endif
ifeq ($(ONE_KO_RELEASE_PACKAGE), 1)
EXTRA_CFLAGS+=-DONE_KO_RELEASE_PACKAGE
endif

EXTRA_CFLAGS+=-DOSAL_ENSURE_ON

ifndef CROSS_COMPILE
MACHINE:=$(shell uname -m)
else
ifndef MACHINE
$(error MACHINE is undefined)
endif
endif


ifdef ICP_NUM_PAGES_PER_ALLOC
EXTRA_CFLAGS+=-DICP_NUM_PAGES_PER_ALLOC
endif

ifdef ICP_SRIOV
EXTRA_CFLAGS+=-DICP_SRIOV
ifdef ICP_WITHOUT_IOMMU
EXTRA_CFLAGS+=-DICP_WITHOUT_IOMMU
endif
else
EXTRA_CFLAGS+=-DICP_WITHOUT_IOMMU
endif

ifdef BANKE_WORKAROUND
EXTRA_CFLAGS+=-DBANKE_WORKAROUND
endif

#Add the name for the executable, Library or Module output definitions
OUTPUT_NAME=libosal

SOURCES+= OsalServices.c \
          OsalSemaphore.c \
          OsalSpinLock.c \
          OsalThread.c \
          OsalAtomic.c \
          OsalMemMgmt.c \
          OsalIOMMUMgmt.c \
          OsalUsrKrlProxy.c \
          OsalCryptoInterface.c \
          OsalDevOperations.c 

ifdef BANKE_WORKAROUND      
SOURCES+= OsalBankE.c \
          OsalZlib.c \
          osalzlib/inflate.c \
          osalzlib/inftrees.c \
          osalzlib/adler32.c \
          osalzlib/inffast.c \
          osalzlib/zutil.c \
          osalzlib/crc32.c
endif

#include your $(ICP_OS)_$(ICP_OS_LEVEL).mk file
include $(ICP_ENV_DIR)/$(ICP_OS)_$(ICP_OS_LEVEL).mk

ifneq ($(MACHINE), x86_64)
ASM_SOURCES+= OsalAtomicAddSub.S
else
ifeq ($(ARCH), i386)
ASM_SOURCES+= OsalAtomicAddSub.S
endif
ifeq ($(ARCH), i686)
ASM_SOURCES+= OsalAtomicAddSub.S
endif
endif

install: lib_static

###################Include rules makefiles########################
include $(ICP_BUILDSYSTEM_PATH)/build_files/rules.mk
###################End of Rules inclusion#########################
