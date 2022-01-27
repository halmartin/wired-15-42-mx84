#!/usr/bin/env bash

function usage() {
    echo "Invalid arguments. Usage: meraki_qatmux_build -a <arch> -c <cross_compiler> -k <kernel_source_root> -o <output_dir> -r <icp_root> -s <openwrt_staging_dir> -t <targets>"
    exit 1
}

function echo_args() {
    echo "ICP_ROOT=${ICP_ROOT}"
    echo "ICP_ARCH_USER=${ICP_ARCH_USER}"
    echo "KERNEL_SOURCE_ROOT=${KERNEL_SOURCE_ROOT}"
    echo "ICP_BUILD_OUTPUT=${ICP_BUILD_OUTPUT}"
    echo "CROSS_COMPILE=${CROSS_COMPILE}"
    echo "MERAKI_CROSS_COMPILE_STAGING_DIR=${MERAKI_CROSS_COMPILE_STAGING_DIR}"
    echo "MACHINE=${MACHINE}"
    echo "QATMUX_TARGET=${QATMUX_TARGET}"
}

set -e # exit on any error

ICP_ROOT=
ICP_ARCH_USER=
KERNEL_SOURCE_ROOT=
ICP_BUILD_OUTPUT=
CROSS_COMPILE=
MERAKI_CROSS_COMPILE_STAGING_DIR=
MACHINE=
QATMUX_TARGET=

while getopts "a:c:k:o:r:s:t:" opt; do
    case "${opt}" in
        a)
            ICP_ARCH_USER=${OPTARG}
            MACHINE=${OPTARG}
            ;;
        c)
            CROSS_COMPILE=${OPTARG}
            ;;
        k)
            KERNEL_SOURCE_ROOT=${OPTARG}
            ;;
        o)
            ICP_BUILD_OUTPUT=${OPTARG}
            ;;
        r)
            ICP_ROOT=${OPTARG}
            ;;
        s)
            MERAKI_CROSS_COMPILE_STAGING_DIR=${OPTARG}
            ;;
        t)
            QATMUX_TARGET=${OPTARG}
            ;;
        *)
            usage
            ;;
    esac
done
shift $((OPTIND-1))

[ -z "${ICP_ARCH_USER}" ] && usage
[ -z "${MACHINE}" ] && usage
[ -z "${CROSS_COMPILE}" ] && usage
[ -z "${KERNEL_SOURCE_ROOT}" ] && usage
[ -z "${ICP_BUILD_OUTPUT}" ] && usage
[ -z "${ICP_ROOT}" ] && usage
[ -z "${MERAKI_CROSS_COMPILE_STAGING_DIR}" ] && usage
[ -z "${QATMUX_TARGET}" ] && usage

echo "***** meraki_qatmux_build: Making *****" && echo_args

ICP_ENV_DIR=${ICP_ROOT}/quickassist/build_system/build_files/env_files
ICP_BUILDSYSTEM_PATH=${ICP_ROOT}/quickassist/build_system

# Default is defined in quickassist top Makefile but doesn't make it to all other Makefiles in sub-directories
ICP_TOOLS_TARGET=accelcomp

# Disable argument parameter checks
DISABLE_PARAM_CHECK=1

# Functions are not inlined in kernel
# Assumption CONFIG_ARCH_SUPPORTS_OPTIMIZED_INLINING is not set
ICP_DISABLE_INLINE=1

# Build

make \
    ICP_ROOT=${ICP_ROOT} \
    ICP_ARCH_USER=${ICP_ARCH_USER} \
    KERNEL_SOURCE_ROOT=${KERNEL_SOURCE_ROOT} \
    CROSS_COMPILE=${MERAKI_CROSS_COMPILE_STAGING_DIR}/bin/${CROSS_COMPILE}- \
    MERAKI_CROSS_COMPILE_STAGING_DIR=${MERAKI_CROSS_COMPILE_STAGING_DIR} \
    MACHINE=${MACHINE} \
    ICP_ENV_DIR=${ICP_ENV_DIR} \
    ICP_BUILD_OUTPUT=${ICP_BUILD_OUTPUT} \
    ICP_BUILDSYSTEM_PATH=${ICP_BUILDSYSTEM_PATH} \
    ICP_TOOLS_TARGET=${ICP_TOOLS_TARGET} \
    DISABLE_PARAM_CHECK=${DISABLE_PARAM_CHECK} \
    ICP_DISABLE_INLINE=${ICP_DISABLE_INLINE} \
    -C ${ICP_ROOT}/quickassist ${QATMUX_TARGET}
