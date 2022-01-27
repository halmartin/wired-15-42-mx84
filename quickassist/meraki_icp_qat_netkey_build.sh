#!/usr/bin/env bash

function usage() {
    echo "Invalid arguments. Usage: meraki_icp_qat_netkey_build -k <kernel_source_root> -o <output_dir> -r <icp_root> [-t]"
    exit 1
}

function echo_args() {
    echo "ICP_ROOT=${ICP_ROOT}"
    echo "KERNEL_SOURCE_ROOT=${KERNEL_SOURCE_ROOT}"
    echo "ICP_BUILD_OUTPUT=${ICP_BUILD_OUTPUT}"
}

set -e # exit on any error

ICP_ROOT=
KERNEL_SOURCE_ROOT=
ICP_BUILD_OUTPUT=
ICP_BUILD_TEST=

while getopts "k:o:r:t" opt; do
    case "${opt}" in
        k)
            KERNEL_SOURCE_ROOT=${OPTARG}
            ;;
        o)
            ICP_BUILD_OUTPUT=${OPTARG}
            ;;
        r)
            ICP_ROOT=${OPTARG}
            ;;
        t)
            ICP_BUILD_TEST=1
            ;;
        *)
            usage
            ;;
    esac
done
shift $((OPTIND-1))

[ -z "${KERNEL_SOURCE_ROOT}" ] && usage
[ -z "${ICP_ROOT}" ] && usage
[ -z "${ICP_BUILD_OUTPUT}" ] && usage

echo "***** meraki_icp_qat_netkey_build: Making *****" && echo_args

# Build

cd ${ICP_ROOT}/../../icp_qat_netkey/icp_netkey && \
    make \
        KERNEL_SOURCE_ROOT=${KERNEL_SOURCE_ROOT} \
        ICP_ROOT=${ICP_ROOT}
set +e && mkdir -pv ${ICP_BUILD_OUTPUT} && set -e
mv -fv ${ICP_ROOT}/../../icp_qat_netkey/icp_netkey/icp_qat_netkey.ko ${ICP_BUILD_OUTPUT}/icp_qat_netkey.ko

if [[ $ICP_BUILD_TEST == 1 ]]; then

    cd ${ICP_ROOT}/../../icp_qat_netkey/icp_netkey/test && make KERNEL_SOURCE_ROOT=${KERNEL_SOURCE_ROOT}
    mv -fv ${ICP_ROOT}/../../icp_qat_netkey/icp_netkey/test/icp_perf_aead.ko ${ICP_BUILD_OUTPUT}/icp_perf_aead.ko

fi
