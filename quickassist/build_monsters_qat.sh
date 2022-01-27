#!/bin/bash

function usage() {
  echo "Invalid arguments. Usage: Build QAT1.7 -r <icp_root> -c <cross_compiler> -k <kernel_source_root> -o <icp_build_output> -h <host_arch>"
  exit 1
}

function echo_args() {
  echo "Here are your variables"
  echo "ICP_ROOT=${ICP_ROOT}"
  echo "TOOLCHAIN=${TOOLCHAIN}"
  echo "KERNEL_SOURCE_ROOT=${KERNEL_SOURCE_ROOT}"
  echo "ICP_BUILD_OUTPUT=${ICP_BUILD_OUTPUT}"
  echo "HOST_ARCH=${HOST_ARCH}"
}

set -e #exit on any error

ICP_ROOT=
TOOLCHAIN=
KERNEL_SOURCE_ROOT=
ICP_BUILD_OUTPUT=
HOST_ARCH=x86_64-linux
BUILD_MACHINE=

while getopts "r:c:k:o:a" opt; do 
  case "${opt}" in
    r)
      ICP_ROOT=${OPTARG}
      ;;
    c)
      TOOLCHAIN=${OPTARG}
      ;;
    k)
      KERNEL_SOURCE_ROOT=${OPTARG}
      ;;
    o)
      ICP_BUILD_OUTPUT=${OPTARG}
      ;;
    a)
      HOST_ARCH=${OPTARG}
      ;;
    *)
      usage
      ;;
  esac
done
shift $((OPTIND-1))

echo_args

[ -z "${ICP_ROOT}" ] && usage
[ -z "${TOOLCHAIN}" ] && usage
[ -z "${KERNEL_SOURCE_ROOT}" ] && usage
[ -z "${ICP_BUILD_OUTPUT}" ] && usage
[ -z "${HOST_ARCH}" ] && usage

#Intel required variables
ICP_ENV_DIR="$ICP_ROOT/quickassist/build_system/build_files/env_files"
ICP_BUILDSYSTEM_PATH="$ICP_ROOT/quickassist/build_system"
BUILD_MACHINE="$(arch)-$(uname -r)" #get local build machine's 
QAT_DEST_DIR="${KERNEL_SOURCE_ROOT}/drivers/crypto/qat/" 

#if the linux in-tree drivers already exist
if [ -d "$QAT_DEST_DIR" ]; then
    rm -r "$QAT_DEST_DIR" #get rid of them
    mkdir -p "$QAT_DEST_DIR"
fi



export PATH=$PATH:${TOOLCHAIN} # ask if this is okay to do
(cd "$ICP_ROOT" && ./configure --build ${BUILD_MACHINE} --host ${HOST_ARCH} KERNEL_SOURCE_ROOT="$KERNEL_SOURCE_ROOT")
(cd "$ICP_ROOT/quickassist" && make lac_kernel ICP_ROOT="$ICP_ROOT" ICP_ENV_DIR="$ICP_ENV_DIR" ICP_BUILD_OUTPUT="$ICP_BUILD_OUTPUT" ICP_BUILDSYSTEM_PATH="$ICP_BUILDSYSTEM_PATH" KERNEL_SOURCE_ROOT="$KERNEL_SOURCE_ROOT")
(cd "$ICP_ROOT" && make qat-driver-all adf-ctl-all qat-service-all ICP_ROOT="$ICP_ROOT" ICP_ENV_DIR="$ICP_ENV_DIR" ICP_BUILD_OUTPUT="$ICP_BUILD_OUTPUT" ICP_BUILDSYSTEM_PATH="$ICP_BUILDSYSTEM_PATH" KERNEL_SOURCE_ROOT="$KERNEL_SOURCE_ROOT")

echo "Done building!!"
echo "Creating Directories"
mkdir -p "$QAT_DEST_DIR/qat_common/"
mkdir -p "$QAT_DEST_DIR/qat_dh895xcc/"
mkdir -p "$QAT_DEST_DIR/qat_d15xx/"

echo "Copying files to Build directory"
find "$ICP_ROOT/quickassist/qat/drivers/crypto/qat/qat_d15xx" -iname "*.o" -exec cp {} "$QAT_DEST_DIR/qat_d15xx/" \;
find "$ICP_ROOT/quickassist/qat/drivers/crypto/qat/qat_d15xx" -iname "*.ko" -exec cp {} "$QAT_DEST_DIR/qat_d15xx/" \;
cp "$ICP_ROOT/quickassist/qat/drivers/crypto/qat/qat_d15xx/qat_d15xx.mod.c" "$QAT_DEST_DIR/qat_d15xx/"

find "$ICP_ROOT/quickassist/qat/drivers/crypto/qat/qat_dh895xcc" -iname "*.o" -exec cp {} "$QAT_DEST_DIR/qat_dh895xcc/" \;
find "$ICP_ROOT/quickassist/qat/drivers/crypto/qat/qat_dh895xcc" -iname "*.ko" -exec cp {} "$QAT_DEST_DIR/qat_dh895xcc/" \;
cp "$ICP_ROOT/quickassist/qat/drivers/crypto/qat/qat_dh895xcc/qat_dh895xcc.mod.c" "$QAT_DEST_DIR/qat_dh895xcc/"

find "$ICP_ROOT/quickassist/qat/drivers/crypto/qat/qat_common" -iname "*.o" -exec cp {} "$QAT_DEST_DIR/qat_common/" \;
find "$ICP_ROOT/quickassist/qat/drivers/crypto/qat/qat_common" -iname "*.ko" -exec cp {} "$QAT_DEST_DIR/qat_common/" \;
cp "$ICP_ROOT/quickassist/qat/drivers/crypto/qat/qat_common/qat_rsaprivkey-asn1.c" "$QAT_DEST_DIR/qat_common/"
cp "$ICP_ROOT/quickassist/qat/drivers/crypto/qat/qat_common/qat_rsaprivkey-asn1.h" "$QAT_DEST_DIR/qat_common/"
cp "$ICP_ROOT/quickassist/qat/drivers/crypto/qat/qat_common/qat_rsapubkey-asn1.c" "$QAT_DEST_DIR/qat_common/"
cp "$ICP_ROOT/quickassist/qat/drivers/crypto/qat/qat_common/qat_rsapubkey-asn1.h" "$QAT_DEST_DIR/qat_common/"

cp "$ICP_BUILD_OUTPUT/usdm_drv.ko" "$QAT_DEST_DIR/"
cp "$ICP_BUILD_OUTPUT/qat_api.ko" "$QAT_DEST_DIR/"