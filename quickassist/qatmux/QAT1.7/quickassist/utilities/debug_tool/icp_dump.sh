#!/bin/bash

###############################################################################
#
#   BSD LICENSE
# 
#   Copyright(c) 2007-2018 Intel Corporation. All rights reserved.
#   All rights reserved.
# 
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     * Neither the name of Intel Corporation nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
# 
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
#  version: QAT1.7.L.4.6.0-00025
#
###############################################################################

ICP_DEBUG_DIR_PATH="$1"
GREP_RESULT=""
NUM_DEVICES="0"
CUR_DEVICE="0"
DATE=$(date '+%Hh_%Mm_%Ss_%dd_%mm_%yy')
SAL_LIB=libqat_s.so

devices=("dh895xcc" "c6xx" "c3xxx" "d15xx")
declare -A device_numbers=([dh895xcc]=0435 [dh895xccvf]=0443 [c6xx]=37c8 [c6xxvf]=37c9 [c3xxx]=19e2 [c3xxxvf]=19e3 [d15xx]=6f54 [d15xxvf]=6f55)
fw_files=("895xcc" "c62x" "c3xxx" "d15xx")

MapDeviceNumber()
{
    dev_id="$1"
    map_dev=$(
           case "$dev_id" in
               ("0435") echo "dh895xcc" ;;
               ("0443") echo "dh895xccvf" ;;
               ("37c8") echo "c6xx" ;;
               ("37c9") echo "c6xxvf" ;;
               ("6f54") echo "d15xx" ;;
               ("6f55") echo "d15xxvf" ;;
               ("19e2") echo "c3xxx" ;;
               ("19e3") echo "c3xxxvf" ;;
               (*) echo "$dev_id" ;;
           esac)
}

GetSkuQAT17()
{
    device_id="$1"
    FUSECTL_OFF=40;
    SOFTSTRAP_OFF=2e0;
    fusectl=$(lspci -xxxnd 8086:"$device_id" | grep -m1 "^$FUSECTL_OFF" | awk '{print $5 $4 $3 $2}')
    sofstrap=$(lspci -xxxxnd 8086:"$device_id" | grep -m1 "^$SOFTSTRAP_OFF" | awk '{print $16 $15 $14 $13}')
    echo "SKU=0x$fusectl-0x$sofstrap" >> "$ICP_DEBUG_DIR"/Sku_details.txt
}

WelcomeMessage()
{
    #Print Welcome Message
    echo -e "\n\n\t==============================================="
    echo -e "\tWelcome to Acceleration Dump Tool"
    echo -e "\t===============================================\n\n"
    if [ "$(id -u)" != "0" ]; then
        echo -e "\n\n\t==============================================="
        echo -e "\n\n\tERROR This script must be run as root"
        echo -e "\n\n\t===============================================\n\n"
        exit
    fi

    if [ -z "$ICP_DEBUG_DIR_PATH" ]; then
        echo -e "\n\n\t================================================================================="
        echo -e "\twarning! ICP_DEBUG_DIR_PATH is not set."
        echo -e "\tOutput defaults to current location $PWD"
        echo -e "\t-> to set the output path, run as"
        echo -e "\t./icp_dump <full path to location of where to store output of debug file>"
        echo -e "\t================================================================================="
        ICP_DEBUG_DIR_PATH=$PWD
    fi
    ICP_DEBUG_DIR="/tmp/ICP_debug"
    CONFIG_FILES="$ICP_DEBUG_DIR/config_files"
    FIRMWARE_FILES="$ICP_DEBUG_DIR/firmware_files"
    KERNEL_CONFIG_FILES="$ICP_DEBUG_DIR/Kernel_config_files"
}

CreateDebugFolder()
{
    if [ -z "$ICP_ROOT" ]; then
        echo -e "error ICP_ROOT is undefined. Please set the path \n\t\t\t
        -> export ICP_ROOT=<path to Install directory>\n\n"
        exit
    fi

   ICP_BUILD_OUTPUT="$ICP_ROOT"/build
   #Read in where to store output from debug script
   #================================
   if [ -e "$ICP_DEBUG_DIR" ]; then
   echo
        echo -e "\t $ICP_DEBUG_DIR Directory Already Exists"
        exit
   fi

   mkdir -p "$ICP_DEBUG_DIR"
}

GetSkuQAT16()
{
    # BDF format is <busNum>:<deviceNum>.<functionNum>
    busNum=$(echo "$bdf" | awk -F: '{print $2}')
    deviceNum=$(echo "$bdf" | awk -F: '{print $3}' | awk -F. '{print $1}')
    functionNum=$(echo "$bdf" | awk -F: '{print $3}' | awk -F. '{print $2}')
    dhSkubit=$(od -tx1 -Ax -j64 -N4 /proc/bus/pci/"$busNum"/"$deviceNum.$functionNum" | awk '{print $4}' | sed '/^$/d')
    dhSkubit=$((16#${dhSkubit}))
    case $dhSkubit in

        0)
            sku_no=1
            ;;

        16)
            sku_no=2
            ;;

        32)
            sku_no=3
            ;;

        48)
            sku_no=4
            ;;

        *)
            sku_no="error"
            echo "SKUError=$dhSkubit" >> "$ICP_DEBUG_DIR"/Sku_details.txt
            ;;
    esac
    echo "SKU=$sku_no" >> "$ICP_DEBUG_DIR"/Sku_details.txt

}

PrintSku()
{
    dev_str=$1
    device_id=$2

    if [ "$dev_str" = "dh895xcc" ]; then
        GetSkuQAT16
    elif [[ ("$dev_str" == "c6xx") || ("$dev_str" == "c3xxx") || ("$dev_str" == "d15xx") ]]; then
        GetSkuQAT17 "$device_id"
    fi
}

GetAccDeviceInfo()
{
    status=0

    echo "********** SKU Details **********" > "$ICP_DEBUG_DIR"/Sku_details.txt

    for device_number in "${device_numbers[@]}";
    do
        numOfDevice=$(lspci -vnd:$device_number | grep -c $device_number)
        if [ "$numOfDevice" != "0" ]; then
            MapDeviceNumber "$device_number"
            echo "Number of $map_dev devices=$numOfDevice">> "$ICP_DEBUG_DIR"/Sku_details.txt
        fi
    done
    for ((CUR_DEVICE=0; CUR_DEVICE<NUM_DEVICES; CUR_DEVICE++))
    do
        for device in "${devices[@]}";
        do
            GREP_RESULT=$(grep -E 'qat_dev'"$CUR_DEVICE " "$ICP_DEBUG_DIR"/adf_ctl_status.txt | grep -c $device)
            if [ "$GREP_RESULT" == "1" ]; then
                bdf=$(grep -E 'qat_dev'"$CUR_DEVICE " "$ICP_DEBUG_DIR"/adf_ctl_status.txt |awk '{print $10}' |cut -d ',' -f1)
                pfvf=$(grep -E 'qat_dev'"$CUR_DEVICE " "$ICP_DEBUG_DIR"/adf_ctl_status.txt |awk '{print $4}' |cut -d ',' -f1)
                echo -e "qat_dev$CUR_DEVICE is $pfvf" >> "$ICP_DEBUG_DIR"/Sku_details.txt
                echo "BDF=$bdf" >> "$ICP_DEBUG_DIR"/Sku_details.txt
                dev_id=${device_numbers[$pfvf]}
                PrintSku "$pfvf" "$dev_id"
                echo "" >> "$ICP_DEBUG_DIR"/Sku_details.txt
            fi
        done
    done
    return $status
}

GetDeviceMemoryInfo()
{
    {
    echo ""
    echo "df -h:"
    df -h
    echo ""
    echo "free:"
    free
    echo ""
    } > "$ICP_DEBUG_DIR"/memory.txt
}

RunDebugTool()
{
    #echo -e "\t Creating lspci.txt:"
    lspci -vvv > "$ICP_DEBUG_DIR"/lspci.txt
    lspci -tv >> "$ICP_DEBUG_DIR"/lspci.txt
    echo -e "\nQAT devices config space dump" >> "$ICP_DEBUG_DIR"/lspci.txt
    for device_number in "${device_numbers[@]}";
    do
        numOfDevice=$(lspci -vnd:$device_number | grep -c $device_number)
        if [ "$numOfDevice" != "0" ]; then
            lspci -xxxnd 8086:$device_number >> "$ICP_DEBUG_DIR"/lspci.txt
        fi
    done
    #echo -e "\t checking for hwloc installed in the system
    hash lstopo-no-graphics &>/dev/null
    if [ "$?" == "0" ]; then
        lstopo-no-graphics -v >"$ICP_DEBUG_DIR"/hwloc.txt
    fi
    #echo -e "\t Copying /sys/kernel/debug/qat* directories:"
    debug_fs_dirs=($(find /sys/kernel/debug/ -name "qat*" |grep -v "vf"))
    for debug_fs_dir in "${debug_fs_dirs[@]}";
    do
        cp -rf "$debug_fs_dir" "$ICP_DEBUG_DIR"
    done

    #echo -e "\t Creating config_files directory:"
    mkdir -p "$CONFIG_FILES"

    #echo -e "\t Creating firmware_files directory:"
    mkdir -p "$FIRMWARE_FILES"

    #echo -e "\t Copying config files and md5sums of firmware images that are used:"
    echo

    #echo -e "\t Creating $ICP_DEBUG_DIR/adf_ctl_status.txt"
    "$ICP_BUILD_OUTPUT"/adf_ctl status > "$ICP_DEBUG_DIR"/adf_ctl_status.txt
    #echo -e "\n"

    NUM_DEVICES=$(grep -c 'qat_dev' "$ICP_DEBUG_DIR"/adf_ctl_status.txt)

    #echo -e "\t Copying config files"

    devices_index=0
    for device in "${devices[@]}";
    do
        GREP_RESULT=$(grep -E 'qat_dev' "$ICP_DEBUG_DIR"/adf_ctl_status.txt | grep $device | grep -c 'up')
        if [ "$GREP_RESULT" != "0" ]; then
            configs=($(find /etc/ -maxdepth 1 -name "$device*"))
            for cfg_file in "${configs[@]}";
            do
                cp "$cfg_file" "$CONFIG_FILES/"
            done
            md5sum /lib/firmware/qat_${fw_files[$devices_index]}_mmp.bin > "$FIRMWARE_FILES"/qat_${fw_files[$devices_index]}_mmp_md5sum.txt
            md5sum /lib/firmware/qat_${fw_files[$devices_index]}.bin > "$FIRMWARE_FILES"/qat_${fw_files[$devices_index]}_md5sum.txt
        fi
        devices_index=$((devices_index+1))
    done

    #echo -e "\n\t Creating environment.txt:"
    env > "$ICP_DEBUG_DIR"/environment.txt

    #echo -e "\t Creating uname.txt:"
    uname -a > "$ICP_DEBUG_DIR"/uname.txt

    #echo -e "\t Creating messages.txt:"
    cat /var/log/messages > "$ICP_DEBUG_DIR"/messages.txt

    #echo -e "\t Creating Kernel_config_files directory:"
    mkdir -p "$KERNEL_CONFIG_FILES"

    #echo -e "\t Copying kernel configuration parameters:"
    cp /boot/config* "$KERNEL_CONFIG_FILES"

    #echo -e "\t Calling GetAccDeviceInfo"
    GetAccDeviceInfo

    #echo -e "\t Bios Version"
    dmidecode -t bios > "$ICP_DEBUG_DIR"/BIOS.txt

    #echo -e "\t Amount of memory on the system"
    GetDeviceMemoryInfo

    #echo -e "\t Creating cmdline.txt"
    cat /proc/cmdline > "$ICP_DEBUG_DIR"/cmdline.txt

    #echo -e "\t Creating kernel modules list"
    lsmod > "$ICP_DEBUG_DIR"/lsmod.txt

    #echo -e "\t Display SW version Information"
    version=`strings /usr/local/lib/$SAL_LIB | grep QAT_SOFTWARE_VERSION | head -1`
    echo "$version" > "$ICP_DEBUG_DIR"/qat_swversion.txt

    cd "$ICP_DEBUG_DIR"
    tar -czvf "$ICP_DEBUG_DIR_PATH/ICP_debug_$DATE".tar.gz *
    cd -
    echo -e "\n\n\t Tar ball of ICP_debug is located at $ICP_DEBUG_DIR_PATH/ICP_debug_$DATE.tar.gz
             \t Please forward this tar ball onto Intel"

    #echo -e "\n\t Removing $ICP_DEBUG_DIR"
    rm -rf "$ICP_DEBUG_DIR"
}

MainViewGeneral()
{

    #Print Disclaimer
    #================
    WelcomeMessage
    ORIG_UMASK=$(umask)
    #set permissions to create owner and group files only
    umask 7
    loop=0

    while [ $loop = 0 ];
    do
      if [ -z "$CL_INSTALL_OPTION" ]; then
        echo -e "\tPlease Accept so these files can be copied and forwarded to Intel for debug:"
        echo -e "\t\t* output of lspci -vvv"
        echo -e "\t\t* output of lstopo-no-graphics -v"
        echo -e "\t\t* /sys/kernel/debug/qat_*/* directories"
        echo -e "\t\t* output of adf_ctl status"
        echo -e "\t\t* config files /etc/*_dev*.conf"
        echo -e "\t\t* firmware md5sum"
        echo -e "\t\t* environment variables"
        echo -e "\t\t* output of uname -a"
        echo -e "\t\t* output of /var/log/messages"
        echo -e "\t\t* kernel config files"
        echo -e "\t\t* device sku"
        echo -e "\t\t* BIOS version"
        echo -e "\t\t* amount of Memory on the system"
        echo -e "\t\t* /proc/cmdline"
        echo -e "\t\t* kernel modules loaded"
        echo -e "\t\t* version of the qat driver installed"
        echo -e "\t\t* QAT software version"
        echo -e "\t -----------------------------------------"
        echo -e "\t yes to accept and run debug tool"
        echo -e "\t no to reject and exit debug tool"
        echo -e "\n"
        read case_var
      fi
    case $case_var in

      yes|YES|Yes)
        echo -e "\n\t****Running Debug Tool****>\n\n"
        CreateDebugFolder
        RunDebugTool
        echo -e "\n\n\n\t***Debug Tool Complete***>\n\n"
        exit
      ;;

      no|NO|No)
        echo -e "\n\t****Quiting Debug Tool****>\n\n"
        exit
      ;;

      *)
        echo -e "\n\t****Invalid Option****>"
        echo -e "\n\t****Please Choose yes or no****>\n\n"
      ;;
    esac

    #set umask back to original user setting
    umask "$ORIG_UMASK"
    done
}
MainViewGeneral
