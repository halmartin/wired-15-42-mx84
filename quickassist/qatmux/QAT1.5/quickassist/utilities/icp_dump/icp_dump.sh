#!/bin/sh

###############################################################################
#
# This file is provided under a dual BSD/GPLv2 license.  When using or 
#   redistributing this file, you may do so under either license.
# 
#   GPL LICENSE SUMMARY
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
#   BSD LICENSE 
# 
#   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
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
# 
#  version: QAT1.5.L.1.11.0-36
#
###############################################################################

ICP_DEBUG_DIR_PATH="$1"
ICP_DEBUG_DIR="$ICP_DEBUG_DIR_PATH/ICP_debug"
CONFIG_FILES="$ICP_DEBUG_DIR/config_files"
FIRMWARE_FILES="$ICP_DEBUG_DIR/firmware_files"
KERNEL_CONFIG_FILES="$ICP_DEBUG_DIR/Kernel_config_files"
GREP_RESULT=""
DH89xxcc_CONF_NUM="0"
C2xxx_CONF_NUM="0"
DH895xcc_CONF_NUM="0"
NUM_DEVICES="0"
CUR_DEVICE="0"
DATE=`date '+%Hh_%Mm_%Ss_%dd_%mm_%yy'`

DH895_DEVICE_NUMBER="0435"
DH895_DEVICE_NUMBER_VM="0443"
DH89x_DEVICE_NUMBER="0434"
DH89x_DEVICE_NUMBER_VM="0442" 
C2xxx_DEVICE_NUMBER="1f18"

WelcomeMessage()
{
    if [ "$(id -u)" != "0" ]; then
        echo -e "\n\n\t==============================================="
        echo -e "\n\n\tERROR This script must be run as root"
        echo -e "\n\n\t===============================================\n\n"
        exit
    fi

    if [ -z $ICP_DEBUG_DIR_PATH ]; then
        echo "        error location of where to store output of debug file not         
        passed in as an argument.
        Defaults to current location $PWD
        -> to set the output path run as 
        ./icp_dump <full path to location of where to store output of debug file>"
        ICP_DEBUG_DIR_PATH="$PWD"
        ICP_DEBUG_DIR="$ICP_DEBUG_DIR_PATH/ICP_debug"
        CONFIG_FILES="$ICP_DEBUG_DIR/config_files"
        FIRMWARE_FILES="$ICP_DEBUG_DIR/firmware_files"
        KERNEL_CONFIG_FILES="$ICP_DEBUG_DIR/Kernel_config_files"
    fi

   #Print Welcome Message
   #====================
   echo -e "\n\n\t==============================================="
   echo -e "\tWelcome to Acceleration Dump Tool"
   echo -e "\t===============================================\n\n"

}

CreateDebugFolder()
{
    if [ -z $ICP_BUILD_OUTPUT ]; then
        echo "error ICP_BUILD_OUTPUT is undefined. Please set the path \n\t\t\t
        -> export ICP_BUILD_OUTPUT=<path to Build output directory>\n\n"
        exit
    fi
    
   #Read in where to store output from debug script
   #================================
   if [ -e $ICP_DEBUG_DIR ]; then
   echo
        echo -e "\t This Directory Already Exists"
        echo -e "\t OverWriting!!!"
   else
   echo
        echo -e "\t Creating ICP_DEBUG_DIR : $ICP_DEBUG_DIR"
   fi

   mkdir -p "$ICP_DEBUG_DIR"

}
GetAccDeviceInfo()
{
    status=0
    if [ -n "`lspci -v | grep Co-pro | grep $DH895_DEVICE_NUMBER`" ]; then
        local dh895DeviceNumber=$DH895_DEVICE_NUMBER
    else
        local dh895DeviceNumber=$DH895_DEVICE_NUMBER_VM
    fi

    if [ -n "`lspci -v | grep Co-pro | grep $DH89x_DEVICE_NUMBER`" ]; then
        local dh89xDeviceNumber=$DH89x_DEVICE_NUMBER
    else
        local dh89xDeviceNumber=$DH89x_DEVICE_NUMBER_VM
    fi

    local c2xxxDeviceNumber="$C2xxx_DEVICE_NUMBER"

    local CUR_DH895xcc=0
    local CUR_DH89xxcc=0
    local CUR_C2xxxcc=0

    
    echo "" > $ICP_DEBUG_DIR/Sku_details.txt
    cp $ICP_DEBUG_DIR/Sku_details.txt $ICP_DEBUG_DIR/Sku_details_temp1.txt

    numDh895xDevice=`lspci -vnd:$dh895DeviceNumber | grep $dh895DeviceNumber | wc -l`
    if [ $numDh895xDevice != "0" ]; then
        echo "Number of Dh895xcc devices=$numDh895xDevice"> $ICP_DEBUG_DIR/Sku_details_temp2.txt
        cat $ICP_DEBUG_DIR/Sku_details_temp1.txt $ICP_DEBUG_DIR/Sku_details_temp2.txt > $ICP_DEBUG_DIR/Sku_details.txt
        cp $ICP_DEBUG_DIR/Sku_details.txt $ICP_DEBUG_DIR/Sku_details_temp1.txt
    fi

    numDh89xxDevice=`lspci -vnd:$dh89xDeviceNumber | grep $dh89xDeviceNumber | wc -l`
    if [ $numDh89xxDevice != "0" ]; then
        echo "Number of Dh89xxcc devices=$numDh89xxDevice" > $ICP_DEBUG_DIR/Sku_details_temp2.txt
        cat $ICP_DEBUG_DIR/Sku_details_temp1.txt $ICP_DEBUG_DIR/Sku_details_temp2.txt > $ICP_DEBUG_DIR/Sku_details.txt
        cp $ICP_DEBUG_DIR/Sku_details.txt $ICP_DEBUG_DIR/Sku_details_temp1.txt
    fi

    numC2xxxDevice=`lspci -vnd:$c2xxxDeviceNumber | grep $c2xxxDeviceNumber | wc -l`
    if [ $numC2xxxDevice != "0" ]; then
        echo "Number of C2xxx devices=$numC2xxxDevice" > $ICP_DEBUG_DIR/Sku_details_temp2.txt
        cat $ICP_DEBUG_DIR/Sku_details_temp1.txt $ICP_DEBUG_DIR/Sku_details_temp2.txt > $ICP_DEBUG_DIR/Sku_details.txt
        cp $ICP_DEBUG_DIR/Sku_details.txt $ICP_DEBUG_DIR/Sku_details_temp1.txt
    fi


    for ((CUR_DEVICE=0; CUR_DEVICE<NUM_DEVICES; CUR_DEVICE++))
    do
        echo "" > $ICP_DEBUG_DIR/Sku_details_temp2.txt
        cat $ICP_DEBUG_DIR/Sku_details_temp1.txt $ICP_DEBUG_DIR/Sku_details_temp2.txt > $ICP_DEBUG_DIR/Sku_details.txt
        cp $ICP_DEBUG_DIR/Sku_details.txt $ICP_DEBUG_DIR/Sku_details_temp1.txt
        GREP_RESULT=`grep -c 'icp_dev'"$CUR_DEVICE"' - type=dh895xcc.*, ' $ICP_DEBUG_DIR/adf_ctl_status.txt`
        if [ $GREP_RESULT == 1 ]; then            
            j=$(($CUR_DH895xcc + 1))
            bdf=`lspci -vnd:$dh895DeviceNumber | grep $dh895DeviceNumber  | awk '{print $1}' | sed -n ''$j'p'`

            cat /proc/icp_dh895xcc_dev$CUR_DH895xcc/version | grep "Hardware Version" > $ICP_DEBUG_DIR/sku.txt
            cat $ICP_DEBUG_DIR/sku.txt | sed 's/^...............................//' > $ICP_DEBUG_DIR/sku2.txt

            echo -e "icp_dev$CUR_DEVICE is DH895xcc" > $ICP_DEBUG_DIR/Sku_details_temp2.txt
            cat $ICP_DEBUG_DIR/Sku_details_temp1.txt $ICP_DEBUG_DIR/Sku_details_temp2.txt > $ICP_DEBUG_DIR/Sku_details.txt
            cp $ICP_DEBUG_DIR/Sku_details.txt $ICP_DEBUG_DIR/Sku_details_temp1.txt

            cat $ICP_DEBUG_DIR/Sku_details_temp1.txt $ICP_DEBUG_DIR/sku2.txt > $ICP_DEBUG_DIR/Sku_details.txt
            cp $ICP_DEBUG_DIR/Sku_details.txt $ICP_DEBUG_DIR/Sku_details_temp1.txt

            echo "BDF=$bdf" > $ICP_DEBUG_DIR/Sku_details_temp2.txt
            cat $ICP_DEBUG_DIR/Sku_details_temp1.txt $ICP_DEBUG_DIR/Sku_details_temp2.txt > $ICP_DEBUG_DIR/Sku_details.txt
            cp $ICP_DEBUG_DIR/Sku_details.txt $ICP_DEBUG_DIR/Sku_details_temp1.txt
            CUR_DH895xcc=`expr $CUR_DH895xcc + 1`
        else
            GREP_RESULT=`grep -c 'icp_dev'"$CUR_DEVICE"' - type=dh89xxcc.*, ' $ICP_DEBUG_DIR/adf_ctl_status.txt`
            if [ $GREP_RESULT == 1 ]; then
                j=$(($CUR_DH89xxcc + 1))
                bdf=`lspci -vnd:$dh89xDeviceNumber | grep $dh89xDeviceNumber  | awk '{print $1}' | sed -n ''$j'p'`
                cat /proc/icp_dh89xxcc_dev$CUR_DH89xxcc/version | grep "Hardware Version" > $ICP_DEBUG_DIR/sku.txt
                cat $ICP_DEBUG_DIR/sku.txt | sed 's/^...............................//' > $ICP_DEBUG_DIR/sku2.txt
                
                echo -e "icp_dev$CUR_DEVICE is DH89xxcc" > $ICP_DEBUG_DIR/Sku_details_temp2.txt
                cat $ICP_DEBUG_DIR/Sku_details_temp1.txt $ICP_DEBUG_DIR/Sku_details_temp2.txt > $ICP_DEBUG_DIR/Sku_details.txt
                cp $ICP_DEBUG_DIR/Sku_details.txt $ICP_DEBUG_DIR/Sku_details_temp1.txt
                
                cat $ICP_DEBUG_DIR/Sku_details_temp1.txt $ICP_DEBUG_DIR/sku2.txt > $ICP_DEBUG_DIR/Sku_details.txt
                cp $ICP_DEBUG_DIR/Sku_details.txt $ICP_DEBUG_DIR/Sku_details_temp1.txt
                
                echo "BDF=$bdf" > $ICP_DEBUG_DIR/Sku_details_temp2.txt
                cat $ICP_DEBUG_DIR/Sku_details_temp1.txt $ICP_DEBUG_DIR/Sku_details_temp2.txt > $ICP_DEBUG_DIR/Sku_details.txt
                cp $ICP_DEBUG_DIR/Sku_details.txt $ICP_DEBUG_DIR/Sku_details_temp1.txt
                CUR_DH89xxcc=`expr $CUR_DH89xxcc + 1`
            else 
                GREP_RESULT=`grep -c 'icp_dev'"$CUR_DEVICE"' - type=c2xxx.*, ' $ICP_DEBUG_DIR/adf_ctl_status.txt`
                if [ $GREP_RESULT == 1 ]; then
                    j=$(($CUR_C2xxxcc + 1))
                    bdf=`lspci -vnd:$c2xxxDeviceNumber | grep $c2xxxDeviceNumber  | awk '{print $1}' | sed -n ''$j'p'`

                    cat /proc/icp_c2xxx_dev$CUR_C2xxxcc/version | grep "Hardware Version" > $ICP_DEBUG_DIR/sku.txt
                    cat $ICP_DEBUG_DIR/sku.txt | sed 's/^...............................//' > $ICP_DEBUG_DIR/sku2.txt
                    
                    echo -e "icp_dev$CUR_DEVICE is C2xxxcc" > $ICP_DEBUG_DIR/Sku_details_temp2.txt
                    cat $ICP_DEBUG_DIR/Sku_details_temp1.txt $ICP_DEBUG_DIR/Sku_details_temp2.txt > $ICP_DEBUG_DIR/Sku_details.txt
                    cp $ICP_DEBUG_DIR/Sku_details.txt $ICP_DEBUG_DIR/Sku_details_temp1.txt
                    
                    cat $ICP_DEBUG_DIR/Sku_details_temp1.txt $ICP_DEBUG_DIR/sku2.txt > $ICP_DEBUG_DIR/Sku_details.txt
                    cp $ICP_DEBUG_DIR/Sku_details.txt $ICP_DEBUG_DIR/Sku_details_temp1.txt
                    
                    echo "BDF=$bdf" > $ICP_DEBUG_DIR/Sku_details_temp2.txt
                    cat $ICP_DEBUG_DIR/Sku_details_temp1.txt $ICP_DEBUG_DIR/Sku_details_temp2.txt > $ICP_DEBUG_DIR/Sku_details.txt
                    cp $ICP_DEBUG_DIR/Sku_details.txt $ICP_DEBUG_DIR/Sku_details_temp1.txt
                    CUR_C2xxxcc=`expr $CUR_C2xxxcc + 1`
                else
                    echo -e "\n\n\t\t Driver not installed, please install the driver and run this script again. \t\t\n\n"
                fi
            fi
        fi
    done

    rm -rf $ICP_DEBUG_DIR/sku.txt
    rm -rf $ICP_DEBUG_DIR/sku2.txt
    rm -rf $ICP_DEBUG_DIR/Sku_details_temp1.txt
    rm -rf $ICP_DEBUG_DIR/Sku_details_temp2.txt
    return $status
}

GetDeviceMemoryInfo()
{
    echo "" > $ICP_DEBUG_DIR/memory.txt
    cp $ICP_DEBUG_DIR/memory.txt $ICP_DEBUG_DIR/memory_temp1.txt

    echo "df -h:" > $ICP_DEBUG_DIR/memory_temp2.txt
    cat $ICP_DEBUG_DIR/memory_temp1.txt $ICP_DEBUG_DIR/memory_temp2.txt > $ICP_DEBUG_DIR/memory.txt
    cp $ICP_DEBUG_DIR/memory.txt $ICP_DEBUG_DIR/memory_temp1.txt
    df -h > $ICP_DEBUG_DIR/memory_temp2.txt
    cat $ICP_DEBUG_DIR/memory_temp1.txt $ICP_DEBUG_DIR/memory_temp2.txt > $ICP_DEBUG_DIR/memory.txt
    cp $ICP_DEBUG_DIR/memory.txt $ICP_DEBUG_DIR/memory_temp1.txt
    
    echo "" > $ICP_DEBUG_DIR/memory_temp2.txt
    cat $ICP_DEBUG_DIR/memory_temp1.txt $ICP_DEBUG_DIR/memory_temp2.txt > $ICP_DEBUG_DIR/memory.txt
    cp $ICP_DEBUG_DIR/memory.txt $ICP_DEBUG_DIR/memory_temp1.txt
    echo "" > $ICP_DEBUG_DIR/memory_temp2.txt
    cat $ICP_DEBUG_DIR/memory_temp1.txt $ICP_DEBUG_DIR/memory_temp2.txt > $ICP_DEBUG_DIR/memory.txt
    cp $ICP_DEBUG_DIR/memory.txt $ICP_DEBUG_DIR/memory_temp1.txt

    echo "free:" > $ICP_DEBUG_DIR/memory_temp2.txt
    cat $ICP_DEBUG_DIR/memory_temp1.txt $ICP_DEBUG_DIR/memory_temp2.txt > $ICP_DEBUG_DIR/memory.txt
    cp $ICP_DEBUG_DIR/memory.txt $ICP_DEBUG_DIR/memory_temp1.txt
    free > $ICP_DEBUG_DIR/memory_temp2.txt
    cat $ICP_DEBUG_DIR/memory_temp1.txt $ICP_DEBUG_DIR/memory_temp2.txt > $ICP_DEBUG_DIR/memory.txt
    cp $ICP_DEBUG_DIR/memory.txt $ICP_DEBUG_DIR/memory_temp1.txt
    echo "" > $ICP_DEBUG_DIR/memory_temp2.txt
    cat $ICP_DEBUG_DIR/memory_temp1.txt $ICP_DEBUG_DIR/memory_temp2.txt > $ICP_DEBUG_DIR/memory.txt
    
    rm -rf $ICP_DEBUG_DIR/memory_temp1.txt
    rm -rf $ICP_DEBUG_DIR/memory_temp2.txt
}

RunDebugTool()
{
    #echo -e "\t Creating lspci.txt:"
    lspci -vvv > $ICP_DEBUG_DIR/lspci.txt
    lspci -tv >> $ICP_DEBUG_DIR/lspci.txt

    #echo -e "\t Copying /proc/icp_*_dev* directories:"
    cp -rf /proc/icp_*_dev* $ICP_DEBUG_DIR

    #echo -e "\t Creating config_files directory:"
    mkdir -p "$CONFIG_FILES"

    #echo -e "\t Creating firmware_files directory:"
    mkdir -p "$FIRMWARE_FILES"

    #echo -e "\t Copying config files and md5sums of firmware images that are used:"
    echo

    #echo -e "\t Creating $ICP_DEBUG_DIR/adf_ctl_status.txt"
    $ICP_BUILD_OUTPUT/adf_ctl status > $ICP_DEBUG_DIR/adf_ctl_status.txt
    #cat $ICP_DEBUG_DIR/adf_ctl_status.txt
    #echo -e "\n"

    NUM_DEVICES=`grep -c 'icp_dev.' $ICP_DEBUG_DIR/adf_ctl_status.txt`

    #echo -e "\t Copying config files"

    for ((CUR_DEVICE=0; CUR_DEVICE<NUM_DEVICES; CUR_DEVICE++))
    do
        GREP_RESULT=`grep -c 'icp_dev'"$CUR_DEVICE"' - type=dh89xxcc.*, state=up' $ICP_DEBUG_DIR/adf_ctl_status.txt`
        if [ $GREP_RESULT == 1 ]; then
            #echo -e "\t cp /etc/dh89xxcc_qa_dev$DH89xxcc_CONF_NUM.conf $CONFIG_FILES"
            cp /etc/dh89xxcc_qa_dev$DH89xxcc_CONF_NUM.conf $CONFIG_FILES/icp_dev"$CUR_DEVICE"_dh89xxcc_qa_dev$DH89xxcc_CONF_NUM.conf
            md5sum /lib/firmware/mmp_firmware.bin > $FIRMWARE_FILES/icp_dev"$CUR_DEVICE"_dh89xxcc_mmp_md5sum.txt
            md5sum /lib/firmware/mof_firmware.bin > $FIRMWARE_FILES/icp_dev"$CUR_DEVICE"_dh89xxcc_mof_md5sum.txt
            DH89xxcc_CONF_NUM=`expr $DH89xxcc_CONF_NUM + 1`
        else 
            GREP_RESULT=`grep -c 'icp_dev'"$CUR_DEVICE"' - type=c2xxx.*, state=up' $ICP_DEBUG_DIR/adf_ctl_status.txt`
            if [ $GREP_RESULT == 1 ]; then
                #echo -e "\t cp /etc/c2xxx_qa_dev$C2xxx_CONF_NUM.conf $CONFIG_FILES"
                cp /etc/c2xxx_qa_dev$C2xxx_CONF_NUM.conf $CONFIG_FILES/icp_dev"$CUR_DEVICE"_c2xxx_qa_dev$C2xxx_CONF_NUM.conf
                md5sum /lib/firmware/mmp_firmware_c2xxx.bin > $FIRMWARE_FILES/icp_dev"$CUR_DEVICE"_c2xxx_mmp_md5sum.txt
                md5sum /lib/firmware/mof_firmware_c2xxx.bin > $FIRMWARE_FILES/icp_dev"$CUR_DEVICE"_c2xxx_mof_md5sum.txt
                C2xxx_CONF_NUM=`expr $C2xxx_CONF_NUM + 1`
            else
                GREP_RESULT=`grep -c 'icp_dev'"$CUR_DEVICE"' - type=dh895xcc.*, state=up' $ICP_DEBUG_DIR/adf_ctl_status.txt`
                if [ $GREP_RESULT == 1 ]; then            
                    #echo -e "\t cp /etc/dh895xcc_qa_dev$DH895xcc_CONF_NUM.conf $CONFIG_FILES"
                    cp /etc/dh895xcc_qa_dev$DH895xcc_CONF_NUM.conf $CONFIG_FILES/icp_dev"$CUR_DEVICE"_dh895xcc_qa_dev$DH895xcc_CONF_NUM.conf
                    md5sum /lib/firmware/dh895xcc/mmp_firmware.bin > $FIRMWARE_FILES/icp_dev"$CUR_DEVICE"_dh895xcc_mmp_md5sum.txt
                    md5sum /lib/firmware/dh895xcc/mof_firmware.bin > $FIRMWARE_FILES/icp_dev"$CUR_DEVICE"_dh895xcc_mof_md5sum.txt
                    DH895xcc_CONF_NUM=`expr $DH895xcc_CONF_NUM + 1`
                else
                    GREP_RESULT=`grep -c 'icp_dev'"$CUR_DEVICE"' - type=dh89xxcc.*, state=down' $ICP_DEBUG_DIR/adf_ctl_status.txt`
                    if [ $GREP_RESULT == 1 ]; then 
                        #echo -e "\ticp_dev$CUR_DEVICE is down and type=dh89xxcc"
                        DH89xxcc_CONF_NUM=`expr $DH89xxcc_CONF_NUM + 1`
                    else
                        GREP_RESULT=`grep -c 'icp_dev'"$CUR_DEVICE"' - type=c2xxx.*, state=down' $ICP_DEBUG_DIR/adf_ctl_status.txt`
                        if [ $GREP_RESULT == 1 ]; then 
                            #echo -e "\ticp_dev$CUR_DEVICE is down and type=c2xxx"
                            C2xxx_CONF_NUM=`expr $C2xxx_CONF_NUM + 1`
                        else
                            GREP_RESULT=`grep -c 'icp_dev'"$CUR_DEVICE"' - type=dh895xcc.*, state=down' $ICP_DEBUG_DIR/adf_ctl_status.txt`
                            if [ $GREP_RESULT == 1 ]; then 
                                #echo -e "\ticp_dev$CUR_DEVICE is down and type=dh895xcc"
                                DH895xcc_CONF_NUM=`expr $DH895xcc_CONF_NUM + 1`
                            else
                                echo -e "\n\n\t\t Driver not installed, please install the driver and run this script again. \t\t\n\n"
                            fi
                        fi
                    fi
                fi
            fi
        fi
    done

    #echo -e "\n\t Creating environment.txt:"
    env > $ICP_DEBUG_DIR/environment.txt

    #echo -e "\t Creating uname.txt:"
    uname -a > $ICP_DEBUG_DIR/uname.txt

    #echo -e "\t Creating messages.txt:"
    cat /var/log/messages > $ICP_DEBUG_DIR/messages.txt

    #echo -e "\t Creating Kernel_config_files directory:"
    mkdir -p "$KERNEL_CONFIG_FILES"

    #echo -e "\t Copying kernel configuration parameters:"
    cp /boot/config* $KERNEL_CONFIG_FILES

    #echo -e "\t Calling GetAccDeviceInfo"
    GetAccDeviceInfo
    
    #echo -e "\t Bios Version"
    dmidecode -t bios > $ICP_DEBUG_DIR/BIOS.txt

    #echo -e "\t Amount of memory on the system"
    GetDeviceMemoryInfo
    
    #echo -e "\t Creating cmdline.txt"
    cat /proc/cmdline > $ICP_DEBUG_DIR/cmdline.txt

    #echo -e "\t Creating kernel modules list"
    lsmod > $ICP_DEBUG_DIR/lsmod.txt

    #echo -e "\t Creating tar ball of ICP_debug folder"
    cd $ICP_DEBUG_DIR_PATH
    tar -czvf $ICP_DEBUG_DIR_PATH/ICP_debug_$DATE.tar.gz ICP_debug
    cd -
    echo -e "\n\n\t Tar ball of ICP_debug is located at $ICP_DEBUG_DIR_PATH/ICP_debug_$DATE.tar.gz
             \t Please forward this tar ball onto Intel"

    #echo -e "\n\t Removing $ICP_DEBUG_DIR"
    rm -rf $ICP_DEBUG_DIR 
}

MainViewGeneral()
{

    #Print Disclaimer
    #================
    WelcomeMessage
    #set permissions to create owner and group files only
    umask 7
    loop=0

    while [ $loop = 0 ];
    do
      if [ -z $CL_INSTALL_OPTION ]; then
        #Switch Statement to Select Installing Option
        #============================================
        echo -e "\tPlease Accept so these files can be copied and forwarded to Intel for debug :"
        echo -e "\t\tThe output of lspci -vvv"
        echo -e "\t\t/proc/icp_*_dev* directories"
        echo -e "\t\tThe config files that are used /etc/*_qa_dev*.conf"
        echo -e "\t\tThe environment used"
        echo -e "\t\tThe output of uname -a"
        echo -e "\t\tThe output of /var/log/messages"
        echo -e "\t\tThe kernel config files"
        echo -e "\t\tMd5Sum of the firmware used"
        echo -e "\t\tSku version of device"
        echo -e "\t\tBios version of device"
        echo -e "\t\tAmount of Memory on the device"
        echo -e "\t\tThe contents of /proc/cmdline"
        echo -e "\t ----------------------------"
        echo -e "\t yes to accept and run debug tool"
        echo -e "\t no to reject and exit debug tool"
        echo -e "\n"
        read case
      fi
    case $case in

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
    umask $ORIG_UMASK
    done
}
MainViewGeneral
