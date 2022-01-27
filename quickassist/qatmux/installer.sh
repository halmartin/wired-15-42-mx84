#!/bin/bash
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
#  version: QATmux.L.2.6.0-60
#
###############################################################################
#Script Default Locations
#========================
INSTALL_DEFAULT="$PWD"
INSTALL_PATH="$PWD"
ICP_ROOT="$PWD"
BUILD_OUTPUT_DEFAULT="build"
ICP_TOOLS_TARGET="accelcomp"
KERNEL_SOURCE_ROOT_DEFAULT="/usr/src/kernels/`\uname -r`"
OS="`\uname -m`"
ORIG_UMASK=`umask`
MODULE_DIR="/lib/modules/`\uname -r`/kernel/drivers"
OBJ_LIST="icp_qa_al.ko"
VF_OBJ_LIST="icp_qa_al_vf.ko"
BIN_LIST_QAT1_6="mof_firmware.bin mmp_firmware.bin"
BIN_LIST_QAT1_5="mof_firmware.bin mmp_firmware.bin
                 mof_firmware_c2xxx.bin mmp_firmware_c2xxx.bin"
VF_BIN_LIST="mmp_firmware.bin"
ERR_COUNT=0
DH895_DEVICE_NUMBER="0435"
DH895_DEVICE_NUMBER_VM="0443"
DH89X_DEVICE_NUMBER="0434"
DH89X_DEVICE_NUMBER_VM="0442"
C2XXX_DEVICE_NUMBER="1f18"
USE_WATCHDOG=false
QAT16="QAT1.6"
QAT15="QAT1.5"

#Set tar command options
#=======================
TAR_ADDL_OPTS=""
TAR_VERSION=`tar --version 2>&1 | grep "GNU"`
if [ $? -eq 0 ] ; then
    TAR_ADDL_OPTS="--no-same-permissions"
fi

#Copy Command not interactive
##################
alias cp='\cp'

#Configuration Options 
#=========================
# Driver Options
DRIVER_OPTION_AUTO="auto"
DRIVER_OPTION_MUX="mux"
DRIVER_OPTION_QAT1_5="QAT1.5"
DRIVER_OPTION_QAT1_6="QAT1.6"
DRIVER_OPTION_QAT1_6_MUX="QAT1.6_mux"

#SR-IOV Options
SRIOV_MODE_NONE="novirt"
SRIOV_MODE_HOST="host"
SRIOV_MODE_GUEST="guest"

# Build Options
BUILD_ACCEL_SAMPLE="Acceleration and Sample Code"
BUILD_ACCEL="Acceleration"
BUILD_SAMPLE="Sample Code"
BUILD_DC_ONLY=false
# Action options
PERFORM_ACTION_BUILD="build"
PERFORM_ACTION_CLEAN_BUILD="clean"
PERFORM_ACTION_INSTALL="install"
PERFORM_ACTION_UNINSTALL="uninstall"
PERFORM_ACTION_BUILD_ACCEL="build_accel"
PERFORM_ACTION_BUILD_SAMPLE="build_sample"
PERFORM_ACTION_ACCEL_INFO="show_accel_info"
PERFORM_ACTION_PKG_DEPENDENCY_LIST="dependency_list"
PERFORM_ACTION_HELP="help"
PERFORM_ACTION_VERSION="version"
#Service Options
SERVICE_NONE="none"
SERVICE_DC_ONLY="build_dc_only"
SERVICE_GIGE="gige"
SERVICE_BUILD_ACCEL="build_accel"
SERVICE_BUILD_SAMPLE="build_sample"
# Setting the defaults
SriovOption=$SRIOV_MODE_NONE
ServiceOption=$SERVICE_NONE
DriverOption=$DRIVER_OPTION_AUTO
BuildTarget=$BUILD_ACCEL_SAMPLE
NumQAT15Device=0
NumQAT16Devices=0
INSTALL_LOCATION=$INSTALL_DEFAULT
InstallAccelSriovArgument=0 # This value would be passed to InstallAccel/InstallAccelDh89xxcc functions
InstallerOperationStatus=0 # Installer operation return value

# Handling Command Line Arguments
#================================
CL_OPTIONS_COUNT=$#

# Validate the number of arguments
if (( $CL_OPTIONS_COUNT > "5" )); then 
    echo -e "\n Not more than 5 arguments accepted. Please use ./installer.sh help for more help"
    exit
fi

action_option_processed=0
driver_option_processed=0
sriov_option_processed=0
service_option_processed=0

# Parse and validate the command line arguments
if (( $CL_OPTIONS_COUNT > "0" )); then
    # First set the defaults
    ActionSelected=$PERFORM_ACTION_BUILD
    BuildTarget=$BUILD_ACCEL_SAMPLE
    DriverOption=$DRIVER_OPTION_AUTO
    SriovOption=$SRIOV_MODE_NONE
    ServiceOption=$SERVICE_NONE
    for i in $@; do
       case $i in
           build | install | clean | uninstall | show_accel_info | version | help )
               if (( $action_option_processed == "0")); then
                    ActionSelected=$i
                    action_option_processed=1
               else
                    echo -e "\nError: Unable to process command. Action is already provided \n"
                    exit
               fi

           ;;

           mux | QAT1.5 | QAT1.6 | QAT1.6_mux)
               if (( $driver_option_processed == "0")); then
                    DriverOption=$i
                    driver_option_processed=1
               else
                    echo -e "\nError: Unable to process command. Driver option is already provided \n"
                    exit
               fi
           ;;

           host| guest )
               if (( $sriov_option_processed == "0")); then
                    SriovOption=$i
                    sriov_option_processed=1

                    if [ $SriovOption = "$SRIOV_MODE_HOST" ]; then
                        echo "Host"
                        export ICP_SRIOV=1
                        InstallAccelSriovArgument=1
                    else
                        echo "Guest"
                        export ICP_SRIOV=1
                        InstallAccelSriovArgument=2
                    fi
               else
                    echo -e "\nError: Unable to process command. SRIOV option is already provided \n"
                    exit
               fi
           ;;
           build_dc_only | build_accel | build_sample | gige)
               if (($service_option_processed == "0")) && (($action_option_processed == "0")); then
                    ServiceOption=$i
                    service_option_processed=1
                    action_option_processed=1
                    
                    if [ $ServiceOption = "$SERVICE_DC_ONLY" ]; then
                        ActionSelected=$PERFORM_ACTION_BUILD
                    elif [ $ServiceOption = "$SERVICE_BUILD_ACCEL" ]; then
                          ActionSelected=$PERFORM_ACTION_BUILD_ACCEL
                          echo -e "BUILD_ACCEL"
                          BuildTarget=$BUILD_ACCEL
                    elif [ $ServiceOption = "$SERVICE_BUILD_SAMPLE" ]; then 
                          ActionSelected=$PERFORM_ACTION_BUILD_SAMPLE 
                          echo -e "BUILD_SAMPLE"
                          BuildTarget=$BUILD_SAMPLE
                    elif [ $ServiceOption = "$SERVICE_GIGE" ]; then 
                          ActionSelected=$PERFORM_ACTION_INSTALL 
                    fi 
               else
                    echo -e "\nError: Invalid combination of service and action option provided \n"
                    exit
               fi
           ;;   
           dependency_list)
                    ActionSelected=$i
           ;;
           /* )
              echo "PATH PROVIDED"
              INSTALL_LOCATION=$i
           ;;

           * )
              echo -e "\nError: Unrecognized argument [$i]"
              echo -e "script usage help: \"./installer.sh help\""
              exit
           ;;
       esac
    done
fi
CmdLineValidate()
{
    if [ $ServiceOption = "$SERVICE_GIGE" ]; then 
        if [ $DriverOption != "$DRIVER_OPTION_QAT1_5" ]; then
            echo -e "\nError: Invalid driver option for GigE install"
            echo -e "GigE installation is supported in QAT1.5 only"
            exit
        else
            echo -e "Enable GIGE"
            USE_WATCHDOG=true
        fi
    fi
    if [ $ServiceOption = "$SERVICE_DC_ONLY" ]; then 
        if [ $DriverOption != "$DRIVER_OPTION_QAT1_6" ]; then
           echo -e "\nError: Invalid driver option for DC_ONLY build"
           echo -e "DC_ONLY build is supported in QAT1.6 only"
           exit
        else
            echo -e "DC_ONLY_BUILD"
            BUILD_DC_ONLY=true
        fi
    fi
}
CmdLineValidate
DisplayConfiguration()
{
    echo -e "\t QAT Devices found: $numQat16Devices QAT1.6 devices"
    echo -e "\t                    $numQat15Devices QAT1.5 devices"
    echo -e "\t Configuration:     Build $BuildTarget"
    if [ $BUILD_DC_ONLY == true ]; then
       echo -e "\t                    DC only build for DH895xCC non-Mux" 
    fi
    if [ $USE_WATCHDOG == true ]; then
       echo -e "\t                    GigE Watchdog Enabled"
    fi
    if [ $DriverOption = "$DRIVER_OPTION_MUX" ]; then
        echo -e "\t                    With MUX Enabled "
    fi

    if [ $DriverOption = "$DRIVER_OPTION_QAT1_5" ]; then
        echo -e "\t                    for QAT1.5 "
    fi

    if [ $DriverOption = "$DRIVER_OPTION_QAT1_6" ]; then
        echo -e "\t                    for QAT1.6 "
    fi

    if [ $DriverOption = "$DRIVER_OPTION_QAT1_6_MUX" ]; then
        echo -e "\t                    for QAT1.6 with Mux "
    fi

    if [ $DriverOption = "$DRIVER_OPTION_AUTO" ]; then
       if (($numQat16Devices != "0")) && (($numQat15Devices != "0")); then
           echo -e "\t                    With MUX Enabled "
       elif (($numQat16Devices != "0")); then
           echo -e "\t                    for QAT1.6 "
       elif (($numQat15Devices != "0")); then
           echo -e "\t                    for QAT1.5 "
       else
           echo -e "\t                    No Device detected "
       fi
    fi
    
    if [ $SriovOption = "$SRIOV_MODE_NONE" ]; then
        echo -e "\t                    And SR-IOV Disabled "
    fi
    if [ $SriovOption = "$SRIOV_MODE_HOST" ]; then
        echo -e "\t                    and with SR-IOV(Host) "
    fi
    if [ $SriovOption = "$SRIOV_MODE_GUEST" ]; then
        echo -e "\t                    and with SR-IOV(Guest) "
    fi
    
    echo -e "\t Exit and re-enter to set defaults" 
    echo -e ""

}

DisplayAndHandleConfigurationOptions()
{
    echo -e "\t Options to change configuration:"
    echo -e "\t --------------------------------"
    echo -e "\t a1  Set Build Target as \"sample_code only\""
    echo -e "\t a2  Set Build Target as \"DC_ONLY acceleration and sample_code\""
    echo -e "\t a3  Set Build Target as \"acceleration driver only\""
    echo -e "\t b1  Set Build Location"
    echo -e "\t c1  QAT1.5 Only"
    echo -e "\t c2  QAT1.6 Only"
    echo -e "\t c3  QAT1.6 with Mux"
    echo -e "\t c4  QAT1.5 and QAT1.6 with Mux"
    echo -e "\t c5  Enable/Disable Mux based on devices detected now"
    echo -e "\t d1  Set SRIOV Mode to \"Host\""
    echo -e "\t d2  Set SRIOV Mode to \"Guest\""
    if [ $USE_WATCHDOG == true ]; then
       echo -e "\t e1  Toggle GigE Watchdog for Acceleration Install (Enabled)"
    else
       echo -e "\t e1  Toggle GigE Watchdog for Acceleration Install (Disabled)"
    fi
    echo -e "\t z1  Go Back to Main Menu"
    echo -e ""
    read config_option
    
    case $config_option in
        a1 )
            BuildTarget=$BUILD_SAMPLE
        ;;
        
        a2 )
            BUILD_DC_ONLY=true
        ;;

        a3 )
            BuildTarget=$BUILD_ACCEL
        ;;
        
        b1 )
            echo -e "\tWhere Would you like to  Build the QAT Package?"
            echo -e "\tEnter to Accept Default [$INSTALL_DEFAULT]"
            read INSTALL_LOCATION
        ;;
        
        c1 )
            DriverOption=$DRIVER_OPTION_QAT1_5
        ;;
        
        c2 )
            DriverOption=$DRIVER_OPTION_QAT1_6
        ;;
        
        c3 )
            DriverOption=$DRIVER_OPTION_QAT1_6_MUX
        ;;
        
        c4 )
            DriverOption=$DRIVER_OPTION_MUX
        ;;
        
        c5 )
            DriverOption=$DRIVER_OPTION_AUTO
        ;;
        
        d1 )
            SriovOption=$SRIOV_MODE_HOST
            export ICP_SRIOV=1
            InstallAccelSriovArgument=1
        ;;
        
        d2 )
            SriovOption=$SRIOV_MODE_GUEST
            export ICP_SRIOV=1
            InstallAccelSriovArgument=2
        ;;
  
        e1 )
            if [ $USE_WATCHDOG == true ]; then
               USE_WATCHDOG=false
            else
               USE_WATCHDOG=true
           fi
        ;;

        z1 ) 
        ;;
        
        * )
            echo -e "\nError: Invalid Option selected "
        ;;
    esac
}

MapInputOptions()
{
    case $case in
        1 )
            if [ "$BuildTarget" = "$BUILD_ACCEL_SAMPLE"  ]; then
                   ActionSelected=$PERFORM_ACTION_BUILD
            elif [  "$BuildTarget" = "$BUILD_ACCEL"  ]; then
                   ActionSelected=$PERFORM_ACTION_BUILD_ACCEL
            else
                   ActionSelected=$PERFORM_ACTION_BUILD_SAMPLE
            fi
        ;;
        
        2 )
            ActionSelected=$PERFORM_ACTION_CLEAN_BUILD
        ;;
        
        3 )
            if [ "$BuildTarget" = "$BUILD_SAMPLE"  ]; then
            	echo -e "\nError: Invalid Action with BuildTarget as '$BUILD_SAMPLE' ****>\n\n"
            	ActionSelected=""
            else
            ActionSelected=$PERFORM_ACTION_INSTALL
            fi
        ;;
        
        4 )
            ActionSelected=$PERFORM_ACTION_UNINSTALL
        ;;

        5 )
            ActionSelected=$PERFORM_ACTION_ACCEL_INFO
        ;;
 
        6 )
            DisplayAndHandleConfigurationOptions
            ActionSelected=""
        ;;

        7 )
            ActionSelected=$PERFORM_ACTION_PKG_DEPENDENCY_LIST
        ;;

        0 )
            echo -e "\n\t****EXITING INSTALLER****>\n\n"
            if [ "$InstallerOperationStatus" != "0" ];then
               echo -e "\n\n\n\t***installer operation status is Failure***>\n\n" | tee -a InstallerLog.txt
               exit 1
            else
               echo -e "\n\n\n\t***installer operation status is Success***>\n\n" | tee -a InstallerLog.txt
               exit 0
            fi
        ;;            

        * )
            echo -e "\nError: Invalid Option selected "
            ActionSelected=""
        ;;

    esac
}

WelcomeMessage()
{
    if [ "$(id -u)" != "0" ]; then
        echo -e "\n\n==============================================="
        echo -e "\n\nError: This script must be run as root"
        echo -e "\n\n===============================================\n\n"
        exit
    fi

   #Print Welcome Message
   #====================
   echo -e "\n\n\t========================================================="
   echo -e "\tWelcome to Intel(R) QuickAssist Interactive Installer -v2"
   echo -e "\t=========================================================\n\n"
}
DisplayVersion()
{
   WelcomeMessage
}

DisplayHelp()
{
   echo -e "\t Usage: ./installer.sh <options> \n"
   echo -e "\t You may choose options from the following groups \n" 
   echo -e "\t Do not combine action and service option \n" 
   echo -e "\t actions         : build / install / clean / uninstall / version / help "
   echo -e "\t driver_options  : QAT1.5 / QAT1.6 / mux / QAT1.6_mux "
   echo -e "\t sriov_options   : host / guest "
   echo -e "\t service_options : build_accel / build_sample / build_dc_only / gige "
   echo -e "\t path            : Location where you would like to build the QAT Package "
   echo -e "\t                   Make sure you start path with /. "
   echo -e "\t commands help   : "
   echo -e "\t To build QAT1.5 acceleration and sample code                   \"./installer.sh build QAT1.5 \""
   echo -e "\t To build QAT1.6 acceleration and sample code                   \"./installer.sh build QAT1.6 \""
   echo -e "\t To build QAT1.6 acceleration and sample code with Mux          \"./installer.sh build QAT1.6_mux \""
   echo -e "\t To build QAT1.5 & QAT1.6 acceleration and sample code with Mux \"./installer.sh build mux \""
   echo -e "\t To build QAT1.5 acceleration driver                            \"./installer.sh build_accel QAT1.5 \""
   echo -e "\t To build QAT1.6 acceleration driver                            \"./installer.sh build_accel QAT1.6 \""
   echo -e "\t To build QAT1.6 acceleration driver with Mux                   \"./installer.sh build_accel QAT1.6_mux \""
   echo -e "\t To build QAT1.5 sample code                                    \"./installer.sh build_sample QAT1.5 \""
   echo -e "\t To build QAT1.6 sample code                                    \"./installer.sh build_sample QAT1.6 \""
   echo -e "\t To build QAT1.6 sample code with Mux                           \"./installer.sh build_sample mux \""
   echo -e "\t To build QAT1.6 acceleration and sample code with dc_only      \"./installer.sh build_dc_only QAT1.6 \""
   echo -e "\t To install QAT1.5 acceleration                                 \"./installer.sh install QAT1.5 \""
   echo -e "\t To install QAT1.6 acceleration                                 \"./installer.sh install QAT1.6 \""
   echo -e "\t To install QAT1.6 acceleration with Mux                        \"./installer.sh install QAT1.6_mux \""
   echo -e "\t To install QAT1.5 & QAT1.6 acceleration with Mux               \"./installer.sh install mux \""
   echo -e "\t To install QAT1.5 acceleration in host                         \"./installer.sh install QAT1.5 host \""
   echo -e "\t To install QAT1.6 acceleration in host                         \"./installer.sh install QAT1.6 host \""
   echo -e "\t To install QAT1.6 acceleration with Mux in host                \"./installer.sh install QAT1.6_mux host \""
   echo -e "\t To install QAT1.5 & QAT1.6 acceleration with Mux in host       \"./installer.sh install mux host \""
   echo -e "\t To install QAT1.5 acceleration in guest                        \"./installer.sh install QAT1.5 guest \""
   echo -e "\t To install QAT1.6 acceleration in guest                        \"./installer.sh install QAT1.6 guest \""
   echo -e "\t To install QAT1.6 acceleration with Mux in guest               \"./installer.sh install QAT1.6_mux guest \""
   echo -e "\t To install QAT1.5 & QAT1.6 acceleration with Mux in guest      \"./installer.sh install mux guest \""
   echo -e "\t To install QAT1.5 acceleration with GigE Watchdog              \"./installer.sh gige QAT1.5 \""
   echo -e "\t To clean QAT1.5 and/or QAT1.6 build output                     \"./installer.sh clean \""
   echo -e "\t To uninstall QAT1.5 and/or QAT1.6 acceleration                 \"./installer.sh uninstall \""
   echo -e "\t To print installer version                                     \"./installer.sh version \""
   echo -e "\t To modify acceleration and sample code build output path       \"./installer.sh build /root/test_build \""
   echo -e "\t To get system package and kernel source header file dependency list   \"./installer.sh dependency_list \""
   echo -e "\n \t You may use interactive menu options by not entering any options in command line\n"
}
ProbeAccDeviceInfo()
{
    numQat16Devices=`lspci -nd 8086: | egrep -c "$DH895_DEVICE_NUMBER|$DH895_DEVICE_NUMBER_VM"`
    numQat15Devices=`lspci -nd 8086: | egrep -c "$C2XXX_DEVICE_NUMBER|$DH89X_DEVICE_NUMBER|$DH89X_DEVICE_NUMBER_VM"`

    if (($numQat16Devices != "0")) && (($numQat15Devices != "0")); then
       export WITH_CPA_MUX=1
    else
      if (($numQat16Devices != "0"));then
         ICP_DRIVER_TYPE=$QAT16
      fi  
      if (($numQat15Devices != "0"));then
         ICP_DRIVER_TYPE=$QAT15 
      fi
    fi

}

SetEnvVariablesForBuild()
{
    invalidConfigDetected=0
    case $DriverOption in
        "$DRIVER_OPTION_MUX" )
            export WITH_CPA_MUX=1
        ;;

        "$DRIVER_OPTION_AUTO" )
            ProbeAccDeviceInfo
            if (($numQat15Devices == "0")) && (($numQat16Devices == "0")); then
                echo -e "\t Driver mode auto selected. But no devices detected. Cannot build"
                invalidConfigDetected=1
                return
            fi
        ;;

        "$DRIVER_OPTION_QAT1_5" )
            unset WITH_CPA_MUX
            ICP_DRIVER_TYPE=$QAT15
        ;;

        "$DRIVER_OPTION_QAT1_6" )
            unset WITH_CPA_MUX
            ICP_DRIVER_TYPE=$QAT16
        ;;

        "$DRIVER_OPTION_QAT1_6_MUX" )
            export WITH_CPA_MUX=1
            ICP_DRIVER_TYPE=$QAT16
        ;;

        * )
            # Ideally this case should never be hit, as DriverOption 
            # would have been already validated.
            unset WITH_CPA_MUX
            ICP_DRIVER_TYPE=$QAT16
        ;;
    esac
}

DetectUpstreamForInstall ()
{
     if [ `lsmod | grep -o ^intel_qat` ] || [ `lsmod | grep -o ^qat_dh895xcc` ]; then
        echo -e "\tAlert!!! This operating system installation has alternative QAT drivers, which"
        echo -e "\tmay have less functionality than the QAT drivers in this QAT package."
        echo -e "\t"
        echo -e "\tDo you want to continue with the installation from this QAT package?\n"
        echo -e "\t\"Y\" will disable the existing QAT drivers and install from this QAT package"
        echo -e "\t\"N\" will keep the existing QAT drivers and abort this installation script"
        echo -e "\t"
        while true; do
            read -p "        Enter your Option Y or N :" option
            echo -e ""
            case $option in
                 [Yy] )
                               rmmod -f qat_dh895xcc;
                               rmmod -f intel_qat;
                               echo -e "\tThe default qat_dh895xcc driver successfully uninstalled."
                               break;;
                 [Nn] )
                               echo -e "\tAborting the QAT Package installation !!!"
                               echo -e ""
                               exit;;
                  * ) echo -e "\tPlease answer Y(y) or N(n).\a";;
            esac
        done
        echo -e ""
     fi
}

ValidateConfigForInstall()
{
     ProbeAccDeviceInfo

     invalidConfigDetected=0
     
     if [ $USE_WATCHDOG == true ]; then
         if [ $DriverOption != "$DRIVER_OPTION_QAT1_5" ]; then
            echo -e "\nError: Invalid driver option for Gige"
            echo -e "Gige is supported in QAT1.5 only"
            invalidConfigDetected=1
            return
         fi
     fi
     if [ $BUILD_DC_ONLY == true ]; then
         echo -e "\nError: Invalid action selected for DC_ONLY build"
         echo -e "Select QAT1.6 driver option and build_dc_only for DC_ONLY build"
        invalidConfigDetected=1
        return
     fi

     if (($numQat15Devices == "0")) && (($numQat16Devices == "0")); then
         echo -e "\t No devices detected to install "
         invalidConfigDetected=1
         return
     fi

     if (($numQat15Devices == "0")); then
         if [ $DriverOption = "$DRIVER_OPTION_QAT1_5" ]; then
             echo -e "\nError: There are no DH89xxcc or C2xxx devices detected. Hence cannot proceed with installation "
             invalidConfigDetected=1
             return
         fi
     fi

     if (($numQat16Devices == "0")); then
         if [ $DriverOption = "$DRIVER_OPTION_QAT1_6" ] || [ $DriverOption = "$DRIVER_OPTION_QAT1_6_MUX" ]; then
             echo " There is  NO DH895xcc detected. Hence cannot proceed with installation "
             invalidConfigDetected=1
             return
         fi
     fi

     if [ ! -z "$WITH_CPA_MUX" ] && [ $WITH_CPA_MUX == "1" ]; then
         if [ $DriverOption = "$DRIVER_OPTION_QAT1_5" ]; then
             echo "Both QAT1.5 and QAT1.6 devices are detected. But only QAT1.5 is selected."
             echo "Proceeding with Only QAT1.5 installation without mux  "
             unset WITH_CPA_MUX
             ICP_DRIVER_TYPE=$QAT15
             return
         fi

         if [ $DriverOption = "$DRIVER_OPTION_QAT1_6" ]; then
             echo "Both QAT 1.5 and 1.6 devices are detected. But only QAT1.6 is selected."
             echo "Proceeding with Only QAT1.6 installation without mux  "
             unset WITH_CPA_MUX
             ICP_DRIVER_TYPE=$QAT16
             return
         fi
     fi

     if [ $DriverOption = "$DRIVER_OPTION_MUX" ]; then
         export WITH_CPA_MUX=1
     fi
     
     if [ $DriverOption = "$DRIVER_OPTION_QAT1_6_MUX" ]; then
         export WITH_CPA_MUX=1
     fi
     
}

CreateBuildFolders()
{
   if [ -z $INSTALL_LOCATION ]; then
      echo -e "\n\n\tInstall Location set :  $INSTALL_DEFAULT\n"
      INSTALL_LOCATION="$INSTALL_DEFAULT"
   else 
      echo -e "\n\n\tInstall Location set :  $INSTALL_LOCATION\n"
   fi

   if [ -z $WITH_CPA_MUX ]; then
  
       ICP_BUILD_OUTPUT="$INSTALL_LOCATION/$ICP_DRIVER_TYPE/$BUILD_OUTPUT_DEFAULT"
  
       if [ ! -e $ICP_BUILD_OUTPUT ]; then
            mkdir -p "$ICP_BUILD_OUTPUT"
       fi
   else 
      echo -e "BUILD WITH MUX \n"

      ICP_BUILD_OUTPUT_QAT16="$INSTALL_LOCATION/$QAT16/$BUILD_OUTPUT_DEFAULT"
      
      ICP_BUILD_OUTPUT_QAT15="$INSTALL_LOCATION/$QAT15/$BUILD_OUTPUT_DEFAULT"
   
      if [ ! -e $ICP_BUILD_OUTPUT_QAT16 ]; then
          mkdir -p "$ICP_BUILD_OUTPUT_QAT16"
      fi
      if [ ! -e $ICP_BUILD_OUTPUT_QAT15 ]; then
          mkdir -p "$ICP_BUILD_OUTPUT_QAT15"
      fi
   fi
}

SetENV_QAT16()
{
   ICP_DRIVER_TYPE=$QAT16
   export ICP_ROOT=$INSTALL_PATH/$QAT16

   if [ -z $ICP_BUILD_OUTPUT_QAT16 ]; then
      export ICP_BUILD_OUTPUT=$ICP_BUILD_OUTPUT
   else
      export ICP_BUILD_OUTPUT=$ICP_BUILD_OUTPUT_QAT16
   fi

   export ICP_BUILD_OUTPUT=$ICP_BUILD_OUTPUT
   export ICP_BUILDSYSTEM_PATH=$ICP_ROOT/quickassist/build_system/
   export ICP_TOOLS_TARGET=$ICP_TOOLS_TARGET
   export ICP_ENV_DIR=$ICP_ROOT/quickassist/build_system/build_files/env_files
   export LD_LIBRARY_PATH=$ICP_ROOT/build
}

SetENV_QAT15()
{
   ICP_DRIVER_TYPE=$QAT15
   export ICP_ROOT=$INSTALL_PATH/$QAT15
   if [ -z $ICP_BUILD_OUTPUT_QAT15 ]; then
      export ICP_BUILD_OUTPUT=$ICP_BUILD_OUTPUT
   else
      export ICP_BUILD_OUTPUT=$ICP_BUILD_OUTPUT_QAT15
   fi
   export ICP_BUILDSYSTEM_PATH=$ICP_ROOT/quickassist/build_system/
   export ICP_TOOLS_TARGET=$ICP_TOOLS_TARGET
   export ICP_ENV_DIR=$ICP_ROOT/quickassist/build_system/build_files/env_files
   export LD_LIBRARY_PATH=$ICP_ROOT/build
}

DetectStepping()
{
    local status=0
    PF_ID=$DH895_DEVICE_NUMBER
    VF_ID=$DH895_DEVICE_NUMBER_VM
    local dh895DeviceNumber=`lspci -nd 8086: | egrep "$PF_ID|$VF_ID"`
    bdf=`lspci -nd 8086: | grep "$dh895DeviceNumber" | awk '{print $1}' | sed -n ''$1'p'`
    busNum=`echo $bdf | awk -F: '{print $1}'`
    deviceNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $1}'`
    functionNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $2}'`
    dh895Stepping=`od -tx1 -Ax -j8 -N1 /proc/bus/pci/$busNum/$deviceNum.$functionNum | awk '{print $2}' | sed '/^$/d'`

    case $dh895Stepping in
        00)
            echo -e "DH895x Stepping A0 detected"
            status=1
        ;;

        *)
            echo -e "\nError: Invalid Dh895x Stepping detected\n"
            status=0
        ;;

    esac

    return $status

}

PrintQat1_6_AccDeviceInfo()
{
    local status=0
    if [ -n "`lspci -nd 8086: | egrep $DH895_DEVICE_NUMBER`" ]; then
        local dh895DeviceNumber=$DH895_DEVICE_NUMBER
    else
        local dh895DeviceNumber=$DH895_DEVICE_NUMBER_VM
    fi

    dh895xCount=`lspci -nd 8086: | egrep -c "$DH895_DEVICE_NUMBER|$DH895_DEVICE_NUMBER_VM"`
    echo -e "Number of DH895xCC devices on the system: $dh895xCount\n"
    numQat16Devices=`lspci -nd 8086: | egrep -c $dh895DeviceNumber`

    if [ $numQat16Devices != "0" ]; then
        for ((  i=1 ;  i <= $numQat16Devices ;  i++  ))
            do
            j=$((i-1))
            bdf=`lspci -nd 8086:$dh895DeviceNumber | grep $dh895DeviceNumber | awk '{print $1}' | sed -n ''$i'p'`
            echo "BDF=$bdf"
            busNum=`echo $bdf | awk -F: '{print $1}'`
            deviceNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $1}'`
            functionNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $2}'`
            dh895Skubit=`od -tx1 -Ax -j64 -N4 /proc/bus/pci/$busNum/$deviceNum.$functionNum | awk '{print $4}' | sed '/^$/d'`            
            dh895Skubit=`echo $((16#${dh895Skubit}))`
            DetectStepping $i
            dh895Sku=`echo $[${dh895Skubit}&0x30]`
            case $dh895Sku in

            0)
                    echo -e "device $j is SKU1 \n"
            ;;

            16)
                    echo -e "device $j is SKU2 \n"
            ;;

            32)
                    echo -e "device $j is SKU3 \n"
            ;;

            48)
                    echo -e "device $j is SKU4 \n"
            ;;

            *)
                    echo -e "\nError: Invalid Dh895x SKU detected \n"
                    status=1
            ;;
            esac
        
        done
    fi
    return $status
}
PrintQat1_5_AccDeviceInfo()
{
    local status=0
    qatDevices=`lspci -n | grep "8086" | egrep "$C2XXX_DEVICE_NUMBER|$DH89X_DEVICE_NUMBER|$DH89X_DEVICE_NUMBER_VM" | sed -e 's/ /-/g'`
    numQat15Devices=`lspci -n | grep "8086" | egrep -c "$C2XXX_DEVICE_NUMBER|$DH89X_DEVICE_NUMBER|$DH89X_DEVICE_NUMBER_VM"`
    echo -e "\nNumber of DH89xxCC devices on the system:$numQat15Devices\n"
    if [ $numQat15Devices != "0" ]; then
        for qatDevice in $qatDevices ; do
            bdf=`echo $qatDevice | awk -F'-' '{print $1}'`
            echo -e "BDF=$bdf"
            did=`echo $qatDevice | awk -F'-' '{print $3}'`

            case $did in
            8086:$C2XXX_DEVICE_NUMBER)  rid=`echo $qatDevice | awk -F'(' '{print $2}'`
                        case $rid in
                            rev-02*)
                                echo -e "C2xxx B0 device detected\n"
                            ;;
                            *)
                                echo -e "C2xxx Ax device detected\n"
                                export ICP_A0_C2XXX_SILICON=1
                            ;;
                        esac
                ;;
             8086:$DH89X_DEVICE_NUMBER) rid=`echo $qatDevice | awk -F'(' '{print $2}'`
                case $rid in
                rev-2*) echo -e "DH89xxCC C0 Stepping detected\n"
                        ;;
                     *) echo -e "\nError: Unsupported DH89xxCC Stepping detected\n"
                        status=1
                        ;;
                esac
                ;;
             8086:$DH89X_DEVICE_NUMBER_VM) echo -e "DH89xxCC Virtual Function device detected\n"
                ;;
            *) echo -e "\nError: Invalid device detected\n"
               status=1
               ;;
            esac
        done
    else
        if [ -z WITH_CPA_MUX ]; then
            echo -e "\nError: No Acceleration Device Detected\n"
            status=1
        else
            echo -e "Warning: No Acceleration Device Detected\n"
        fi
    fi
    return $status
}

PrintAccDeviceInfo()
{
   if [ -z $WITH_CPA_MUX ]; then
       if [ "$ICP_DRIVER_TYPE" == $QAT16 ];then
          PrintQat1_6_AccDeviceInfo
       fi
       if [ "$ICP_DRIVER_TYPE" == $QAT15 ];then
          PrintQat1_5_AccDeviceInfo
       fi
   else
      PrintQat1_6_AccDeviceInfo
      PrintQat1_5_AccDeviceInfo
   fi
   
}

PkgDependList()
{
    if [ $DriverOption = $DRIVER_OPTION_QAT1_5 ]; then
        SetENV_QAT15
    else
        SetENV_QAT16
    fi
    if [ -z $ICP_BUILD_OUTPUT ]; then
        ICP_BUILD_OUTPUT="$INSTALL_LOCATION/$ICP_DRIVER_TYPE/$BUILD_OUTPUT_DEFAULT"
    fi
    export ICP_BUILDSYSTEM_PATH=$ICP_ROOT/quickassist/build_system/
    if [ -e $ICP_ROOT/quickassist/ ]; then
        echo "The quickassist directory has already been created"
        echo "Do not run the tar command for quickassist"
    else
        echo "The quickassist directory is not present, so untaring the file now"
        cd $ICP_ROOT
        if [ "$ICP_DRIVER_TYPE" == $QAT16 ];then
            tar -zxof ${ICP_DRIVER_TYPE}*.L.*.tar.gz ${TAR_ADDL_OPTS}
            if [ -e cpa_mux ]; then
                mv cpa_mux $INSTALL_PATH
            fi
        else
            tar -zxof ${ICP_DRIVER_TYPE}*.L.*.tar.gz ${TAR_ADDL_OPTS}
            rm -f installer.sh LICENSE.GPL
        fi
    fi
    cd $ICP_ROOT/quickassist
    make depend
    return 0;
}

BuildAccelSample()
{
    echo "Inside BuildAccelSample"
    if [ "&PrintAccDeviceInfo" ]; then
        if [ -e $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code ]; then
            echo -e "The quickassist directory has already been created."
            echo -e "Do not run the tar command for quickassist"
        else
            echo -e "The quickassist directory is not present, do the tar command for quickassist"
            cd $ICP_ROOT
            if [ "$ICP_DRIVER_TYPE" == $QAT16 ];then
              tar -zxof $ICP_ROOT/${ICP_DRIVER_TYPE}*.L.*.tar.gz ${TAR_ADDL_OPTS} 
              if [ -e cpa_mux ]; then
                 mv cpa_mux $INSTALL_PATH
              fi
            else
              tar -zxof ${ICP_DRIVER_TYPE}*.L.*.tar.gz ${TAR_ADDL_OPTS}  
              rm -f installer.sh LICENSE.GPL
           fi
        fi

        cd $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code
        touch $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/performance/compression/calgary
        touch $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/performance/compression/calgary32
        touch $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/performance/compression/canterbury
        if [ -d /lib/firmware ]; then
            echo -e "/lib/firmware exists"
        else
            echo -e "/lib/firmware doesn't exist, create it first"
            mkdir /lib/firmware
        fi
        cp $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/performance/compression/calgary /lib/firmware
        cp $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/performance/compression/calgary32 /lib/firmware
        cp $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/performance/compression/canterbury /lib/firmware
        make perf_all
        InstallerOperationStatus=$?
        if [ "$InstallerOperationStatus" != "0" ];then
           echo -e "\nError: Sample code Build performance Failed***>" | tee -a InstallerLog.txt
           return 1
        fi 
        if make ; then
            cd $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/build
            cp * $ICP_BUILD_OUTPUT/. 
            cd $ICP_BUILD_OUTPUT
            ls -la
        else
            echo -e "\nError: Acceleration Sample Code Build Failed***>"
            return 1 
        fi
    fi
    return 0 
}

InstallAccel()
{
    local INSTALL_TYPE=$1
    if [ $INSTALL_TYPE == "2" ]; then
        local dh895DeviceNumber=$DH895_DEVICE_NUMBER_VM
    else
        local dh895DeviceNumber=$DH895_DEVICE_NUMBER
    fi
    local libDir="/lib"
    if [ "&PrintAccDeviceInfo" ]; then
        if [ -e $ICP_ROOT/quickassist/ ]; then
            echo "The $ICP_DRIVER_TYPE package has already been compiled"
        else
            echo "The quickassist directory is not present, do the tar command for quickassist"
            cd $ICP_ROOT
            tar -zxof $ICP_ROOT/${ICP_DRIVER_TYPE}*.L.*.tar.gz ${TAR_ADDL_OPTS} 
            if [ -e cpa_mux ]; then
               mv cpa_mux $INSTALL_PATH
            fi
        fi

        if [ -d /lib/firmware ]; then
            echo "/lib/firmware exists, then check /lib/firmware/dh895xcc"
        else
            echo "/lib/firmware doesn't exist, create it before installing"
            mkdir /lib/firmware
        fi
                
        if [ -d /lib/firmware/dh895xcc ];then
            echo "/lib/firmware/dh895xcc exists, install *.bin to this dir"
        else
            echo "/lib/firmware/dh895xcc doesn't exist, create it before installing"
            mkdir /lib/firmware/dh895xcc
        fi
    
        cd $ICP_ROOT/quickassist
        make clean
        if make; then
            numQat16Devices=`lspci -nd 8086: | egrep -c $dh895DeviceNumber`
            cd $ICP_BUILD_OUTPUT
            if [ $numQat16Devices != "0" ]; then
                if [ -z $WITH_CPA_MUX ]; then
                    echo -e "Install Without Mux"
                else
                    if [ -e $INSTALL_LOCATION/$QAT15/build/adf_ctl ]; then
                        rm -f $INSTALL_LOCATION/$QAT15/build/adf_ctl
                        cp -f adf_ctl $INSTALL_LOCATION/$QAT15/build/adf_ctl
                    fi
                fi 
                echo "Copy QAT firmware to /lib/firmware/dh895xcc"
            
                if [ $1 == "2" ]; then
                    echo -e "SR_IOV guest install"
                    FIRMWARE_LIST=$VF_BIN_LIST
                else
                    echo -e "normal/SR_IOV host install"
                    FIRMWARE_LIST=$BIN_LIST_QAT1_6
                fi
                for bin_obj in $FIRMWARE_LIST;
                    do
                        echo "Copying $bin_obj to /lib/firmware/dh895xcc"
                        if [ -e /lib/firmware/dh895xcc/$bin_obj ]; then
                            echo "dh895xcc firmware exists, delete it first"
                            /bin/rm -f /lib/firmware/dh895xcc/$bin_obj
                        fi
                        install -D -m 640 $bin_obj /lib/firmware/dh895xcc
                done

                echo "Copying kernel obj to $MODULE_DIR"
                if [ -z $WITH_CPA_MUX ]; then
                    if [ $1 == "2" ]; then
                        install -D -m 640 icp_qa_al_vf.ko $MODULE_DIR/icp_qa_al.ko
                    else  
                        install -D -m 640 icp_qa_al.ko $MODULE_DIR/icp_qa_al.ko
                    fi  
                else
                    install -D -m 640 qat_mux.ko $MODULE_DIR/qat_mux.ko
                    if [ $1 == "2" ]; then
                        install -D -m 640 qat_1_6_mux_vf.ko $MODULE_DIR/qat_1_6_mux.ko
                    else  
                        install -D -m 640 qat_1_6_mux.ko $MODULE_DIR/qat_1_6_mux.ko
                    fi  
                fi    

                echo "Creating module.dep file for QAT released kernel object"
                echo "This will take a few moments"
                /sbin/depmod -a

                if [ $1 == "0" ]; then
                    if [ $numQat16Devices != "0" ]; then
                        for (( i=1; i<=$numQat16Devices; i++ ))
                            do
                                j=$((i-1))
                                bdf=`lspci -nd 8086: | grep $dh895DeviceNumber | awk '{print $1}' | sed -n ''$i'p'`
                                busNum=`echo $bdf | awk -F: '{print $1}'`
                                deviceNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $1}'`
                                functionNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $2}'`
                                dh895Skubit=`od -tx1 -Ax -j64 -N4 /proc/bus/pci/$busNum/$deviceNum.$functionNum | awk '{print $4}' | sed '/^$/d'`            
                                dh895Skubit=`echo $((16#${dh895Skubit}))`
                                dh895Sku=`echo $[${dh895Skubit}&0x30]`

                                case $dh895Sku in

                                    0|16|48)
                                        echo "Copying dh895xcc_qa_dev$(($j%2)).conf.v2 to /etc/dh895xcc_qa_dev$j.conf"
                                        install -D -m 660 $ICP_ROOT/quickassist/config/dh895xcc_qa_dev"$(($j%2))".conf.v2 /etc/dh895xcc_qa_dev"$j".conf
                                    
                                    ;;

                                    32)
                                        echo "Copying dh895xcc_qa_dev0.conf.v2 to /etc/dh895xcc_qa_dev$j.conf"
                                        install -D -m 660 $ICP_ROOT/quickassist/config/dh895xcc_qa_dev0.conf.v2.compression /etc/dh895xcc_qa_dev"$j".conf
                                    ;;

                                    *)
                                        echo -e "\nError: Invalid Dh895x SKU detected"
                                        status=1
                                    ;;

                                esac
                        done
                    fi 
                    elif [ $1 == "1" ]; then
                    for (( i=1; i<=$numQat16Devices; i++ ))
                        do
                            j=$((i-1))
                            bdf=`lspci -nd 8086:$dh895DeviceNumber | grep $dh895DeviceNumber | awk '{print $1}' | sed -n ''$i'p'`
                            busNum=`echo $bdf | awk -F: '{print $1}'`
                            deviceNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $1}'`
                            functionNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $2}'`
                            dh895Skubit=`od -tx1 -Ax -j64 -N4 /proc/bus/pci/$busNum/$deviceNum.$functionNum | awk '{print $4}' | sed '/^$/d'`            
                            dh895Skubit=`echo $((16#${dh895Skubit}))`
                            dh895Sku=`echo $[${dh895Skubit}&0x30]`
                            case $dh895Sku in
                                0|16|48)
                                    echo "Copying dh895xcc_qa_dev0.conf.v2.iov to /etc/dh895xcc_qa_dev$j.conf"
                                    install -D -m 660 $ICP_ROOT/quickassist/config/dh895xcc_qa_dev0.conf.v2.iov /etc/dh895xcc_qa_dev"$j".conf
                                ;;
                        
                                32)
                                    echo "Copying dh895xcc_qa_dev0.conf.v2.iov.compression to /etc/dh895xcc_qa_dev$j.conf"
                                    install -D -m 660 $ICP_ROOT/quickassist/config/dh895xcc_qa_dev0.conf.v2.iov.compression /etc/dh895xcc_qa_dev"$j".conf
                                ;;
                        
                                *)
                                    echo -e "\nError: Invalid Dh895x SKU detected"
                                    status=1
                                ;;
                            esac
                        done
                    else
                        for (( i=0; i<$numQat16Devices; i++ ))
                        do
                            echo "Copying dh895xcc_qa_dev0.conf.v2.vm to /etc/dh895xcc_qa_dev$i.conf"
                            install -D -m 660 $ICP_ROOT/quickassist/config/dh895xcc_qa_dev0.conf.v2.vm /etc/dh895xcc_qa_dev"$i".conf
                        done
                    fi

                    echo "Creating startup and kill scripts"
                    install -D -m 750 qat_service /etc/init.d/qat_service
                    install -D -m 750 adf_ctl /etc/init.d/adf_ctl

                    if [ $OS == "x86_64" ]; then
                       if [ -d /lib64 ]; then
                          libDir="/lib64"
                       fi
                    fi

                    echo "Copying Dynamic libraries to $libDir"
                    cp lib*_s.so $libDir
                    echo "Copying Static libraries to $libDir"
                    cp lib*.a $libDir

                    echo 'KERNEL=="icp_adf_ctl" MODE="0600"' > /etc/udev/rules.d/00-dh895xcc_qa.rules
                    echo 'KERNEL=="icp_dev[0-9]*" MODE="0600"' >> /etc/udev/rules.d/00-dh895xcc_qa.rules
                    echo 'KERNEL=="icp_dev_mem?" MODE="0600"' >> /etc/udev/rules.d/00-dh895xcc_qa.rules

                    chkconfig --add qat_service

                    if [ -z $WITH_CPA_MUX ]; then
                        echo "Starting QAT service"
                        /etc/init.d/qat_service start
                        /etc/init.d/qat_service status
                    else
                         if [ $numQat15Devices == "0" ] || [ $DriverOption = "$DRIVER_OPTION_QAT1_6_MUX" ]; then
                             echo "Starting QAT service"
                             /etc/init.d/qat_service start
                             /etc/init.d/qat_service status
                         fi
                    fi
                else
                    echo "Copy the startup and kill scripts"
                    install -D -m 750 qat_service /etc/init.d/qat_service
                    install -D -m 750 adf_ctl /etc/init.d/adf_ctl
                    install -D -m 640 qat_mux.ko $MODULE_DIR/qat_mux.ko
                    if [ $OS != "x86_64" ]; then
                        echo "Copying libqat_mux_s.so to /lib"
                        cp libqat_mux_s.so /lib
                    else
                        echo "Copying libqat_mux_s.so to /lib64"
                        cp libqat_mux_s.so /lib64
                    fi
                fi
                install -D -m 660 ${ICP_ROOT}/quickassist/config/dh895xcc_qa_dev*.conf.v2 ${ICP_BUILD_OUTPUT}/

            else
                echo -e "\n\t***QAT1.6 Acceleration Not Installed***" 
                return 1
            fi
        fi 

        if [ `grep "Error" $INSTALL_PATH/InstallerLog.txt | wc -l` != "0" ]; then
            echo -e "\n\t******** An error was detected in InstallerLog.txt file********\n"
            ERR_COUNT=1
            return 1
        else
            echo -e "\n\t*** No issue detected in InstallerLog.txt file ***\n"
            ERR_COUNT=0
            return 0
        fi
}

InstallAccelDh89xxcc()
{
    local INSTALL_TYPE=$1
    if [ $INSTALL_TYPE == "2" ]; then
        local dh89DeviceNumber="$DH89X_DEVICE_NUMBER_VM"
    else
        local dh89DeviceNumber="$DH89X_DEVICE_NUMBER"
    fi

    local numDh89xxDevice=`lspci -nd 8086:$dh89DeviceNumber | grep -c $dh89DeviceNumber`
    local numC2xxxDevice=`lspci -nd 8086:$C2XXX_DEVICE_NUMBER | grep -c $C2XXX_DEVICE_NUMBER`
    local libDir="/lib"

    PrintAccDeviceInfo
    if [ "$?" -ne "0" ]; then
        echo -e "\nError: Aborting Installation***>\n"
        return 1
    fi

    if [ $numC2xxxDevice != "0" ] && [ $numDh89xxDevice == "0" ]; then
        if [ $INSTALL_TYPE != "0" ]; then
            echo -e "SR-IOV is not supported on C2xxx, you need at least one DH89xx device to be present"
            return 1
        fi
    fi
    if [ -e $ICP_ROOT/quickassist/ ]; then
        echo "The quickassist directory has already been created"
        echo "Do not run the tar command for quickassist"
    else
        cd $ICP_ROOT  
        echo "The quickassist directory is not present, do the tar command for quickassist"
        tar -zxof $ICP_ROOT/${ICP_DRIVER_TYPE}*.L.*.tar.gz ${TAR_ADDL_OPTS} 
        rm -f installer.sh LICENSE.GPL
    fi
    cd $ICP_ROOT/quickassist
    make clean
    make 2>&1 | tee -a $INSTALL_PATH/InstallerLog.txt; (exit ${PIPESTATUS[0]})
    if [ "$?" -ne "0" ] || [ `grep "Error" $INSTALL_PATH/InstallerLog.txt | wc -l` != "0" ]; then
        echo -e "\n\t******** An error was detected in InstallerLog.txt file********\n"
        ERR_COUNT=1
        return 1
    else
        echo -e "\n\t*** No issue detected in InstallerLog.txt file ***\n"
        ERR_COUNT=0
    fi
    #Check whether the device is available before installing
    if [ $numDh89xxDevice != "0" ] || [ $numC2xxxDevice != "0" ]; then
    cd $ICP_BUILD_OUTPUT

    ##check that the any dh89xx device is of the right sku to install the driver
    installDriver=0
    if [ $numDh89xxDevice != "0" ]; then
        for (( i=1; i<=$numDh89xxDevice; i++ ))
        do
            j=$((i-1))
            bdf=`lspci -nd 8086:$dh89DeviceNumber | grep $dh89DeviceNumber | awk '{print $1}' | sed -n ''$i'p'`
            echo -e "BDF=$bdf\n"
            busNum=`echo $bdf | awk -F: '{print $1}'`
            deviceNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $1}'`
            functionNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $2}'`
            dh89Sku=`od -tx4 -Ax /proc/bus/pci/$busNum/$deviceNum.$functionNum | grep "^000040" | awk '{print $5}'`
            case $dh89Sku in

                0bb80000)
                    installDriver=1
                ;;

                0b310000)
                    installDriver=1
                ;;
                0b320000)
                    dh89Sku_detail=`od -tx4 -Ax /proc/bus/pci/$busNum/$deviceNum.$functionNum | grep "^000040" | awk '{print $2}'`
                    case $dh89Sku_detail in
                        000006f0)
                            installDriver=1
                        ;;
                        000007ff)
                            echo -e "\n\t***SKU1 device does not support acceleration***\n"
                            ##do nothing
                        ;;
                    esac

                ;;
                00000000*)
                    installDriver=1
                ;;

                *)
                    echo -e "\n\t...Warning: Unknown device found...\n"
                    ##do nothing
                ;;

            esac
        done

    else
        installDriver=0
    fi

    if [ $numC2xxxDevice != "0" ]; then
        installDriver=1
    fi

    if [ $installDriver == "0" ]; then
        echo -e "\n\t***No suitable device detected to install driver***>"
        return 1
    fi

    if [ -z $WITH_CPA_MUX ]; then
        echo -e "Install Without Mux"
    else
       if [ -e $INSTALL_LOCATION/$QAT16/build/adf_ctl ]; then
          echo "Copy adf_ctl from QAT1.6/build to QAT1.5/build/"
          rm -f $INSTALL_LOCATION/$QAT15/build/adf_ctl
          cp -f $INSTALL_LOCATION/$QAT16/build/adf_ctl $INSTALL_LOCATION/$QAT15/build/adf_ctl
       fi
    fi
 
    echo "Copy QAT firmware to /lib/firmware/"
    if [ $1 == "2" ]; then
        echo -e "SR_IOV guest install"
        FIRMWARE_LIST=$VF_BIN_LIST
    else
        echo -e "normal or SR_IOV host install"
        FIRMWARE_LIST=$BIN_LIST_QAT1_5
    fi

    if [ ! -d "/lib/firmware/" ]; then
    	mkdir /lib/firmware/
    fi
    for bin_obj in $FIRMWARE_LIST;
    do
        echo "Copying $bin_obj to /lib/firmware/$bin_obj"
        install -D -m 640 $bin_obj /lib/firmware/$bin_obj
    done

    echo "Copying kernel obj to $MODULE_DIR"
    if [ -z $WITH_CPA_MUX ]; then
       if [ $1 == "2" ]; then
          install -D -m 640 icp_qa_al_vf.ko $MODULE_DIR/icp_qa_al.ko
       else  
          install -D -m 640 icp_qa_al.ko $MODULE_DIR/icp_qa_al.ko
       fi  
    else
        if [ $1 == "2" ]; then
            install -D -m 640 qat_1_5_mux_vf.ko $MODULE_DIR/qat_1_5_mux.ko
        else  
            install -D -m 640 qat_1_5_mux.ko $MODULE_DIR/qat_1_5_mux.ko
        fi  
    fi    

    echo "Creating module.dep file for QAT released kernel object"
    echo "This will take a few moments"
    /sbin/depmod -a


    if [ $1 == "0" ]; then
        #determine dh89 SKU and copy appropriate config file
        if [ $numDh89xxDevice != "0" ]; then
            for (( i=1; i<=$numDh89xxDevice; i++ ))
            do
                j=$((i-1))
                bdf=`lspci -nd 8086:$dh89DeviceNumber | grep $dh89DeviceNumber | awk '{print $1}' | sed -n ''$i'p'`
                busNum=`echo $bdf | awk -F: '{print $1}'`
                deviceNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $1}'`
                functionNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $2}'`
                dh89Sku=`od -tx1 -Ax -j64 -N1 /proc/bus/pci/$busNum/$deviceNum.$functionNum | awk '{print $2}' | sed '/^$/d'`

                case $dh89Sku in

                    00)
                        let "k = $j%2"
                        echo "Copying dh89xxcc_qa_dev$k.conf to /etc/dh89xxcc_qa_dev$j.conf"
                        install -D -m 660 dh89xxcc_qa_dev"$k".conf /etc/dh89xxcc_qa_dev"$j".conf
                    ;;

                    f0)
                        echo "Copying dh89xxcc_qa_dev0_single_accel.conf to /etc/dh89xxcc_qa_dev$j.conf"
                        install -D -m 660 dh89xxcc_qa_dev0_single_accel.conf /etc/dh89xxcc_qa_dev"$j".conf
                    ;;

                    *)
                        echo -e "\nError: Invalid Dh89xx SKU detected***>\n\n"
                        status=1
                    ;;

                esac

            done
        fi
        #copy c2xxx config file
        if [ $numC2xxxDevice != "0" ]; then
        	for (( i=1; i<=$numC2xxxDevice; i++ ))
		    do
       		        j=$((i-1))
			bdf=`lspci -nd 8086:$C2XXX_DEVICE_NUMBER | grep $C2XXX_DEVICE_NUMBER | awk '{print $1}' | sed -n ''$i'p'`
			busNum=`echo $bdf | awk -F: '{print $1}'`
			deviceNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $1}'`
			functionNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $2}'`
			c2xxxSku=`od -tx1 -Ax -j64 -N1 /proc/bus/pci/$busNum/$deviceNum.$functionNum | awk '{print $2}' | sed '/^$/d'`
			if [ $c2xxxSku == "0a" ] || [ $c2xxxSku == "06" ]; then
				echo "Copying $ICP_ROOT/quickassist/config/c2xxx_qa_dev0_single_ae.conf to /etc/c2xxx_qa_dev$j.conf"
				install -D -m 660 $ICP_ROOT/quickassist/config/c2xxx_qa_dev0_single_ae.conf /etc/c2xxx_qa_dev"$j".conf
			fi
			if [ $c2xxxSku == "00" ]; then
				echo "Copying c2xxx_qa_dev0.conf to /etc/c2xxx_qa_dev$j.conf"
			    install -D -m 660 c2xxx_qa_dev0.conf /etc/c2xxx_qa_dev"$j".conf
			fi
		done
        fi
        elif [ $1 == "1" ]; then
       	     for (( i=0; i<$numDh89xxDevice; i++ ))
	     do
                echo "Copying dh89xxcc_qa_dev0.conf.iov to /etc/dh89xxcc_qa_dev$i.conf"
	        install -D -m 660 $ICP_ROOT/quickassist/config/dh89xxcc_qa_dev0.conf.iov /etc/dh89xxcc_qa_dev"$i".conf
             done
	else
	     for (( i=0; i<$numDh89xxDevice; i++ ))
	     do
	        echo "Copying dh89xxcc_qa_dev0.conf.vm to /etc/dh89xxcc_qa_dev$i.conf"
                install -D -m 660 $ICP_ROOT/quickassist/config/dh89xxcc_qa_dev0.conf.vm /etc/dh89xxcc_qa_dev"$i".conf
	        ln -sf /etc/dh89xxcc_qa_dev"$i".conf /etc/dh89xxccvf_qa_dev"$i".conf
             done
        fi
        echo "Creating startup and kill scripts"
	if [ "$numDh89xxDevice" -ne "0" ]; then
           if [ $USE_WATCHDOG == true ]; then
              install -D -m 750 gige_watchdog_service /etc/init.d
	      install -D -m 750 icp_gige_watchdog /etc/init.d
           fi
	fi
	if [ -z $WITH_CPA_MUX ]; then
	       install -D -m 750 qat_service /etc/init.d
	       install -D -m 750 adf_ctl /etc/init.d
        fi
	if [ $OS == "x86_64" ]; then
	    if [ -d /lib64 ]; then
               libDir="/lib64"
	    fi
	fi
        echo "Copying libicp_qa_al_s.so to $libDir"
        cp lib*_s.so $libDir 
        echo "Copying lib*.a to $libDir"
        cp lib*.a $libDir
        echo "Copying libosal.a to $libDir"
        cp libosal.a $libDir
        echo "Copying libadf_proxy.a to $libDir"
        cp libadf_proxy.a $libDir
    
        if [ $numDh89xxDevice != "0" ]; then
           echo 'KERNEL=="icp_adf_ctl" MODE="0600"' > /etc/udev/rules.d/00-dh89xxcc_qa.rules
           echo 'KERNEL=="icp_dev[0-9]*" MODE="0600"' >> /etc/udev/rules.d/00-dh89xxcc_qa.rules
	   echo 'KERNEL=="icp_dev_mem?" MODE="0600"' >> /etc/udev/rules.d/00-dh89xxcc_qa.rules
        fi
	
        if [ $numC2xxxDevice != "0" ]; then
           echo 'KERNEL=="icp_adf_ctl" MODE="0600"' > /etc/udev/rules.d/00-c2xxx_qa.rules
           echo 'KERNEL=="icp_dev[0-9]*" MODE="0600"' >> /etc/udev/rules.d/00-c2xxx_qa.rules
	   echo 'KERNEL=="icp_dev_mem?" MODE="0600"' >> /etc/udev/rules.d/00-c2xxx_qa.rules
	fi
	
	if [ -e /sbin/chkconfig ] ; then
	    if [ -z $WITH_CPA_MUX ]; then
	        chkconfig --add qat_service
            fi 
            if [ "$numDh89xxDevice" -ne "0" ]; then
	       if [ $USE_WATCHDOG == true ]; then
                  chkconfig --add gige_watchdog_service
	       fi
             fi
	elif [ -e /usr/sbin/update-rc.d ]; then
	     if [ -z $WITH_CPA_MUX ]; then
                update-rc.d qat_service defaults
             fi
	     if [ "$numDh89xxDevice" -ne "0" ]; then
	        if [ $USE_WATCHDOG == true ]; then
                    update-rc.d gige_watchdog_service defaults
	        fi
	     fi
	else
	   echo "\nError: Failed to add qat_service to start-up***>"
           return 1
	fi
	echo "Starting QAT service"
	/etc/init.d/qat_service start
	/etc/init.d/qat_service status
	if [ $USE_WATCHDOG == true ]; then
	   echo "Starting GigE watchdog service"
	   /etc/init.d/gige_watchdog_service start
	fi
    fi
    return 0
}

UninstallAccel()
{
    KERNEL_OBJ_LIST="icp_qa_al.ko 
                     qat_1_6_mux.ko qat_1_5_mux.ko qat_mux.ko"
    KERNEL_VF_OBJ_LIST="icp_qa_al_vf.ko qat_1_6_mux_vf.ko qat_1_5_mux_vf.ko" 

    LIB_LIST="libicp_qa_al.a libicp_qa_al_s.so 
              libqat_1_6_mux.a libqat_1_6_mux_s.so libqat_1_5_mux.a libqat_1_5_mux_s.so
              libqat_mux.a libqat_mux_s.so
              libicp_qa_al_vf.a libicp_qa_al_vf_s.so
              libqat_1_6_mux_vf.a libqat_1_6_mux_vf_s.so libqat_1_5_mux_vf.a libqat_1_5_mux_vf_s.so
              libadf_proxy.a libosal.a"

    local QAT_MEM_DRV="qaeMemDrv"

    # Check if any drivers are installed
    driverInstalled=0
    for kern_obj in $KERNEL_OBJ_LIST;
    do
        if [[ -e $MODULE_DIR/$kern_obj ]]; then
            driverInstalled=1
            break;
        fi
    done 
    for kern_obj in $KERNEL_VF_OBJ_LIST;
    do
        if [[ -e $MODULE_DIR/$kern_obj ]]; then
            driverInstalled=1
            break;
        fi
    done 

    if (( $driverInstalled == "0" )); then
        echo -e "\nWarning: No Drivers installed. Unable to uninstall"
        return 1
    fi
    
    if [ -e /etc/init.d/gige_watchdog_service ]; then
        /etc/init.d/gige_watchdog_service stop
        if [ -e /sbin/chkconfig ] ; then
            chkconfig --del gige_watchdog_service
        fi
        if [ -e /usr/sbin/update-rc.d ]; then
            update-rc.d -f gige_watchdog_service remove
        fi
        /bin/rm -f /etc/init.d/gige_watchdog_service
        /bin/rm -f /etc/init.d/icp_gige_watchdog
    fi
   
    # Remove the qaeMemDrv.ko file if it is loaded.
    if [ `lsmod | grep -o ^$QAT_MEM_DRV` ]; then
        rmmod -f qaeMemDrv
    fi
    
    # First try to bringdown the devices
    # Check if one of the /etc/init.d script exist
    if [ -e /etc/init.d/qat_service ]; then
        /etc/init.d/qat_service shutdown
        shutdownStatus=`/etc/init.d/qat_service status | grep "state=up" | wc -l`        
        if [ $shutdownStatus = "0" ]; then
            echo -e "Removing startup scripts"
            if [ -e /sbin/chkconfig ] ; then
               chkconfig --del qat_service
            fi

            if [ -e /usr/sbin/update-rc.d ]; then
                update-rc.d -f qat_service remove
            fi

            /bin/rm -f /etc/init.d/qat_service
            /bin/rm -f /etc/init.d/adf_ctl
        else
            echo -e "\nError: Unable to bring down the devices. Cannot proceed with uninstall \n"
            return 1
        fi
    fi
    echo "Removing the firmware files"
    for bin_obj in $BIN_LIST_QAT1_6;
    do
        if [ -e /lib/firmware/dh895xcc/$bin_obj ]; then
            /bin/rm -f /lib/firmware/dh895xcc/$bin_obj
        fi
    done

    for bin_obj in $BIN_LIST_QAT1_5;
    do
        if [ -e /lib/firmware/$bin_obj ]; then
        /bin/rm -f /lib/firmware/$bin_obj
        fi
    done

    echo "Removing objects from $MODULE_DIR"
    for kern_obj in $KERNEL_OBJ_LIST;
    do
        if [ -e $MODULE_DIR/$kern_obj ]; then
            /bin/rm -f $MODULE_DIR/$kern_obj
            #incase the driver is a location that previous versions of the
            # installer placed it then attempt to remove it
            /bin/rm -f $MODULE_DIR/../../$kern_obj
        fi
    done
    for kern_obj in $KERNEL_VF_OBJ_LIST;
    do
        if [ -e $MODULE_DIR/$kern_obj ]; then
            /bin/rm -f $MODULE_DIR/$kern_obj
            #incase the driver is a location that previous versions of the
            # installer placed it then attempt to remove it
            /bin/rm -f $MODULE_DIR/../../$kern_obj
        fi
        if [ `lsmod | grep -o icp_qa_al_vf` ]; then
         `rmmod -f icp_qa_al_vf.ko`
        fi
    done

    echo "Removing device permissions rules "
    if [ -e /etc/udev/rules.d/00-dh895xcc_qa.rules ]; then
        rm -f /etc/udev/rules.d/00-dh895xcc_qa.rules
    fi
    /sbin/depmod -a
    if [ -e /etc/udev/rules.d/00-dh89xxcc_qa.rules ]; then
        rm -f /etc/udev/rules.d/00-dh89xxcc_qa.rules
    fi
    if [ -e /etc/udev/rules.d/00-c2xxx_qa.rules ]; then
        rm -f /etc/udev/rules.d/00-c2xxx_qa.rules
    fi

    echo "Rebuilding *module.dep file, this will take a few moments"
    /sbin/depmod -a

    # Remove *.a, *_s.so files
    for lib_obj in $LIB_LIST;
    do
        if [ -e /lib64/$lib_obj ]; then
            rm -f /lib64/$lib_obj
        fi

        if [ -e /lib/$lib_obj ]; then
            rm -f /lib/$lib_obj
        fi
    done
    
    # Removing soft link from the device files
    ICP_DEVICE=`ls -la /dev/char|grep icp_|awk '{ print $9 }'`

    for icp_obj in $ICP_DEVICE
    do
         unlink /dev/char/${icp_obj}
    done

    qae_obj=`ls -la /dev/char|grep qae_mem|awk '{ print $9 }'`
    if [ ! -z "$qae_obj" ]; then
         unlink /dev/char/$qae_obj
    fi	 
    echo -e "\n\n\t*** QAT Acceleration Package Uninstalled ***>\n\n\n"
    return 0
}
BuildAccel()
{
    if [ -e $ICP_ROOT/quickassist/ ]; then
        echo "The quickassist directory has already been created"
        echo "Do not run the tar command for quickassist"
    else
        echo "The quickassist directory is not present, do the tar command for quickassist"
        cd $ICP_ROOT
        if [ "$ICP_DRIVER_TYPE" == $QAT16 ];then
           tar -zxof ${ICP_DRIVER_TYPE}*.L.*.tar.gz ${TAR_ADDL_OPTS}
           if [ -e cpa_mux ]; then
               mv cpa_mux $INSTALL_PATH
            fi
        else 
           tar -zxof ${ICP_DRIVER_TYPE}*.L.*.tar.gz ${TAR_ADDL_OPTS}
           rm -f installer.sh LICENSE.GPL
        fi
    fi
    
    cd $ICP_ROOT/quickassist
    touch lookaside/firmware/icp_qat_ae.uof
    touch lookaside/firmware/icp_qat_pke.mof
    if make; then
        cd $ICP_BUILD_OUTPUT
        if [ -z $WITH_CPA_MUX ]; then
           ls -la
        else
           if [ "$ICP_DRIVER_TYPE" == $QAT15 ];then
              if [ -e $INSTALL_LOCATION/$QAT16/build/adf_ctl ]; then
                 rm -f adf_ctl
                 cp -f $INSTALL_LOCATION/$QAT16/build/adf_ctl adf_ctl
              fi 
           fi 
           ls -la
        fi
    else
        echo -e "\nError: Acceleration Build Failed***>"
        return 1;
    fi

    install -D -m 660 ${ICP_ROOT}/quickassist/config/dh89*xcc_qa_dev*.conf.v2 ${ICP_BUILD_OUTPUT}/
    return 0;
}

RemoveLogFile()
{
    echo -e "\n\n\n\t**********************************************"
    echo -e "\t* Removing log file in Build Location : $INSTALL_PATH"
    rm -f $INSTALL_PATH/InstallerLog.txt
    echo -e "\t************************************************\n\n\n\n\n"

}

PrintInstallOptions()
{
    echo -e "\n\n\n\t**********************************************"
    echo -e "\t* Build Location : $ICP_ROOT"
    echo -e "\t* Build Output Location : $ICP_BUILD_OUTPUT"
    echo -e "\t************************************************\n\n\n\n\n"

}

PrintAccSampleInstallOptions()
{
    echo -e "\n\n\n\t**********************************************"
    echo -e "\t* Build Location : $ICP_ROOT"
    echo -e "\t* Build Location : $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/build"
    echo -e "\t************************************************\n\n\n\n\n"

}

LogFileHeader()
{
    echo -e "\n\n\n ****** `date`\t*********" 2>&1 > InstallerLog.txt
    echo -e " ************************************************\n" 2>&1 > InstallerLog.txt
}

CleanBuild()
{
    if [ $DriverOption = "$DRIVER_OPTION_MUX" ] || [ $DriverOption = "$DRIVER_OPTION_QAT1_6_MUX" ]; then
       export WITH_CPA_MUX=1
    fi
    # Clean QAT1.5 build
    SetENV_QAT15
    if [ -e $ICP_ROOT/quickassist/ ]; then
        echo "Clean build for QAT1.5"
        cd $ICP_ROOT/quickassist
        make clean
        cd $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/
        make clean 
        
        if [ -z $ICP_BUILD_OUTPUT ]; then
            ICP_BUILD_OUTPUT="$INSTALL_LOCATION/$ICP_DRIVER_TYPE/$BUILD_OUTPUT_DEFAULT"
            rm -rf $ICP_BUILD_OUTPUT
            unset ICP_BUILD_OUTPUT
        fi
    else
        echo "The quickassist directory is not present for QAT1.5. Nothing to clean"
    fi

    # Clean QAT1.6 build
    SetENV_QAT16
    if [ -e $ICP_ROOT/quickassist/ ]; then
        echo "Clean build for QAT1.6"
        cd $ICP_ROOT/quickassist
        make clean
        cd $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/
        make clean 
        if [ -z $ICP_BUILD_OUTPUT ]; then
            ICP_BUILD_OUTPUT="$INSTALL_LOCATION/$ICP_DRIVER_TYPE/$BUILD_OUTPUT_DEFAULT"
            rm -rf $ICP_BUILD_OUTPUT
            unset ICP_BUILD_OUTPUT
        fi
    else
        echo "The quickassist directory is not present for QAT1.6. Nothing to clean"
    fi
    if [ $DriverOption = "$DRIVER_OPTION_MUX" ] || [ $DriverOption = "$DRIVER_OPTION_QAT1_6_MUX" ]; then
       unset WITH_CPA_MUX
    fi
    return 0
}

MainViewGeneral()
{

    #Print Disclaimer
    #================
    if (( $CL_OPTIONS_COUNT == "0" )); then
        WelcomeMessage
    fi
    #set permissions to create owner and group files only
    umask 7
    ProbeAccDeviceInfo
    
    loop=0
    while [ $loop = 0 ];
    do
        # If command line options are not provided then print these.
        if (( $CL_OPTIONS_COUNT == "0" )); then
           #Switch Statement to Select Installing Option
           #============================================
           echo -e "\t Please Select Option :"
           echo -e "\t ----------------------"
           echo -e "\t 1   Build"
           echo -e "\t 2   Clean Build"
           echo -e "\t 3   Install"
           echo -e "\t 4   Uninstall"
           echo -e "\t 5   Show Accel Info"
           echo -e "\t 6   Change Configuration"
           echo -e "\t 7   Dependency List"
           echo -e "\t 0   Exit"
           echo -e "\n"
           DisplayConfiguration
           echo -e "\n"

           read case
           MapInputOptions
        fi

        case $ActionSelected in
            $PERFORM_ACTION_BUILD )
                SetEnvVariablesForBuild
                if (($invalidConfigDetected == "1")); then
                    ActionSelected=""
                    continue
                fi
                  
                CreateBuildFolders
                if [ $BUILD_DC_ONLY == true ]; then
                    if [ $DriverOption != "$DRIVER_OPTION_QAT1_6" ]; then
                        echo -e "\nError:  DC_ONLY build is supported in QAT1.6 only \n"
                        ActionSelected=""
                        continue
                    fi
                    SetENV_QAT16
                    export ICP_DC_ONLY=1
                    export DO_CRYPTO=0
                    unset WITH_CPA_MUX
                    
                    RemoveLogFile
                    PrintInstallOptions  
                    sleep 3
                    BuildAccel 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                    InstallerOperationStatus=$?
                    if [ "$InstallerOperationStatus" != "0" ];then
                       echo -e "\nError observed in QAT1.6 accel build***>\n\n" | tee -a InstallerLog.txt
                       continue
                    else
                       echo -e "\n\n\n\t***Acceleration Build Complete***>\n\n"
                    fi
                    unset ICP_DC_ONLY
                    
                    BuildAccelSample 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                    InstallerOperationStatus=$?
                    if [ "$InstallerOperationStatus" != "0" ];then
                       echo -e "\nError observed in Sample code build***>\n\n" | tee -a InstallerLog.txt
                    else
                       echo -e "\n\n\n\t***Sample Code Build Complete***>\n\n"
                    fi
                    unset DO_CRYPTO

                elif [ -z $WITH_CPA_MUX ]; then
                   if [ $ICP_DRIVER_TYPE = $QAT16 ]; then 
                       SetENV_QAT16
                   fi
                   if [ $ICP_DRIVER_TYPE = $QAT15 ]; then 
                       SetENV_QAT15
                   fi     
                   RemoveLogFile
                   PrintInstallOptions
                   sleep 3
                   BuildAccel 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                   InstallerOperationStatus=$?
                   if [ "$InstallerOperationStatus" != "0" ];then
                      echo -e "\nError observed in $ICP_DRIVER_TYPE accel build***>\n\n" | tee -a InstallerLog.txt
                      continue
                   else
                      echo -e "\n\n\n\t***Acceleration Build Complete***>\n\n"
                   fi

                   BuildAccelSample 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                   InstallerOperationStatus=$?
                   if [ "$InstallerOperationStatus" != "0" ];then
                      echo -e "\nError observed in Sample code build***>\n\n" | tee -a InstallerLog.txt
                   else
                      echo -e "\n\n\n\t***Sample Code Build Complete***>\n\n"
                   fi

                else
                   if [ $DriverOption = "$DRIVER_OPTION_QAT1_6_MUX" ]; then
                       RemoveLogFile
                       SetENV_QAT16
                       PrintInstallOptions
                       sleep 3
                       BuildAccel 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                       InstallerOperationStatus=$?
                       if [ "$InstallerOperationStatus" != "0" ];then
                          echo -e "\nError observed in QAT1.6 accel build***>\n\n" | tee -a InstallerLog.txt
                          continue
                       else
                          echo -e "\n\n\n\t***Acceleration Build Complete***>\n\n"
                       fi
                       
                       BuildAccelSample 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                       InstallerOperationStatus=$?
                       if [ "$InstallerOperationStatus" != "0" ];then
                          echo -e "\nError observed in Sample code build***>\n\n" | tee -a InstallerLog.txt
                       else
                          echo -e "\n\n\n\t***Sample Code Build Complete***>\n\n"
                       fi
                   else
                       RemoveLogFile
                       SetENV_QAT16
                       PrintInstallOptions
                       sleep 3
                       BuildAccel 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                       InstallerOperationStatus=$?
                       if [ "$InstallerOperationStatus" != "0" ];then
                          echo -e "\nError observed in QAT1.6 accel build***>\n\n" | tee -a InstallerLog.txt
                          continue
                       else
                          echo -e "\n\n\n\t***Acceleration Build Complete***>\n\n"
                       fi
                       
                       BuildAccelSample 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                       InstallerOperationStatus=$?
                       if [ "$InstallerOperationStatus" != "0" ];then
                          echo -e "\nError observed in Sample code build***>\n\n" | tee -a InstallerLog.txt
                          continue
                       else
                          echo -e "\n\n\n\t***Sample Code Build Complete***>\n\n"
                       fi
                 
                       SetENV_QAT15
                       PrintInstallOptions
                       sleep 3
                       BuildAccel 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                       InstallerOperationStatus=$?
                       if [ "$InstallerOperationStatus" != "0" ];then
                          echo -e "\nError observed in QAT1.5 accel build***>\n\n" | tee -a InstallerLog.txt
                       else
                          echo -e "\n\n\n\t***Acceleration Build Complete***>\n\n"
                       fi
                   fi            
                       echo -e "\n\n\n\t***Acceleration and Sample_code Build With Mux Complete***>\n\n"
               fi
            ;;
            
            $PERFORM_ACTION_CLEAN_BUILD )
                CleanBuild
                InstallerOperationStatus=$?
            ;;
            
            $PERFORM_ACTION_INSTALL )
                DetectUpstreamForInstall
                ValidateConfigForInstall

                if (($invalidConfigDetected == "1")); then
                    echo -e  "\nError: Invalid Config - ABORTING INSTALLATION *** \n"
                    ActionSelected=""
                    continue
                fi
                echo -e "\n"
                CreateBuildFolders
                if [ -z $WITH_CPA_MUX ]; then
                   RemoveLogFile
                   if [ $ICP_DRIVER_TYPE = $QAT16 ]; then
                      SetENV_QAT16
                      PrintInstallOptions
                      sleep 3
                      InstallAccel $InstallAccelSriovArgument 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                      InstallerOperationStatus=$?
                      if [ "$InstallerOperationStatus" != "0" ];then
                         echo -e "\nError observed in QAT1.6 accel install***>\n\n" | tee -a InstallerLog.txt
                         continue
                      else
                         echo -e "\n\n\n\t***Acceleration install Complete***>\n\n"
                      fi
                      if [ "$BuildTarget" != "$BUILD_ACCEL" ]; then
                         BuildAccelSample 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                         InstallerOperationStatus=$?
                      fi
                   else 
                      SetENV_QAT15
                      PrintInstallOptions
                      sleep 3
                      InstallAccelDh89xxcc $InstallAccelSriovArgument 2>&1 ; (exit ${PIPESTATUS[0]})
                      InstallerOperationStatus=$?
                      if [ "$InstallerOperationStatus" != "0" ];then
                         echo -e "\nError observed in QAT1.5 accel install***>\n\n" | tee -a InstallerLog.txt
                         continue
                      else
                         echo -e "\n\n\n\t***Acceleration install Complete***>\n\n"
                      fi
                      if [ "$BuildTarget" != "$BUILD_ACCEL" ]; then
                         BuildAccelSample 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                         InstallerOperationStatus=$?
                      fi
                   fi
                else
                   if [ $DriverOption = "$DRIVER_OPTION_QAT1_6_MUX" ]; then
                       RemoveLogFile
                       SetENV_QAT16
                       PrintInstallOptions
                       sleep 3
                       VF_OBJ_LIST="qat_1_6_mux_vf.ko"
                       InstallAccel $InstallAccelSriovArgument 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                       InstallerOperationStatus=$?
                       if [ "$InstallerOperationStatus" != "0" ];then
                          echo -e "\nError observed in QAT1.6_mux accel install***>\n\n" | tee -a InstallerLog.txt
                          continue
                       else
                          echo -e "\n\n\n\t***Acceleration install Complete***>\n\n"
                       fi
                       if [ "$BuildTarget" != "$BUILD_ACCEL" ]; then
                          BuildAccelSample 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                          InstallerOperationStatus=$?
                       fi
                   else
                       RemoveLogFile
                       SetENV_QAT16
                       PrintInstallOptions
                       sleep 3
                       VF_OBJ_LIST="qat_1_6_mux_vf.ko"
                       InstallAccel $InstallAccelSriovArgument 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                       InstallerOperationStatus=$?
                       if [ "$InstallerOperationStatus" != "0" ];then
                          echo -e "\nError observed in QAT1.6 accel install***>\n\n" | tee -a InstallerLog.txt
                          continue
                       else
                          echo -e "\n\n\n\t***Acceleration install Complete***>\n\n"
                       fi
                       if [ "$BuildTarget" != "$BUILD_ACCEL" ]; then
                          BuildAccelSample 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                          InstallerOperationStatus=$?
                       fi

                       SetENV_QAT15
                       PrintInstallOptions
                       sleep 3
                       VF_OBJ_LIST="qat_1_5_mux_vf.ko"
                       InstallAccelDh89xxcc $InstallAccelSriovArgument 2>&1 ; (exit ${PIPESTATUS[0]})
                       InstallerOperationStatus=$?
                   fi
                fi
                   echo -e "\n\n\n\t<***Acceleration Installation Complete***>\n\n"
                echo -e "\n"
            ;;
            
            "$PERFORM_ACTION_UNINSTALL" )
                UninstallAccel
                InstallerOperationStatus=$?
            ;;

            $PERFORM_ACTION_BUILD_ACCEL )
                SetEnvVariablesForBuild
                if (($invalidConfigDetected == "1")); then
                    ActionSelected=""
                    continue
                fi
                CreateBuildFolders
                if [ -z $WITH_CPA_MUX ]; then
                    unset WITH_CPA_MUX
                fi
                if [ $ICP_DRIVER_TYPE = $QAT16 ]; then
                    SetENV_QAT16
                elif [ $ICP_DRIVER_TYPE = $QAT15 ]; then
                    SetENV_QAT15
                fi
                RemoveLogFile
                PrintInstallOptions
                sleep 3
                BuildAccel 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                InstallerOperationStatus=$?
                if [ "$InstallerOperationStatus" != "0" ];then
                    echo -e "\nError observed in $ICP_DRIVER_TYPE accel build***>\n\n" | tee -a InstallerLog.txt
                    continue
                else
                    echo -e "\n\n\n\t***Acceleration Build Complete***>\n\n"
                fi
            ;;

            "$PERFORM_ACTION_BUILD_SAMPLE" )
                SetEnvVariablesForBuild
                if (($invalidConfigDetected == "1")); then
                    ActionSelected=""
                    continue
                fi
                CreateBuildFolders
		if [ $BUILD_DC_ONLY == true ]; then
                   unset WITH_CPA_MUX
                   export DO_CRYPTO=0
                fi
                if [ -z $WITH_CPA_MUX ]; then
                   if [ $ICP_DRIVER_TYPE = $QAT16 ]; then 
                       SetENV_QAT16
                   fi
                   if [ $ICP_DRIVER_TYPE = $QAT15 ]; then 
                       SetENV_QAT15
                   fi     
                   RemoveLogFile
                   PrintAccSampleInstallOptions
                   sleep 3
                   BuildAccelSample 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                   InstallerOperationStatus=$?
                   echo -e "\n\n\n\t***Sample Code Build Complete***>\n\n"
                else
                    RemoveLogFile
                    SetENV_QAT16
                    PrintAccSampleInstallOptions
                    sleep 3
                    BuildAccelSample 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
                    InstallerOperationStatus=$?
                    echo -e "\n\n\n\t***Sample code Build With Mux Complete***>\n\n"
                fi
		if [ $BUILD_DC_ONLY == true ]; then
                   unset DO_CRYPTO
                fi
            ;;

           "$PERFORM_ACTION_HELP" )
               DisplayHelp
           ;;
           "$PERFORM_ACTION_VERSION" )
               DisplayVersion
           ;;
           
           "$PERFORM_ACTION_ACCEL_INFO" )
               echo -e "\n\t<****Acceleration Information****>\n\n"
                ProbeAccDeviceInfo
                PrintAccDeviceInfo
            ;;

           "$PERFORM_ACTION_PKG_DEPENDENCY_LIST" )
               echo -e "\n\t<****PACKAGE DEPENDENCY List****>\n\n"
                PkgDependList
            ;;

           * )
           ;;
        esac
        #If Running with command line arguments exit after install
        #==================================================
        if (( $CL_OPTIONS_COUNT != "0" )); then
            if [ "$InstallerOperationStatus" != "0" ];then
               echo -e "\n\n\n\t***installer operation status is Failure***>\n\n" | tee -a InstallerLog.txt
               exit 1
            else
               echo -e "\n\n\n\t***installer operation status is Success***>\n\n" | tee -a InstallerLog.txt
               exit 0
            fi
        fi
        #set umask back to original user setting
        umask $ORIG_UMASK
    done     
}

MainViewGeneral

