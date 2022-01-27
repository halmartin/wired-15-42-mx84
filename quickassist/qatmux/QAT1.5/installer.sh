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
#  version: QAT1.5.L.1.11.0-36
#
###############################################################################

#Script Default Locations
#========================
INSTALL_DEFAULT="$PWD"
BUILD_OUTPUT_DEFAULT="build"
ICP_TOOLS_TARGET="accelcomp"
KERNEL_SOURCE_ROOT_DEFAULT="/usr/src/kernels/`\uname -r`"
KERNEL_CONFIG="$KERNEL_SOURCE_ROOT_DEFAULT/.config"
OS="`\uname -m`"
ORIG_UMASK=`umask`
MODULE_DIR="/lib/modules/`\uname -r`/kernel/drivers"
OBJ_LIST="icp_qa_al.ko"
VF_OBJ_LIST="icp_qa_al_vf.ko"
BIN_LIST="mof_firmware.bin mmp_firmware.bin
          mof_firmware_c2xxx.bin mmp_firmware_c2xxx.bin"
VF_BIN_LIST="mmp_firmware.bin"
ERR_COUNT=0
USE_WATCHDOG=false
InstallerOperationStatus=0 # Installer operation return value

#Copy Command not interactive
##################
alias cp='\cp'

#Command Line Arguments
#=========================
CL_INSTALL_OPTION_LIST=('a' 'h' 'ba' 'bs' 'host' 'vf')
CL_INSTALL_OPTION=$1
CL_INSTALL_LOCATION=$2
CL_KERNEL_LOCATION=$3

if [ $CL_INSTALL_OPTION ]; then
for i in "${CL_INSTALL_OPTION_LIST[@]}"; do
    if [ $CL_INSTALL_OPTION = $i ]; then
        case=$CL_INSTALL_OPTION
    fi
done

fi

printCmdLineHelp()
{
echo -e "\n
The Installer script takes the following command line arguments:
Usage: ./installer.sh <*What to Build*> <*Where to Build*> <*Kernel Source*>

 <What to Build> Parameter:
ba - Build Acceleration

 <Where to Build> Parameter:
Set the build location, for example, /tmp or $PWD.

 <Kernel Source> Parameter:
Set the kernel source here, for example, /usr/src/kernels/linux-3.1.0-7/.

 Example Usage:
./installer ba /tmp /usr/src/kernels/linux-3.1.0-7
\n
"
return
}


WelcomeMessage()
{
    if [ "$(id -u)" != "0" ]; then
        echo -e "\n\n\t==============================================="
        echo -e "\n\n\tERROR This script must be run as root"
        echo -e "\n\n\t===============================================\n\n"
        exit
    fi

   #Print Welcome Message
   #====================
   echo -e "\n\n\t==============================================="
   echo -e "\tWelcome to Acceleration Interactive Installer"
   echo -e "\t===============================================\n\n"

}



CreateBuildFolders()
{
   if [ $CL_INSTALL_LOCATION ]; then
      echo -e "\n\n\tInstall Location set through command line :  $CL_INSTALL_LOCATION \n\n"
      ICP_ROOT=$CL_INSTALL_LOCATION
      ICP_BUILD_OUTPUT="$ICP_ROOT/$BUILD_OUTPUT_DEFAULT"
      return
   fi

   #Read in Where to Untar/Build Package
   #================================
   echo -e "\tWhere Would you like to  Build the QAT Package?"
   echo -e "\tEnter to Accept Default [$INSTALL_DEFAULT]"
   read ICP_ROOT

   #Check if String is Null, If so use Default Value
   #================================================
   if [ -z $ICP_ROOT ]; then
   #echo "string is empty, apply Default"
   ICP_ROOT="$INSTALL_DEFAULT"
   #echo install dir : "$ICP_ROOT"
   else
   echo
        #echo "string is valid"
   #echo install dir : "$ICP_ROOT"
   fi

   #Create Directory
   #================
   if [ -e $ICP_ROOT ]; then
   echo
        #echo "This Directory Already Exists"
   #echo "OverWriting!!!"
   else
   echo
        #echo "Creating ICP_ROOT : $ICP_ROOT"
   fi

   mkdir -p "$ICP_ROOT"

   #Location of Build Output
   #================================
   echo -e "\tWhat would you like to call the Build Output Directory?"
   echo -e "\tEnter to Accept Default [$ICP_ROOT/$BUILD_OUTPUT_DEFAULT]"
   read ICP_BUILD_OUTPUT

   #Check if String is Null, If so use Default Value
   #================================================
   if [ -z $ICP_BUILD_OUTPUT ]; then
        #echo "string is empty, apply Default"
        ICP_BUILD_OUTPUT="$ICP_ROOT/$BUILD_OUTPUT_DEFAULT"
        #echo ICP_BUILD_OUTPUT  : "$ICP_BUILD_OUTPUT"
   else
        echo
        #echo "string is valid"
        #echo ICP_BUILD_OUTPUT : "$ICP_BUILD_OUTPUT"
   fi

   #Create Directory
   #================
   if [ -e $ICP_BUILD_OUTPUT ]; then
        echo
        #echo "This Directory Already Exists"
        #echo "OverWriting!!!"
   else
       echo
       #echo "Creating New Directory $ICP_BUILD_OUTPUT"
   fi

   #echo Creating ICP_BUILD_OUTPUT : "$ICP_BUILD_OUTPUT"
   mkdir -p "$ICP_BUILD_OUTPUT"
}

SetENV()
{
   export ICP_ROOT=$ICP_ROOT
   export ICP_BUILD_OUTPUT=$ICP_BUILD_OUTPUT
   export ICP_BUILDSYSTEM_PATH=$ICP_ROOT/quickassist/build_system/
   export ICP_TOOLS_TARGET=$ICP_TOOLS_TARGET
   export ICP_ENV_DIR=$ICP_ROOT/quickassist/build_system/build_files/env_files
   export ICP_NONBLOCKING_PARTIALS_PERFORM=1
}

SetKernel()
{
   if [ $CL_KERNEL_LOCATION ]; then
      echo -e "\n\n\tKernel Location set throug command line :  $CL_KERNEL_LOCATION \n\n"
      KERNEL_ROOT_SOURCE=$CL_KERNEL_LOCATION
      return
   fi

   #Location of Kernel Source
   #================================
   echo -e "\tWhere is the Kernel Source Located ?"
   echo -e "\tEnter to Accept Default [$KERNEL_SOURCE_ROOT_DEFAULT]"
   read KERNEL_SOURCE_ROOT

   #Check if String is Null, If so use Default Value
   #================================================
   if [ -z $KERNEL_SOURCE_ROOT ]; then
        #echo "string is empty, apply Default"
        KERNEL_SOURCE_ROOT="$KERNEL_SOURCE_ROOT_DEFAULT"
        #echo KERNEL_SOURCE_ROOT  : "$KERNEL_SOURCE_ROOT"
   else
        echo
        #echo "string is valid"
        #echo KERNEL_SOURCE_ROOT : "$KERNEL_SOURCE_ROOT"
   fi
}

BuildAccel()
{
    if [ -e $ICP_ROOT/quickassist/ ]; then
        echo "The quickassist directory has already been created"
        echo "Do not run the tar command for quickassist"
    else
        echo "The quickassist directory is not present, do the tar command for quickassist"
        #tar -zxof $ICP_ROOT/QAT.L.*.tar.gz
    fi

    PrintAccDeviceInfo

    cd $ICP_ROOT/quickassist
    touch lookaside/firmware/icp_qat_ae.uof
    touch lookaside/firmware/icp_qat_pke.mof
    if make; then
        cd $ICP_BUILD_OUTPUT
        ls -la
        #echo ""
    else
        echo -e "\n\t***Acceleration Build Failed***>"
        return 1
    fi
    return 0
}


PrintAccDeviceInfo()
{
    local status=1
    if [ -z $ICP_ROOT ]; then
        #echo "string is empty, apply Default"
        ICP_ROOT="$INSTALL_DEFAULT"
    fi

    qatDevices=`lspci -nd 8086: | egrep "1f18|0434|0442" | sed -e 's/ /-/g'`
    numQatDevice=`lspci -nd 8086: | egrep -c "1f18|0434|0442"`
    echo -e "\n\t****numQatDevice=$numQatDevice***\n"

    if [ $numQatDevice != "0" ]; then
        for qatDevice in $qatDevices ; do
            bdf=`echo $qatDevice | awk -F'-' '{print $1}'`
            echo -e "\tBDF=$bdf"
            did=`echo $qatDevice | awk -F'-' '{print $3}'`

            case $did in
            8086:1f18)  rid=`echo $qatDevice | awk -F'(' '{print $2}'`
                        case $rid in
                            rev-02*)
                                echo -e "\t***C2xxx B0 device detected***\n"
                            ;;
                            *)
                                echo -e "\t***C2xxx Ax device detected***\n"
                                export ICP_A0_C2XXX_SILICON=1
                            ;;
                        esac

                       status=0
                ;;

             8086:0434) rid=`echo $qatDevice | awk -F'(' '{print $2}'`
                case $rid in
                rev-2*) echo -e "\t***DH89xxCC C0 Stepping detected***\n"
                        status=0
                        ;;
                     *) echo -e "\t***Error: Unsupported DH89xxCC Stepping detected***\n"
                        ;;
                esac
                ;;
             8086:0442) echo -e "\t***DH89xxCC Virtual Function device detected***\n"
                        status=0
                ;;
            *) echo -e "\t***Error: Invalid device detected***\n"
               ;;
            esac
        done
    else
        echo -e "\t***Error: No Acceleration Device Detected***\n"
    fi
    return $status
}


BuildAccelSample()
{
    PrintAccDeviceInfo

    if [ -e $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code ]; then
        echo "The quickassist directory has already been created"
            echo "Do not run the tar command for quickassist"
        else
            echo "The quickassist directory is not present, do the tar command for quickassist"
            tar -zxof $ICP_ROOT/DH89xxCC_ACCEL.L.*.tar.gz
    fi

    cd $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code
    touch $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/performance/compression/calgary
    touch $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/performance/compression/calgary32
    touch $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/performance/compression/canterbury
    if [ ! -d "/lib/firmware/" ]; then
    	mkdir /lib/firmware/
    fi
    cp $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/performance/compression/calgary /lib/firmware/
    cp $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/performance/compression/calgary32 /lib/firmware/
  	cp $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/performance/compression/canterbury /lib/firmware/

    make perf_all
    InstallerOperationStatus=$?
    if [ "$InstallerOperationStatus" != "0" ];then
        echo -e "\nError: Sample code Build performance Failed***>\n\n" | tee -a InstallerLog.txt
        return 1
    fi

    if make ; then
        cd $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/build
        ls -la
            #echo ""
    else
        echo -e "\n\t***Acceleration Sample Code Build Failed***>"
        return 1
    fi
    return 0
}


InstallAccel()
{
    local INSTALL_TYPE=$1
    if [ $INSTALL_TYPE == "2" ]; then
        local dh89DeviceNumber="0442"
    else
        local dh89DeviceNumber="0434"
    fi

    local numDh89xxDevice=`lspci -nd 8086:$dh89DeviceNumber | grep -c $dh89DeviceNumber`
    local c2xxxDeviceNumber="1f18"
    local numC2xxxDevice=`lspci -nd 8086:$c2xxxDeviceNumber | grep -c $c2xxxDeviceNumber`
    local libDir="/lib"

    PrintAccDeviceInfo
    if [ "$?" -ne "0" ]; then
        echo -e "\n\t***Aborting Installation***>\n"
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
        echo "The quickassist directory is not present, do the tar command for quickassist"
        tar -zxof $ICP_ROOT/QAT.L.*.tar.gz
    fi
    cd $ICP_ROOT/quickassist
    make clean
    make 2>&1 | tee -a $ICP_ROOT/InstallerLog.txt; (exit ${PIPESTATUS[0]})

    if [ "$?" -ne "0" ] || [ `grep "Error" $ICP_ROOT/InstallerLog.txt | wc -l` != "0" ]; then
        echo -e "\n\t******** An error was detected in InstallerLog.txt file********\n"
        ERR_COUNT=1
        return 1
    else
        echo -e "\n\t*** No error detected in InstallerLog.txt file ***\n"
        ERR_COUNT=0
    fi
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
                    echo -e "\n\t***Warning unknown device found***\n"
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


    echo "Copy QAT firmware to /lib/firmware/"
    if [ $1 == "0" ]; then
        echo -e "normal install"
        FIRMWARE_LIST=$BIN_LIST
        ACC_DRIVER_LIST=$OBJ_LIST
    elif [ $1 == "1" ]; then

        echo -e "SR_IOV host install"
        FIRMWARE_LIST=$BIN_LIST
        ACC_DRIVER_LIST=$OBJ_LIST
    else
        echo -e "SR_IOV guest install"
        FIRMWARE_LIST=$VF_BIN_LIST
        ACC_DRIVER_LIST=$VF_OBJ_LIST
    fi

    if [ ! -d "/lib/firmware/" ]; then
    	mkdir /lib/firmware/
    fi
    for bin_obj in $FIRMWARE_LIST;
    do
        echo "Copying $bin_obj to /lib/firmware/$bin_obj"
        install -D -m 640 $bin_obj /lib/firmware/$bin_obj
    done

    # delete the existing kernel objects if they exist in /lib/modules

    for kern_obj in $ACC_DRIVER_LIST;
    do
        echo "Copying $kern_obj to $MODULE_DIR"
        install -D -m 640 $kern_obj $MODULE_DIR/icp_qa_al.ko
    done

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
                        echo -e "\n\t***Error: Invalid Dh89xx SKU detected***>\n\n"
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
				bdf=`lspci -nd 8086:$c2xxxDeviceNumber | grep $c2xxxDeviceNumber | awk '{print $1}' | sed -n ''$i'p'`
				busNum=`echo $bdf | awk -F: '{print $1}'`
				deviceNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $1}'`
				functionNum=`echo $bdf | awk -F: '{print $2}' | awk -F. '{print $2}'`
				c2xxxSku=`od -tx1 -Ax -j64 -N1 /proc/bus/pci/$busNum/$deviceNum.$functionNum | awk '{print $2}' | sed '/^$/d'`

				if [ $c2xxxSku == "0a" ] || [ $c2xxxSku == "06" ]; then
					echo "Copying ../quickassist/config/c2xxx_qa_dev0_single_ae.conf to /etc/c2xxx_qa_dev$j.conf"
					install -D -m 660 ../quickassist/config/c2xxx_qa_dev0_single_ae.conf /etc/c2xxx_qa_dev"$j".conf
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
            install -D -m 660 ../quickassist/config/dh89xxcc_qa_dev0.conf.iov /etc/dh89xxcc_qa_dev"$i".conf
        done
    else
        for (( i=0; i<$numDh89xxDevice; i++ ))
        do
            echo "Copying dh89xxcc_qa_dev0.conf.vm to /etc/dh89xxcc_qa_dev$i.conf"
            install -D -m 660 ../quickassist/config/dh89xxcc_qa_dev0.conf.vm /etc/dh89xxcc_qa_dev"$i".conf
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
    install -D -m 750 qat_service /etc/init.d
    install -D -m 750 adf_ctl /etc/init.d
    
    if [ $OS == "x86_64" ]; then
        if [ -d /lib64 ]; then
            libDir="/lib64"
        fi
    fi

    if [ $1 == "2" ]; then
        echo "Copying libicp_qa_al_vf_s.so to $libDir"
        cp libicp_qa_al_vf_s.so $libDir
        echo "Copying libicp_qa_al_vf.a to $libDir"
        cp libicp_qa_al_vf.a $libDir
        ln -sf $libDir/libicp_qa_al_vf_s.so $libDir/libicp_qa_al_s.so 
        ln -sf $libDir/libicp_qa_al_vf.a $libDir/libicp_qa_al.a
    else
        echo "Copying libicp_qa_al_s.so to $libDir"
        cp libicp_qa_al_s.so $libDir 
        echo "Copying libicp_qa_al.a to $libDir"
        cp libicp_qa_al.a $libDir
    fi
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
        chkconfig --add qat_service
        if [ "$numDh89xxDevice" -ne "0" ]; then
            if [ $USE_WATCHDOG == true ]; then
                chkconfig --add gige_watchdog_service
            fi
        fi
    elif [ -e /usr/sbin/update-rc.d ]; then
        update-rc.d qat_service defaults
        if [ "$numDh89xxDevice" -ne "0" ]; then
            if [ $USE_WATCHDOG == true ]; then
                update-rc.d gige_watchdog_service defaults
            fi
        fi
    else
        echo "\n\t***Failed to add qat_service to start-up***>"
        return 1
    fi
    echo "Starting QAT service"
    /etc/init.d/qat_service start
    if [ $USE_WATCHDOG == true ]; then
        echo "Starting GigE watchdog service"
        /etc/init.d/gige_watchdog_service start
    fi
    /etc/init.d/qat_service status
    return 0
}

UninstallAccel()
{
    local VF_KO_MODULE="icp_qa_al_vf"
    local QAT_MEM_DRV="qaeMemDrv"

    # Remove the qaeMemDrv.ko file if it is loaded.
    if [ `lsmod | grep -o ^$QAT_MEM_DRV` ]; then
        rmmod -f qaeMemDrv
    fi

    echo "Unloading QAT kernel object"
    echo "Removing startup scripts"
    # Check if one of the /etc/init.d script exist
    if [ -e /etc/init.d/qat_service ]; then
        if [ -e /etc/init.d/gige_watchdog_service ]; then
            ret=`/etc/init.d/gige_watchdog_service stop`
        fi
        /etc/init.d/qat_service shutdown

        if [ -e /sbin/chkconfig ] ; then
            ret=`chkconfig --del gige_watchdog_service`
            ret=`chkconfig --del qat_service`
        elif [ -e /usr/sbin/update-rc.d ]; then
            ret=`update-rc.d -f gige_watchdog_service remove`
            ret=`update-rc.d -f qat_service remove`
        else
            echo "\n\t***Failed to remove qat_service from start-up***>"
            return 1
        fi
        /bin/rm -f /etc/init.d/gige_watchdog_service
        /bin/rm -f /etc/init.d/icp_gige_watchdog
        /bin/rm -f /etc/init.d/qat_service
        /bin/rm -f /etc/init.d/adf_ctl

    fi

    echo "Removing the QAT firmware"
    for bin_obj in $BIN_LIST
    do
        if [ -e /lib/firmware/$bin_obj ]; then
        /bin/rm -f /lib/firmware/$bin_obj
        fi
    done

    echo "Removing kernel objects from $MODULE_DIR"
    for kern_obj in $OBJ_LIST
    do
        if [ -e $MODULE_DIR/$kern_obj ]; then
            /bin/rm -f $MODULE_DIR/$kern_obj
            # In case the driver is a location that previous versions of the
            # installer placed it then attempt to remove it
            /bin/rm -f $MODULE_DIR/../../$kern_obj
         fi
    done
    # Remove the icp_qa_al_vf.ko file if it is loaded.
    if [ `lsmod | grep -o ^$VF_KO_MODULE` ]
    then
        `rmmod -f $VF_KO_MODULE`
    fi
    echo "Removing device permissions rules"
    rm -f /etc/udev/rules.d/00-dh89xxcc_qa.rules
    rm -f /etc/udev/rules.d/00-c2xxx_qa.rules
    echo "Rebuilding the module.dep file, this will take a few moments"
    /sbin/depmod -a
    rm -f /lib/libicp_qa_al_s.so
    rm -f /lib/libicp_qa_al.a
    rm -f /lib/libosal.a
    rm -f /lib/libadf_proxy.a
    rm -f /lib/libicp_qa_al_vf_s.so
    rm -f /lib/libicp_qa_al_vf.a
    rm -f /lib64/libicp_qa_al_s.so
    rm -f /lib64/libicp_qa_al.a
    rm -f /lib64/libosal.a
    rm -f /lib64/libadf_proxy.a
    rm -f /lib64/libicp_qa_al_vf_s.so
    rm -f /lib64/libicp_qa_al_vf.a
    if [ -e $MODULE_DIR/icp_qa_al_vf.ko ]; then
        for kern_obj in $VF_OBJ_LIST
        do
            if [ -e $MODULE_DIR/$kern_obj ]; then
                /bin/rm -f $MODULE_DIR/$kern_obj
            fi
        done
    fi

    echo -e "\n\n\n\n\t***************************************"
    echo -e "\n\t*** Acceleration Uninstall Complete ***>"
    echo -e "\n\t***************************************\n\n\n\n\n"
    return 0
}

RemoveLogFile()
{
    echo -e "\n\n\n\t**********************************************"
    echo -e "\t* Removing log file in Build Location : $ICP_ROOT"
    rm -f $ICP_ROOT/InstallerLog.txt
    echo -e "\t************************************************\n\n\n\n\n"

}

PrintInstallOtions()
{
    echo -e "\n\n\n\t**********************************************"
    echo -e "\t* Build Location : $ICP_ROOT"
    echo -e "\t* Build Output Location : $ICP_BUILD_OUTPUT"
    if [ -n "$KERNEL_SOURCE_ROOT" ]; then
        echo -e "\t* Kernel Source Location : $KERNEL_SOURCE_ROOT"
    fi
    echo -e "\t************************************************\n\n\n\n\n"

}

PrintAccSampleInstallOptions()
{
    echo -e "\n\n\n\t**********************************************"
    echo -e "\t* Build Location : $ICP_ROOT"
    echo -e "\t* Build Location : $ICP_ROOT/quickassist/lookaside/access_layer/src/sample_code/build"
    if [ -n "$KERNEL_SOURCE_ROOT" ]; then
        echo -e "\t* Kernel Source Location : $KERNEL_SOURCE_ROOT"
    fi
    echo -e "\t************************************************\n\n\n\n\n"

}


UninstallOptions()
{
    echo -e "\n\t***Uninstalling Acceleration***>\n\n"
    UninstallAccel
    if [ "$?" -ne "0" ];then
       echo -e "\n\n\n\t***Acceleration Uninstall Failed***>\n\n"
       return 1
    else
       echo -e "\n\n\n\t***Acceleration Uninstall Complete***>\n\n"
       return 0
    fi
}

LogFileHeader()
{
    echo -e "\n\n\n ****** `date`\t*********" 2>&1 > InstallerLog.txt
    echo -e " ************************************************\n" 2>&1 > InstallerLog.txt
}

GetKernelIOVConfig()
{
    local status
    if [ `cat $KERNEL_CONFIG | grep CONFIG_PCI_IOV | cut -d'=' -f2` == "y" ]; then
        echo -e "\n\t**** Kernel supports PCI_IOV ****\n\n"
        status=0
    else
        echo -e "\n\t**** Kernel does not support PCI_IOV ****\n\n"
        status=1
    fi
    return $status
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
        echo -e "\t Please Select Install Option :"
        echo -e "\t ----------------------------"
        echo -e "\t 1  Build Acceleration"
        echo -e "\t 2  Install Acceleration"
        echo -e "\t 3  Install SR-IOV Host Acceleration (DH89xx only)"
        echo -e "\t 4  Install SR-IOV Guest Acceleration (DH89xx only)"
        echo -e "\t 5  Show Acceleration Device Information"
        echo -e "\t 6  Build Acceleration Sample Code"
        if [ $USE_WATCHDOG == true ]; then
            echo -e "\t 7  Toggle GigE Watchdog for Acceleration install (Enabled)"
        else
            echo -e "\t 7  Toggle GigE Watchdog for Acceleration install (Disabled)"
        fi
        echo -e "\t 8  Uninstall"
        echo -e "\t 9  Exit"
        echo -e "\n"
        read case
      fi
    case $case in

      1 | ba)
        echo -e "\n\t****Building Acceleration****>\n\n"
        CreateBuildFolders
        SetENV
        RemoveLogFile
        PrintInstallOtions
        sleep 3
        BuildAccel 2>&1 | tee -a InstallerLog.txt; (exit ${PIPESTATUS[0]})
        InstallerOperationStatus=$?
        echo -e "\n\n\n\t***Acceleration Build Complete with $InstallerOperationStatus***>\n\n"

      ;;

      2 | a)
        echo -e "\n\t****Installing Acceleration****>\n\n"
        CreateBuildFolders
        SetENV
        RemoveLogFile
        PrintInstallOtions
        sleep 3
        InstallAccel 0
        InstallerOperationStatus=$?

        echo -e "\n\n\n\t***Acceleration Installation Complete***>\n\n"

      ;;

      3 | host )
        echo -e "\n\t****Installing SRV-IOV Host Acceleration****>\n\n"
        if GetKernelIOVConfig; then
            CreateBuildFolders
            SetENV
            export ICP_SRIOV=1
            RemoveLogFile
            PrintInstallOtions
            sleep 3
            InstallAccel 1
            InstallerOperationStatus=$?
            unset ICP_SRIOV
            echo -e "\n\n\n\t***Acceleration Installation Complete***>\n\n"
        else
            echo -e "\n\n\n\t IOV is not supported in this kernel version \n\n"
            echo -e " \n Disable SRIOV to build the driver \n\n"
        fi

      ;;

      4 | vf )
        echo -e "\n\t****Installing SRV-IOV Guest Acceleration****>\n\n"
        if GetKernelIOVConfig; then
            CreateBuildFolders
            SetENV
            export ICP_SRIOV=1
            RemoveLogFile
            PrintInstallOtions
            sleep 3
            InstallAccel 2
            InstallerOperationStatus=$?
            echo -e "\n\n\n\t***Acceleration Installation Complete***>\n\n"
            unset ICP_SRIOV
        else
            echo -e "\n\n\n\t IOV is not supported in this kernel version \n\n"
            echo -e " \n Disable SRIOV to build the driver \n\n"
        fi

       ;;


      5 )
        echo -e "\n\t****Acceleration Information****>\n\n"
        SetENV
        PrintAccDeviceInfo

      ;;

      6 | bs)
        echo -e "\n\t****Building Acceleration Sample Code****>\n\n"
        CreateBuildFolders
        SetENV
        RemoveLogFile
        PrintAccSampleInstallOptions
        sleep 3
        BuildAccelSample 2>&1 | tee -a InstallerLog.txt
        InstallerOperationStatus=$?
        echo -e "\n\n\n\t***Acceleration Sample Code Build Complete***>\n\n"

      ;;

      7)
        if [ $USE_WATCHDOG == true ]; then
            USE_WATCHDOG=false
        else
            USE_WATCHDOG=true
        fi
      ;;

      8)
        echo -e "\n\t****Uninstall****>\n\n"
        UninstallOptions
        InstallerOperationStatus=$?

      ;;

      9)
        echo -e "\n\t****EXITING INSTALLER****>\n\n"
        if [ "$InstallerOperationStatus" != "0" ];then
           echo -e "\nInstaller operation status is Failure***>\n\n" | tee -a InstallerLog.txt
           exit 1
        else
           echo -e "\n\n\n\t***installer operation status is Success***>\n\n" | tee -a InstallerLog.txt
           exit 0
        fi
      ;;
      0)
        echo -e "\n\t****EXITING INSTALLER****>\n\n"
        if [ "$InstallerOperationStatus" != "0" ];then
           echo -e "\nInstaller operation status is Failure***>\n\n" | tee -a InstallerLog.txt
           exit 1
        else
           echo -e "\n\n\n\t***installer operation status is Success***>\n\n" | tee -a InstallerLog.txt
           exit 0
        fi
      ;;

      h | -h)
        echo -e "\b\b\t\t================="
        echo -e "\t\tCommand Line Help"
        echo -e "\t\t================="
        printCmdLineHelp
        exit
      ;;

      *)
        echo -e "\n\t****Invalid Option****>"
        echo -e "\n\t****Please Choose Again****>\n\n"
      ;;
    esac

    #If Running with command line arguments exit after install
    #==================================================
    if [ $CL_INSTALL_OPTION ]; then
      exit
    fi
    #set umask back to original user setting
    umask $ORIG_UMASK
    done
}


MainViewGeneral

