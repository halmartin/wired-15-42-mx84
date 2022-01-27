/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or 
 *   redistributing this file, you may do so under either license.
 * 
 *   GPL LICENSE SUMMARY
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
 *   BSD LICENSE 
 * 
 *   Copyright(c) 2007-2013 Intel Corporation. All rights reserved.
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without 
 *   modification, are permitted provided that the following conditions 
 *   are met:
 * 
 *     * Redistributions of source code must retain the above copyright 
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright 
 *       notice, this list of conditions and the following disclaimer in 
 *       the documentation and/or other materials provided with the 
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its 
 *       contributors may be used to endorse or promote products derived 
 *       from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 *  version: QAT1.5.L.1.11.0-36
 *
******************************************************************************/

/******************************************************************************
 * @file adf_gige_watchdog.c
 *
 * @description
 * GigE watchdog restarts gigE interfaces on a given dh89xxCC device during
 * QAT hardware reset.
 *
******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>
#include <signal.h>
#include <syslog.h>
#include <icp_accel_devices.h>
#include <icp_platform.h>

#define WD_DEVICE_NAME "/dev/icp_adf_gige_wd"
#define DEV_PATH       "/sys/bus/pci/devices/0000:%02d:00.%d/net"
#define DOWN_CMD       "ifconfig %s down"
#define UP_CMD         "ifconfig %s up"
#define DEV_PATH_LEN   64
#define DEV_NAME_LEN   32
#define DEV_CMD_LEN    64
#define NUM_OF_DEVICES 4

static int running = 1;
static void ctrl_c(int sig)
{
    syslog(LOG_INFO, "QAT GigE watchdog exiting\n");
    running = 0;
}

static int bring_down_netdev(const char *dev)
{
    char cmd[DEV_CMD_LEN] = {0};

    syslog(LOG_ALERT, "QAT GigE watchdog stopping %s\n", dev);
    snprintf(cmd, DEV_CMD_LEN, DOWN_CMD, dev);
    return system(cmd);
}

static int bring_up_netdev(const char *dev)
{
    char cmd[DEV_CMD_LEN] = {0};

    syslog(LOG_ALERT, "QAT GigE watchdog starting %s\n", dev);
    snprintf(cmd, DEV_CMD_LEN, UP_CMD, dev);
    return system(cmd);
}

static int get_netdev_name(const char *dev_path, char *dev)
{
    struct dirent *ent = NULL;
    DIR *dir = opendir(dev_path);
    int found = 0;

    if (!dir) {
        syslog(LOG_ERR, "Could not open %s\n", dev_path);
        return -1;
    }
    do {
        ent = readdir(dir);
        if (!ent) {
            syslog(LOG_ERR, "Could not read %s\n", dev_path);
            closedir(dir);
            return -1;
        }
        if (strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0) {
            strcpy(dev, ent->d_name);
            found = 1;
            break;
        }
    } while(found == 0);
    closedir(dir);
    return 0;
}

int main(int argc, char **argv)
{
    char gige_dev_name[NUM_OF_DEVICES][DEV_NAME_LEN] = {{0}};
    char gige_dev_path[DEV_PATH_LEN] = {0};
    int ret = 0, i = 0;
    Cpa32U bus = 0;
    int fhdl = 0;

    signal(SIGINT, ctrl_c);
    signal(SIGUSR1, ctrl_c);
    syslog(LOG_INFO, "QAT GigE watchdog started. "
                     "To close send SIGINT or SIGUSR1\n");
    while(running)
    {
        do {
            fhdl = open(WD_DEVICE_NAME, O_RDWR);
            if (fhdl < 0) {
                sleep(1);
            }
        } while (running && fhdl < 0);

        while(running && (!(fhdl < 0)))
        {
            ret = read(fhdl, &bus, sizeof(Cpa32U));
            if (sizeof(Cpa32U) == ret) {
                for (i = 1; i < NUM_OF_DEVICES + 1; i++) {
                    snprintf(gige_dev_path, DEV_PATH_LEN, DEV_PATH, bus, i);
                    get_netdev_name(gige_dev_path, gige_dev_name[i-1]);
                    bring_down_netdev(gige_dev_name[i-1]);
                }
                ret = write(fhdl, &bus, sizeof(bus));
                sleep(1);
                for (i = 1; i < NUM_OF_DEVICES + 1; i++) {
                    bring_up_netdev(gige_dev_name[i-1]);
                }
            }
            bus = 0;
        }
    }
    close(fhdl);
    return 0;
}
