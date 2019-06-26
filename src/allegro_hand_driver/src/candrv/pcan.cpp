/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Wonik Robotics.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Wonik Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  @file pcan.cpp
 *  @brief CAN API implementation to support PEAK CAN interface
 *
 *  Created on:         July 29, 2016
 *  Added to Project:   July 29, 2016
 *  Author:             Sean Yi
 *  Maintained by:      Sean Yi(seanyi@wonikrobotics.com)
 */

/*======================*/
/*       Includes       */
/*======================*/
//system headers
#include <stdio.h>
#include <errno.h>
#ifndef _WIN32
#include <inttypes.h>
#include <pthread.h>
#include <syslog.h>
#include <unistd.h>
#include <fcntl.h>
#else
#include <windows.h>
#endif
#include <malloc.h>
#include <assert.h>


typedef unsigned int DWORD;
typedef unsigned short WORD;
typedef char BYTE;
typedef void* LPSTR;

extern "C" {
#ifndef _WIN32
#include "libpcan/libpcan.h"
#else
#include "Peak/PCANBasic.h"
#endif
}
#include "candef.h"
#include "candrv.h"
#include "ros/ros.h" // for ROS_ERROR, ROS_INFO


CANAPI_BEGIN

/*=====================*/
/*       Defines       */
/*=====================*/
//macros
#define isAlpha(c) ( ((c >= 'A') && (c <= 'Z')) ? 1 : 0 )
#define isSpace(c) ( (c == ' ') ? 1 : 0 )
#define isDigit(c) ( ((c >= '0') && (c <= '9')) ? 1 : 0 )

//constants
#define NUM_OF_FINGERS          4 // number of fingers
#define NUM_OF_TEMP_SENSORS     4 // number of temperature sensors

/*=========================================*/
/*       Global file-scope variables       */
/*=========================================*/
unsigned char CAN_ID = 0;

/*==========================================*/
/*       Private functions prototypes       */
/*==========================================*/
int canReadMsg(void* ch, int *id, int *len, unsigned char *data, int blocking, int timeout_usec);
int canSendMsg(void* ch, int id, char len, unsigned char *data, int blocking, int timeout_usec);
int canSentRTR(void* ch, int id, int blocking, int timeout_usec);

/*========================================*/
/*       Public functions (CAN API)       */
/*========================================*/
int canInit(void* ch)
{
    int err;
    int i;
    char CanVersion[VERSIONSTRING_LEN];
    TPCANRdMsg CanMsg;

    err = CAN_VersionInfo(ch, CanVersion);
    if (err) {
        ROS_ERROR("CAN: Error in CAN_VersionInfo()");
    } else {
        ROS_INFO("CAN: Version info: %s", CanVersion);
    }

    ROS_INFO("CAN: Initializing device");
    // init to an user defined bit rate
    err = CAN_Init(ch, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
    if (err) {
        ROS_ERROR("CAN: Error in CAN_Init()");
        return err;
    }

    ROS_INFO("CAN: Clearing the CAN buffer");
    for (i = 0; i < 100; i++) {
        LINUX_CAN_Read_Timeout(ch, &CanMsg, 1000);
    }

    return 0; // PCAN_ERROR_OK
}

int canReadMsg(void* ch, int *id, int *len, unsigned char *data, int blocking, int timeout_usec){
    int err;
    int i;
    TPCANRdMsg CanMsg;

    if (blocking || timeout_usec < 0) {
        err = LINUX_CAN_Read(ch, &CanMsg);
    }
    else {
        err = LINUX_CAN_Read_Timeout(ch, &CanMsg, timeout_usec);
        if (CAN_ERR_QRCVEMPTY == err) // when receive queue is empty...
            return err;
    }

    if (err) {
        ROS_ERROR("CAN: CAN_Read() failed with error %x", err);
        return err;
    }

    *id = (CanMsg.Msg.ID & 0xfffffffc) >> 2;;
    *len = CanMsg.Msg.LEN;
    for(i = 0; i < CanMsg.Msg.LEN; i++)
        data[i] = CanMsg.Msg.DATA[i];

    return 0;
}

int canSendMsg(void* ch, int id, char len, unsigned char *data, int blocking, int timeout_usec){
    int err;
    int i;
    TPCANMsg CanMsg;
    CanMsg.ID = (id << 2) | CAN_ID;
    CanMsg.MSGTYPE = MSGTYPE_STANDARD;
    CanMsg.LEN = len & 0x0F;
    for(i = 0; i < len; i++)
        CanMsg.DATA[i] = data[i];

    if (blocking || timeout_usec < 0)
        err = CAN_Write(ch, &CanMsg);
    else
        err = LINUX_CAN_Write_Timeout(ch, &CanMsg, timeout_usec);

    if (err)
        ROS_ERROR("CAN: CAN_Write() failed with error %d", err);

    return err;
}

int canSentRTR(void* ch, int id, int blocking, int timeout_usec){
    int err;
    int i;
    TPCANMsg CanMsg;
    CanMsg.ID = (id << 2) | CAN_ID;
    CanMsg.MSGTYPE = MSGTYPE_RTR;
    CanMsg.LEN = 0;

    if (blocking || timeout_usec < 0)
        err = CAN_Write(ch, &CanMsg);
    else
        err = LINUX_CAN_Write_Timeout(ch, &CanMsg, timeout_usec);

    if (err)
        ROS_ERROR("CAN: CAN_Write() failed with error %d", err);

    return err;
}

/*========================================*/
/*       CAN API                          */
/*========================================*/
int command_can_open_with_name(void*& ch, const char* dev_name)
{
    int err;

    ROS_INFO("CAN: Opening device on channel [%s]", dev_name);
    ch = LINUX_CAN_Open(dev_name, O_RDWR);
    if (!ch) {
        ROS_ERROR("CAN: Error in CAN_Open() on channel %s", dev_name);
        return -1;
    }
    else {
        ROS_INFO("CAN: Channel %s(channel:=%d) is opend successfully", dev_name, *((int*)ch));
    }

    err = canInit(ch);
    return err;
}

int command_can_open(void* ch)
{
    ROS_ERROR("CAN: Error! Unsupported function call, can_open(void*&)");
    return -1;
}

int command_can_open_ex(void* ch, int type, int index)
{
    ROS_ERROR("CAN: Error! Unsupported function call, can_open(void*&, int, int)");
    return -1;
}

int command_can_flush(void* ch)
{
    int i;
    TPCANRdMsg CanMsg;

    for (i = 0; i < 100; i++) {
        LINUX_CAN_Read_Timeout(ch, &CanMsg, 1000);
    }

    return 0;
}

int command_can_reset(void* ch)
{
    return -1;
}

int command_can_close(void* ch)
{
    int err;

    err = CAN_Close(ch);
    if (err) {
        ROS_ERROR("CAN: Error in CAN_Close()");
        return -1;
    }

    return 0; // PCAN_ERROR_OK
}

int command_can_set_id(void* ch, unsigned char can_id)
{
    CAN_ID = can_id;
    return 0; //PCAN_ERROR_OK;
}

int command_servo_on(void* ch)
{
    long Txid;
    unsigned char data[8];
    int ret;

    Txid = ID_CMD_SYSTEM_ON;
    ret = canSendMsg(ch, Txid, 0, data, TRUE, 0);

    return ret;
}

int command_servo_off(void* ch)
{
    long Txid;
    unsigned char data[8];
    int ret;

    Txid = ID_CMD_SYSTEM_OFF;
    ret = canSendMsg(ch, Txid, 0, data, TRUE, 0);

    return ret;
}

int command_set_torque(void* ch, int findex, short* pwm)
{
    assert(findex >= 0 && findex < NUM_OF_FINGERS);

    long Txid;
    unsigned char data[8];
    int ret;

    if (findex >= 0 && findex < NUM_OF_FINGERS)
    {
        data[0] = (unsigned char)( (pwm[0]     ) & 0x00ff);
        data[1] = (unsigned char)( (pwm[0] >> 8) & 0x00ff);

        data[2] = (unsigned char)( (pwm[1]     ) & 0x00ff);
        data[3] = (unsigned char)( (pwm[1] >> 8) & 0x00ff);

        data[4] = (unsigned char)( (pwm[2]     ) & 0x00ff);
        data[5] = (unsigned char)( (pwm[2] >> 8) & 0x00ff);

        data[6] = (unsigned char)( (pwm[3]     ) & 0x00ff);
        data[7] = (unsigned char)( (pwm[3] >> 8) & 0x00ff);

        Txid = ID_CMD_SET_TORQUE_1 + findex;

        ret = canSendMsg(ch, Txid, 8, data, TRUE, 0);
    }
    else
        return -1;

    return ret;
}

int command_set_pose(void* ch, int findex, short* jposition)
{
    assert(findex >= 0 && findex < NUM_OF_FINGERS);

    long Txid;
    unsigned char data[8];
    int ret;

    if (findex >= 0 && findex < NUM_OF_FINGERS)
    {
        data[0] = (unsigned char)( (jposition[0]     ) & 0x00ff);
        data[1] = (unsigned char)( (jposition[0] >> 8) & 0x00ff);

        data[2] = (unsigned char)( (jposition[1]     ) & 0x00ff);
        data[3] = (unsigned char)( (jposition[1] >> 8) & 0x00ff);

        data[4] = (unsigned char)( (jposition[2]     ) & 0x00ff);
        data[5] = (unsigned char)( (jposition[2] >> 8) & 0x00ff);

        data[6] = (unsigned char)( (jposition[3]     ) & 0x00ff);
        data[7] = (unsigned char)( (jposition[3] >> 8) & 0x00ff);

        Txid = ID_CMD_SET_POSE_1 + findex;

        ret = canSendMsg(ch, Txid, 8, data, TRUE, 0);
    }
    else
        return -1;

    return ret;
}

int command_set_period(void* ch, short* period)
{
    long Txid;
    unsigned char data[8];
    int ret;

    Txid = ID_CMD_SET_PERIOD;
    if (period != 0)
    {
        data[0] = (unsigned char)( (period[0]     ) & 0x00ff);
        data[1] = (unsigned char)( (period[0] >> 8) & 0x00ff);
        data[2] = (unsigned char)( (period[1]     ) & 0x00ff);
        data[3] = (unsigned char)( (period[1] >> 8) & 0x00ff);
        data[4] = (unsigned char)( (period[2]     ) & 0x00ff);
        data[5] = (unsigned char)( (period[2] >> 8) & 0x00ff);
    }
    else
    {
        data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = 0x0;
    }
    ret = canSendMsg(ch, Txid, 6, data, TRUE, 0);

    return ret;
}

int command_set_device_id(void* ch, unsigned char did)
{
    long Txid;
    unsigned char data[8];
    int ret;

    Txid = ID_CMD_CONFIG;
    data[0] = did | 0x80;
    data[1] = 0x0;
    data[5] = 0x0;
    ret = canSendMsg(ch, Txid, 6, data, TRUE, 0);

    return ret;
}

int command_set_rs485_baudrate(void* ch, unsigned int baudrate)
{
    long Txid;
    unsigned char data[8];
    int ret;

    Txid = ID_CMD_CONFIG;
    data[0] = 0x0;
    data[1] = (unsigned char)( (baudrate      ) & 0x000000ff);
    data[2] = (unsigned char)( (baudrate >> 8 ) & 0x000000ff);
    data[3] = (unsigned char)( (baudrate >> 16) & 0x000000ff);
    data[4] = (unsigned char)( (baudrate >> 24) & 0x000000ff) | 0x80;
    data[5] = 0x0;
    ret = canSendMsg(ch, Txid, 6, data, TRUE, 0);

    return ret;
}

int request_hand_information(void* ch)
{
    long Txid = ID_RTR_HAND_INFO;
    int ret = canSentRTR(ch, Txid, TRUE, 0);

    return ret;
}

int request_hand_serial(void* ch)
{
    long Txid = ID_RTR_SERIAL;
    int ret = canSentRTR(ch, Txid, TRUE, 0);

    return ret;
}

int request_finger_pose(void* ch, int findex)
{
    assert(findex >= 0 && findex < NUM_OF_FINGERS);

    long Txid = ID_RTR_FINGER_POSE + findex;
    int ret = canSentRTR(ch, Txid, TRUE, 0);

    return ret;
}

int request_imu_data(void* ch)
{
    long Txid = ID_RTR_IMU_DATA;
    int ret = canSentRTR(ch, Txid, TRUE, 0);

    return ret;
}

int request_temperature(void* ch, int sindex)
{
    assert(sindex >= 0 && sindex < NUM_OF_TEMP_SENSORS);

    long Txid = ID_RTR_TEMPERATURE + sindex;
    int ret = canSentRTR(ch, Txid, TRUE, 0);

    return ret;
}

int can_write_message(void* ch, int id, int len, unsigned char* data, int blocking, int timeout_usec)
{
    int err;
    err = canSendMsg(ch, id, len, data, blocking, timeout_usec);
    return err;
}

int can_read_message(void* ch, int* id, int* len, unsigned char* data, int blocking, int timeout_usec)
{
    int err;
    err = canReadMsg(ch, id, len, data, blocking, timeout_usec);
    return err;
}

CANAPI_END
