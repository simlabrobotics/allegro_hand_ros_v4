/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Wonik Robotics.
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

#ifndef __ALLEGROHAND_CANDRV_H__
#define __ALLEGROHAND_CANDRV_H__

/*
 *  @file candrv.h
 *  @brief API for communication over CAN bus
 *  @detailed The API for communicating with the various motor controllers
 *          over the CAN bus interface on the robot hand
 *
 *  Created on:         July 29, 2016
 *  Added to Project: 	July 29, 2016
 *  Author:             Sean Yi
 *  Maintained by:      Sean Yi(seanyi@wonikrobotics.com)
 */

#include "candef.h"

CANAPI_BEGIN

#ifndef FALSE
#define FALSE (0)
#endif
#ifndef TRUE
#define TRUE (1)
#endif

/*=====================*/
/*       Defines       */
/*=====================*/
//constants
#define TX_QUEUE_SIZE       (32)
#define RX_QUEUE_SIZE       (32)
#define TX_TIMEOUT          (5)
#define RX_TIMEOUT          (5)
#define MAX_BUS             (256)

/******************/
/* CAN device API */
/******************/

/**
 * @brief command_can_open_with_name
 * @param ch
 * @param dev_name
 * @return
 */
int command_can_open_with_name(void*& ch, const char* dev_name);

/**
 * @brief command_can_open
 * @param ch
 * @return
 */
int command_can_open(void* ch);

/**
 * @brief command_can_open_ex
 * @param ch
 * @param type
 * @param index
 * @return
 */
int command_can_open_ex(void* ch, int type, int index);

/**
 * @brief command_can_flush
 * @param ch
 * @return
 */
int command_can_flush(void* ch);

/**
 * @brief command_can_reset
 * @param ch
 * @return
 */
int command_can_reset(void* ch);

/**
 * @brief command_can_close
 * @param ch
 * @return
 */
int command_can_close(void* ch);

/**
 * @brief command_can_set_id
 * @param ch
 * @param can_id
 * @return
 */
int command_can_set_id(void* ch, unsigned char can_id);

/**
 * @brief command_servo_on
 * @param ch
 * @return
 */
int command_servo_on(void* ch);

/**
 * @brief command_servo_off
 * @param ch
 * @return
 */
int command_servo_off(void* ch);

/**
 * @brief command_set_torque
 * @param ch
 * @param findex
 * @param pwm
 * @return
 */
int command_set_torque(void* ch, int findex, short* pwm);

/**
 * @brief command_set_pose
 * @param ch
 * @param findex
 * @param jposition
 * @return
 */
int command_set_pose(void* ch, int findex, short* jposition);

/**
 * @brief command_set_period
 * @param ch
 * @param period Data period in millisecond. It is a array of which dimesion is 3. [0]:Joint positions, [1]:IMU, [2]temperature. If period is set to zero, periodic read will be stopped for that item.
 *               If it is null, all periodic communication will stop.
 * @return
 */
int command_set_period(void* ch, short* period);

/**
 * @brief command_set_device_id
 * @param ch
 * @param did
 * @return
 */
int command_set_device_id(void* ch, unsigned char did);

/**
 * @brief 
 * @param ch
 * @param baudrate
 * @return
 */
int command_set_rs485_baudrate(void* ch, unsigned int baudrate);

/**
 * @brief request_hand_information
 * @param ch
 * @return
 */
int request_hand_information(void* ch);

/**
 * @brief request_hand_serial
 * @param ch
 * @return
 */
int request_hand_serial(void* ch);

/**
 * @brief request_finger_pose
 * @param ch
 * @param findex [0,3]
 * @return
 */
int request_finger_pose(void* ch, int findex);

/**
 * @brief request_imu_data
 * @param ch
 * @return
 */
int request_imu_data(void* ch);

/**
 * @brief request_temperature
 * @param ch
 * @param sindex sensor index [0,3]
 * @return
 */
int request_temperature(void* ch, int sindex);

/**
 * @brief can_write_message
 * @param ch
 * @param id
 * @param len
 * @param data
 * @param blocking
 * @param timeout_usec
 * @return
 */
int can_write_message(void* ch, int id, int len, unsigned char* data, int blocking, int timeout_usec);

/**
 * @brief can_read_message
 * @param ch
 * @param id
 * @param len
 * @param data
 * @param blocking
 * @param timeout_usec
 * @return
 */
int can_read_message(void* ch, int* id, int* len, unsigned char* data, int blocking, int timeout_usec);

CANAPI_END

#endif // __ALLEGROHAND_CANDRV_H__
