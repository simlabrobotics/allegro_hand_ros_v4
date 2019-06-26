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
 *  @file candef.h
 *  @brief definition of constants used by CAN API
 *
 *  Created on:         July 29, 2016
 *  Added to Project: 	July 29, 2016
 *  Author:             Sean Yi
 *  Maintained by:      Sean Yi(seanyi@wonikrobotics.com)
 */

#ifndef __ALLEGROHAND_CANDEF_H__
#define __ALLEGROHAND_CANDEF_H__

#ifdef USING_NAMESPACE_CANAPI
#   define CANAPI_BEGIN namespace CANAPI {
#   define CANAPI_END };
#else
#   define CANAPI_BEGIN
#   define CANAPI_END
#   define CANAPI
#endif


CANAPI_BEGIN

////////////////////////////////////////////////
//  Define CAN Command
#define ID_CMD_SYSTEM_ON                0x40
#define ID_CMD_SYSTEM_OFF               0x41
#define ID_CMD_SET_TORQUE               0x60
#define ID_CMD_SET_TORQUE_1             (ID_CMD_SET_TORQUE+0)
#define ID_CMD_SET_TORQUE_2             (ID_CMD_SET_TORQUE+1)
#define ID_CMD_SET_TORQUE_3             (ID_CMD_SET_TORQUE+2)
#define ID_CMD_SET_TORQUE_4             (ID_CMD_SET_TORQUE+3)
#define ID_CMD_SET_POSE_1               0xE0
#define ID_CMD_SET_POSE_2               0xE1
#define ID_CMD_SET_POSE_3               0xE2
#define ID_CMD_SET_POSE_4               0xE3
#define ID_CMD_SET_PERIOD               0x81
#define ID_CMD_CONFIG                   0x68

////////////////////////////////////////////////
//  Define CAN Data Reqeust (RTR)
#define ID_RTR_HAND_INFO                0x80
#define ID_RTR_SERIAL                   0x88
#define ID_RTR_FINGER_POSE              0x20
#define ID_RTR_FINGER_POSE_1            (ID_RTR_FINGER_POSE+0)
#define ID_RTR_FINGER_POSE_2            (ID_RTR_FINGER_POSE+1)
#define ID_RTR_FINGER_POSE_3            (ID_RTR_FINGER_POSE+2)
#define ID_RTR_FINGER_POSE_4            (ID_RTR_FINGER_POSE+3)
#define ID_RTR_IMU_DATA                 0x30
#define ID_RTR_TEMPERATURE              0x38
#define ID_RTR_TEMPERATURE_1            (ID_RTR_TEMPERATURE+0)
#define ID_RTR_TEMPERATURE_2            (ID_RTR_TEMPERATURE+1)
#define ID_RTR_TEMPERATURE_3            (ID_RTR_TEMPERATURE+2)
#define ID_RTR_TEMPERATURE_4            (ID_RTR_TEMPERATURE+3)


CANAPI_END

#endif // __ALLEGROHAND_CANDEF_H__

