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
 *  @file AllegroHandDrv.h
 *  @brief Allegro Hand Driver
 *
 *  Created on:         July 29, 2016
 *  Added to Project:   July 29, 2016
 *  Author:             Sean Yi
 *  Maintained by:      Sean Yi(seanyi@wonikrobotics.com)
 */
#ifndef __ALLEGROHAND_DRV_H__
#define __ALLEGROHAND_DRV_H__

#include <fcntl.h>
#include <list>
#include <string>
#include "AllegroHandDef.h"

namespace allegro
{

class AllegroHandDrv
{
public:
    AllegroHandDrv();
    ~AllegroHandDrv();

    bool init(int mode = 0);                ///< initialize Allegro Hand driver and CAN channel

    void setTorque(double *torque);         ///< set desired joint torque
    void getJointInfo(double *position);    ///< get current joint position

    bool emergencyStop() { return _emergency_stop; }        ///< whether emergency is activated
    double torqueConversion() { return _tau_cov_const; }    ///< get torque conversion constant
    double inputVoltage() { return _input_voltage; }        ///< get input voltage of this system

    int readCANFrames();                    ///< try to read CAN frames (user code should call this fast enough)
    int writeJointTorque();                 ///< send joint command via CAN comm
    bool isJointInfoReady();                ///< return whether all joint positions are updated
    void resetJointInfoReady();             ///< reset joint position update flag

private:
    void* _can_handle;                      ///< CAN device(driver) handle

    double _curr_position[DOF_JOINTS];      ///< current joint position (radian)
    double _curr_torque[DOF_JOINTS];        ///< current joint torque (Nm)
    double _desired_position[DOF_JOINTS];   ///< desired joint position (radian)
    double _desired_torque[DOF_JOINTS];     ///< desired joint torque (Nm)

    double _hand_version;                   ///< hand version
    double _tau_cov_const;                  ///< constant to convert joint torque to pwm command
    double _input_voltage;                  ///< input voltage

    int _curr_position_get;                 ///< bit flag telling which joint positions are updated (0x01:index 0x02:middle 0x04:pinky 0x08:thumb)

    double _pwm_max_global;                 ///< global max value of PWM command is limited by the input voltage
    double _pwm_max[DOF_JOINTS];            ///< max value of PWM command of each joint
    int    _encoder_offset[DOF_JOINTS];     ///< encoder offset
    int    _encoder_direction[DOF_JOINTS];  ///< encoder direction
    int    _motor_direction[DOF_JOINTS];    ///< motor direction

    volatile bool _emergency_stop;          ///< something goes wrong?

private:
    void _readDevices();                    ///< read CAN messages
    void _writeDevices();                   ///< write CAN messages(torque command)
    //void _parseMessage(char cmd, char src, char des, int len, unsigned char* data); ///< parse CAN messages and calculate current joint angles from encoder values
    void _parseMessage(int id, int len, unsigned char* data);
};

}

#endif // __ALLEGROHAND_DRV_H__
