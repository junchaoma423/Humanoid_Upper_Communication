/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#ifndef _MULTI_PC_TYPE_H_
#define _MULTI_PC_TYPE_H_

#include <stdint.h>

struct ArmCommand{
    int direction;
    float deepth;
    float q_command[4];
    float dq_command[4];
    float tau_command[4];
    float kp_command[4];
    float kd_command[4];
    float q_command_1;
    uint32_t crc;
};

struct ArmState{
    float yaw;
    float pitch;
    float roll;
    float q_state[4];
    float dq_state[4];
    float tauEst[4];
    uint32_t crc;
};

#endif
