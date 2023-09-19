/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include "multi_pc_type.h"
#include "Serial.h"
#include <cerrno>
#include <cstring>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(): udp(8018, "192.168.123.3", 8017, sizeof(ArmState), sizeof(ArmCommand)),
    serial("/dev/ttyUSB0", 4800000) {}
    void UDPRecv();
    void UDPSend();
    void Calc();

    UDP udp;
    UDPState udpstate;
    ArmCommand command;
    ArmState state;
    float dt = 0.001;
    Serial serial;
    int validDataReceived = -3;;
};

void Custom::UDPRecv()
{
    validDataReceived = udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::Calc()
{
    udp.GetRecv((char*)&command);
    if (validDataReceived == 0){
    printf("%f\n", command.q_command[0]);
    std::cout << validDataReceived << std::endl;

    // Generate message and send
    uint8_t message[34];
    int motorid = 2;
    int operation_mode = 5;
    float torque = command.tau_command[0];
    float position = command.q_command[0];
    float velocity = command.dq_command[0];
    float kp = command.kp_command[0];
    float kd = command.kd_command[0];
    
    serial.generateMessage(motorid, operation_mode, torque, position, velocity, kp, kd, message);
    //serial.printMessage(message,34);
    
    try{
        serial.writeData(message, sizeof(message));
        std::cout << "Message sent successfully!" << std::endl;
    } catch(const std::exception& e){
        std::cerr << "An error occured: " << e.what() << std::endl;
    }
    
    // Reading and decoding the response message
    uint8_t response[78];
    ssize_t bytesRead = serial.readData(response, sizeof(response));
    std::cout << "Bytes read is " << bytesRead << std::endl;
    if (bytesRead == -1){
            std::cerr << "Error reading from serial port: " << strerror(errno) << std::endl;
    if(bytesRead == sizeof(response)){
        serial.decodeFromResponse(response, state.tauEst[0], state.dq_state[0], state.q_state[0]);
        }
    } else{
        std::cerr << "Failed to read the full response!" << std::endl;
    }

    state.yaw +=  1;
    state.pitch += 2;

    udp.SetSend((char*)&state);
    }
}

int main(void) 
{
    Custom custom;
    // InitEnvironment();
    LoopFunc loop_calc("calc_loop",   custom.dt,    boost::bind(&Custom::Calc,    &custom));
    LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
    LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_calc.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
