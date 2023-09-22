/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include "multi_pc_type.h"
#include "Serial.h"
#include <cerrno>
#include <cstring>
#include <chrono>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(): udp(8018, "192.168.123.3", 8017, sizeof(ArmState), sizeof(ArmCommand)),
    serial1("/dev/ttyUSB0", 4800000),serial2("/dev/ttyUSB1", 4800000) {}
    void UDPRecv();
    void UDPSend();
    void Calc();
    void DecodeMessage();
    uint8_t message[34];
    int disconnect_counter = 0;

    UDP udp;
    UDPState udpstate;
    ArmCommand command;
    ArmState state;
    float dt = 0.001;
    Serial serial1;
    Serial serial2;
    int validDataReceived = -3;
    float torque = 0;
    float position = 0;
    float velocity = 0;
    float kp = 0;
    float kd = 0;
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
    uint8_t message[34];
    int id_feedback;
    auto start = std::chrono::high_resolution_clock::now();
    udp.GetRecv((char*)&command);
    if (validDataReceived == 0){

    // Generate message 1 and send to 1st serial port
    for (int motor=0; motor < 2; motor ++){
        // int motorid = 2;
        int operation_mode = 10;
        
        std::cout << "tau_command is " << command.tau_command[motor] << std::endl;
        std::cout << "Position command is " << command.q_command[motor] << std::endl;
        std::cout << "kp command is " << command.kp_command[motor] << std::endl;
        torque = command.tau_command[motor];
        position = command.q_command[motor];
        velocity = command.dq_command[motor];
        kp = command.kp_command[motor];
        kd = command.kd_command[motor];
        
        serial1.generateMessage(motor, operation_mode, torque, position, velocity, kp, kd, message);
        serial1.printMessage(message,34);
        
        try{
            serial1.writeData(message, sizeof(message));
        } catch(const std::exception& e){
            std::cerr << "An error occured: " << e.what() << std::endl;
        }
    }
    if (disconnect_counter > 10){
        serial1.generateMessage(0xbb, 0, 0.0, 0.0, 0.0, 0.0, 0.0, message);
        serial1.writeData(message, sizeof(message));
    }

    // Generate message 2 and send to 1st serial port
    for (int motor=2; motor < 4; motor ++){
        // int motorid = 2;
        int operation_mode = 10;
        
        std::cout << "tau_command is " << command.tau_command[motor] << std::endl;
        std::cout << "Position command is " << command.q_command[motor] << std::endl;
        std::cout << "kp command is " << command.kp_command[motor] << std::endl;
        float torque = command.tau_command[motor];
        float position = command.q_command[motor];
        float velocity = command.dq_command[motor];
        float kp = command.kp_command[motor];
        float kd = command.kd_command[motor];
        
        serial2.generateMessage(motor, operation_mode, torque, position, velocity, kp, kd, message);
        serial2.printMessage(message,34);
        
        try{
            serial2.writeData(message, sizeof(message));
        } catch(const std::exception& e){
            std::cerr << "An error occured: " << e.what() << std::endl;
        }
    }
    if (disconnect_counter > 10){
        serial2.generateMessage(0xbb, 0, 0.0, 0.0, 0.0, 0.0, 0.0, message);
        serial2.writeData(message, sizeof(message));
    }
    
    // Reading and decoding the response message
    uint8_t response[78];
    ssize_t bytesRead = serial1.readData(response, sizeof(response));
    
    std::cout << "bytesRead is " << bytesRead << std::endl;
    
    if (bytesRead == sizeof(response)){
        std::cout << "Full Response received from the motor: " << std::endl;
            serial1.decodeMessage(response, id_feedback, state.tauEst[0], state.dq_state[0], state.q_state[0]);
    }
    
    std::cout << "Position is " << state.q_state[0] << std::endl;


    disconnect_counter = 0;
    } else {
    disconnect_counter++;
    }
    
    udp.SetSend((char*)&state);

    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Encode Loop took: " << duration.count() << "microseconds" << std::endl;
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
