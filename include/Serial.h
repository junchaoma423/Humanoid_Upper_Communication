#ifndef SERIAL_H_
#define SERIAL_H_

#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <cstdint>
#include <iostream>
#include <cstring>

class Serial {
public:
    Serial(std::string path, int baudrate);
    virtual ~Serial();
    void generateMessage(int motorID, int operationMode, float torque, float position, float velocity, float kp, float kd, uint8_t* message);
    void printMessage(const uint8_t* message, size_t size);
    // void sendMessage();
    void writeData(const uint8_t* data, size_t size);
    
    ssize_t readData(uint8_t* buffer, size_t size);
    void decodeFromResponse(uint8_t* response, float& torqueActual, float& speedActual, float& positionActual);

private:
    uint32_t crc32_core(uint32_t* ptr, uint32_t len);

    int fd;
    struct termios2 ntio;
    uint8_t message[34];
    uint8_t response[78];
};

#endif /* SERIAL_H_ */
