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
#include <libserialport.h>

class Serial {
public:
    Serial(std::string path, int baudrate);
    virtual ~Serial();
    void generateMessage(int motorID, int operationMode, float torque, float position, float velocity, float kp, float kd, uint8_t* message);
    void printMessage(const uint8_t* message, size_t size);
    // void sendMessage();
    //ssize_t writeData(const uint8_t* buffer, size_t length);
    void writeData(const uint8_t* data, size_t size);
    ssize_t readDataWithTimeout(uint8_t* buffer, size_t size, int timeoutMs);
    void decodeMessage(const uint8_t* response, int &id, float &tauEst, float &speed, float &position);

private:
    uint32_t crc32_core(uint32_t* ptr, uint32_t len);

    int fd;
    struct termios2 ntio;
    struct sp_port *port;
};

#endif /* SERIAL_H_ */
