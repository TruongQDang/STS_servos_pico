#include "STSServoDriver.hpp"

namespace instruction
{
        uint8_t const PING_ = 0x01;
        uint8_t const READ = 0x02;
        uint8_t const WRITE = 0x03;
        uint8_t const REGWRITE = 0x04;
        uint8_t const ACTION = 0x05;
        uint8_t const SYNCWRITE = 0x83;
        uint8_t const RESET = 0x06;
};

bool STSServoDriver::init(uart_inst_t *serialPort, long const &baudRate)
{
        if (serialPort == nullptr)
                return false;

        port_ = serialPort;
        uart_init(serialPort, baudRate);

        for (uint8_t servo_id = 0; servo_id < 0xFE; servo_id++) {
                if (ping(servo_id))
                        return true;
        }
        return false;
}

bool STSServoDriver::ping(uint8_t const &servoId)
{
        uint8_t response[1] = {0xFF};
        int send = sendMessage(servoId,
                               instruction::PING_,
                               0,
                               response);
        // // Failed to send
        // if (send != 6)
        //         return false;
        // // Read response
        // int rd = receiveMessage(servoId, 1, response);
        // if (rd < 0)
        //         return false;
        // return response[0] == 0x00;
}

bool STSServoDriver::sendMessage(uint8_t const &servoId, uint8_t const &commandID, uint8_t const &paramLength, uint8_t *parameters)
{
        uint8_t message[6 + paramLength];
        uint8_t checksum = servoId + paramLength + 2 + commandID;
        message[0] = 0xFF;
        message[1] = 0xFF;
        message[2] = servoId;
        message[3] = paramLength + 2;
        message[4] = commandID;
        for (uint32_t i = 0; i < paramLength; i++) {
                message[5 + i] = parameters[i];
                checksum += parameters[i];
        }
        message[5 + paramLength] = ~checksum;
        if (uart_is_writable(port_)) {
                uart_puts(port_, reinterpret_cast<const char *>(message));
                // Give time for the message to be processed.
                // delayMicroseconds(200);
                return true;
        }
        
        return false;
}
