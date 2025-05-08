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

STSServoDriver::STSServoDriver()
{
}

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

bool STSServoDriver::ping(uint8_t const &servo_id)
{
        uint8_t response[1] = {0xFF};
        bool sent = sendMessage(servo_id,
                               instruction::PING_,
                               0,
                               response);
        // Failed to send
        if (!sent) {
                return false;
        }
        // Read response
        int8_t rd = receiveMessage(
                servo_id,
                1,
                response);
        if (rd < 0) {
                return false;
        }

        return response[0] == 0x00;
}

bool STSServoDriver::sendMessage(
        uint8_t const &servo_id, 
        uint8_t const &command_id, 
        uint8_t const &param_length, 
        uint8_t *parameters)
{
        uint8_t message[6 + param_length];
        uint8_t checksum = servo_id + param_length + 2 + command_id;
        message[0] = 0xFF;
        message[1] = 0xFF;
        message[2] = servo_id;
        message[3] = param_length + 2;
        message[4] = command_id;
        for (uint32_t i = 0; i < param_length; i++) {
                message[5 + i] = parameters[i];
                checksum += parameters[i];
        }
        message[5 + param_length] = ~checksum;
        if (uart_is_writable(port_)) {
                uart_puts(port_, reinterpret_cast<const char *>(message));
                return true;
        }
        
        return false;
}

int8_t STSServoDriver::receiveMessage(uint8_t const &servo_id, uint8_t const &read_length, uint8_t *output_buffer)
{
        uint8_t result[read_length + 5];
        // 500ms timeout
        if (!uart_is_readable_within_us(port_, 500000)) {
                return -1;
        }
        uart_read_blocking(port_, output_buffer, 5 + read_length);
        // Check message integrity
        if (result[0] != 0xFF || result[1] != 0xFF || 
            result[2] != servo_id || result[3] != read_length +1) {
                return -2;
        }
        uint8_t checksum = 0;
        for (uint8_t i = 2; i < read_length; i++) {
                checksum += result[i];
        }
        checksum = ~checksum;
        if (result[read_length + 4] != checksum) {
                return -3;
        }

        // Copy result to output buffer
        for (int i = 0; i < read_length; i++)
                output_buffer[i] = result[i + 4];

        return 0;
}
