#include "STSServoDriver.hpp"

#include <cstdint>

namespace instruction {
uint8_t const PING_ = 0x01;
uint8_t const READ = 0x02;
uint8_t const WRITE = 0x03;
uint8_t const REGWRITE = 0x04;
uint8_t const ACTION = 0x05;
uint8_t const SYNCWRITE = 0x83;
uint8_t const RESET = 0x06;
};  // namespace instruction

STSServoDriver::STSServoDriver() {}

bool STSServoDriver::init(uart_inst_t *serial_port)
{
	if (serial_port == nullptr) {
		return false;
	}
	port_ = serial_port;

	// Test that a servo is present.
	for (uint8_t servo_id = 0; servo_id < 0xFE; servo_id++) {
		if (ping(servo_id)) {
			return true;
		}
	}
	return false;
}

bool STSServoDriver::ping(uint8_t const &servo_id)
{
	uint8_t response[1] = {0xFF};
	int send = sendMessage(servo_id, instruction::PING_, 0, response);
	if (send != 6) {
		return false;
	}
	// Read response
	int8_t rd = receiveMessage(servo_id, 1, response);
	if (rd < 0) {
		return false;
	}

	return response[0] == 0x00;
}

bool STSServoDriver::setId(uint8_t const &old_servo_id,
                           uint8_t const &new_servo_id)
{
	if (old_servo_id >= 0xFE || new_servo_id >= 0xFE) {
		return false;
	}
	if (ping(new_servo_id)) {
		return false;  // address taken
	}

	unsigned char lock_register = STSRegisters::WRITE_LOCK;

	// Unlock EEPROM
	if (!writeRegister(old_servo_id, lock_register, 0))
		return false;
	sleep_ms(5);
	// Write new ID
	if (!writeRegister(old_servo_id, STSRegisters::ID, new_servo_id)) {
		return false;
	}
	// Lock EEPROM
	sleep_ms(5);
	if (!writeRegister(new_servo_id, lock_register, 1)) {
		return false;
	}
	// Give it some time to change id.
	bool has_ping = false;
	int n_iter = 0;
	while (!has_ping && n_iter < 10) {
		sleep_ms(50);
		has_ping = ping(new_servo_id);
	}

	return has_ping;
}

int STSServoDriver::sendMessage(uint8_t const &servo_id,
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
	for (int i = 0; i < param_length; i++) {
		message[5 + i] = parameters[i];
		checksum += parameters[i];
	}
	message[5 + param_length] = ~checksum;
	int index = 0;
	while (index < (6 + param_length)) {
		if (uart_is_writable(port_)) {
			uart_putc(port_, message[index++]);
		}
	}
	// Give time for the message to be processed.
	sleep_ms(1);  // default 200ms
	return index;
}

int STSServoDriver::receiveMessage(uint8_t const &servo_id,
                                   uint8_t const &read_length,
                                   uint8_t *output_buffer)
{
	uint8_t result[read_length + 5];
	uint8_t index = 0;
	while (uart_is_readable_within_us(port_, 2000)) {
		result[index++] = uart_getc(port_);
	}
	if (index != (read_length + 5)) {
		return -1;
	}
	// Check message integrity
	if (result[0] != 0xFF || result[1] != 0xFF || result[2] != servo_id ||
	    result[3] != read_length + 1) {
		return -2;
	}
	uint8_t checksum = 0;
	for (int i = 2; i < read_length + 4; i++) {
		checksum += result[i];
	}
	checksum = ~checksum;
	if (result[read_length + 4] != checksum) {
		return -3;
	}

	// Copy result to output buffer
	for (int i = 0; i < read_length; i++) {
		output_buffer[i] = result[i + 4];
	}

	return 0;
}

bool STSServoDriver::writeRegisters(uint8_t const &servo_id,
                                    uint8_t const &start_register,
                                    uint8_t const &write_length,
                                    uint8_t const *parameters,
                                    bool const &asynchronous)
{
	uint8_t param[write_length + 1];
	param[0] = start_register;
	for (int i = 0; i < write_length; i++) {
		param[i + 1] = parameters[i];
	}
	int rc = sendMessage(
	    servo_id,
	    asynchronous ? instruction::REGWRITE : instruction::WRITE,
	    write_length + 1,
	    param);
	return rc == write_length + 7;
}

int STSServoDriver::readRegisters(uint8_t const &servo_id,
                                  uint8_t const &start_register,
                                  uint8_t const &read_length,
                                  uint8_t *output_buffer)
{
	uint8_t read_param[2] = {start_register, read_length};
	// Flush
	while (uart_is_readable(port_)) {
		uart_getc(port_);
	}
	int send = sendMessage(servo_id, instruction::READ, 2, read_param);
	// Failed to send
	if (send != 8) {
		return -1;
	}
	// Read
	uint8_t result[read_length + 1];
	int rd = receiveMessage(servo_id, read_length + 1, result);
	if (rd < 0) {
		return rd;
	}

	for (int i = 0; i < read_length; i++) {
		output_buffer[i] = result[i + 1];
	}
	return 0;
}

void STSServoDriver::convertIntToBytes(uint8_t const &servo_id,
                                       int const &value,
                                       uint8_t result[2])
{
	uint16_t servo_value = 0;

	servo_value = value < 0 ? -value : value;  // abs(value)
	if (value < 0) {
		servo_value = 0x8000 | servo_value;
	}
	result[0] = static_cast<unsigned char>(servo_value & 0xFF);
	result[1] = static_cast<unsigned char>((servo_value >> 8) & 0xFF);
}

int STSServoDriver::getCurrentSpeed(uint8_t const &servo_id)
{
	return readTwoBytesRegister(servo_id, STSRegisters::CURRENT_SPEED);
}

int STSServoDriver::getCurrentPosition(uint8_t const &servo_id)
{
	return readTwoBytesRegister(servo_id, STSRegisters::CURRENT_POSITION);
}

bool STSServoDriver::isMoving(uint8_t const &servo_id)
{
	uint8_t const result =
	    readRegister(servo_id, STSRegisters::MOVING_STATUS);
	return result > 0;
}

bool STSServoDriver::setTargetPosition(uint8_t const &servo_id,
                                       int const &position,
                                       int const &speed,
                                       bool const &asynchronous)
{
	uint8_t params[6] = {0,
	                     0,  // Position
	                     0,
	                     0,  // Padding
	                     0,
	                     0};  // Velocity
	convertIntToBytes(servo_id, position, &params[0]);
	convertIntToBytes(servo_id, speed, &params[4]);
	return writeRegisters(servo_id,
	                      STSRegisters::TARGET_POSITION,
	                      sizeof(params),
	                      params,
	                      asynchronous);
}

bool STSServoDriver::setTargetVelocity(uint8_t const &servo_id,
                                       int const &velocity,
                                       bool const &asynchronous)
{
	return writeTwoBytesRegister(servo_id,
	                             STSRegisters::RUNNING_SPEED,
	                             velocity,
	                             asynchronous);
}

bool STSServoDriver::setMode(unsigned char const &servo_id, STSMode const &mode)
{
	return writeRegister(servo_id,
	                     STSRegisters::OPERATION_MODE,
	                     static_cast<unsigned char>(mode));
}

bool STSServoDriver::writeRegister(uint8_t const &servo_id,
                                   uint8_t const &register_id,
                                   uint8_t const &value,
                                   bool const &asynchronous)
{
	return writeRegisters(servo_id, register_id, 1, &value, asynchronous);
}

bool STSServoDriver::writeTwoBytesRegister(uint8_t const &servo_id,
                                           uint8_t const &registerId,
                                           int16_t const &value,
                                           bool const &asynchronous)
{
	uint8_t params[2] = {0, 0};
	convertIntToBytes(servo_id, value, params);
	return writeRegisters(servo_id, registerId, 2, params, asynchronous);
}

uint8_t STSServoDriver::readRegister(uint8_t const &servo_id,
                                     uint8_t const &register_id)
{
	uint8_t result = 0;
	int rc = readRegisters(servo_id, register_id, 1, &result);
	if (rc < 0) {
		return 0;
	}
	return result;
}

int16_t STSServoDriver::readTwoBytesRegister(uint8_t const &servo_id,
                                             uint8_t const &register_id)
{
	unsigned char result[2] = {0, 0};
	int16_t value = 0;
	int16_t signed_value = 0;
	int rc = readRegisters(servo_id, register_id, 2, result);
	if (rc < 0) {
		return 0;
	}

	value = static_cast<int16_t>(result[0] + (result[1] << 8));
	// Bit 15 is sign
	signed_value = value & ~0x8000;
	if (value & 0x8000) {
		signed_value = -signed_value;
	}
	return signed_value;
}

bool STSServoDriver::setPositionOffset(uint8_t const &servo_id,
                                       int const &position_offset)

{
	if (!writeRegister(servo_id, STSRegisters::WRITE_LOCK, 0))
		return false;
	// Write new position offset
	if (!writeTwoBytesRegister(servo_id,
	                           STSRegisters::POSITION_CORRECTION,
	                           position_offset))
		return false;
	// Lock EEPROM
	if (!writeRegister(servo_id, STSRegisters::WRITE_LOCK, 1))
		return false;
	return true;
}

int STSServoDriver::getCurrentTemperature(uint8_t const &servo_id)
{
	return readTwoBytesRegister(servo_id,
	                            STSRegisters::CURRENT_TEMPERATURE);
}

float STSServoDriver::getCurrentCurrent(uint8_t const &servo_id)
{
	int16_t current =
	    readTwoBytesRegister(servo_id, STSRegisters::CURRENT_CURRENT);
	return current * 0.0065;
}

bool STSServoDriver::setTargetAcceleration(uint8_t const &servo_id,
                                           uint8_t const &acceleration,
                                           bool const &asynchronous = false)
{
	return writeRegister(servoId,
	                     STSRegisters::TARGET_ACCELERATION,
	                     acceleration,
	                     asynchronous);
}

bool STSServoDriver::trigerAction()
{
	uint8_t no_param = 0;
	int send = sendMessage(0xFE, instruction::ACTION, 0, &no_param);
	return send == 6;
}

void STSServoDriver::sendAndUpdateChecksum(uint8_t converted_value[],
                                           uint8_t &checksum)
{
	port_->write(converted_value, 2);
	int index = 0;
	while (index < 2) {
		if (uart_is_writable(port_)) {
			uart_putc(port_, converted_value[index++]);
		}
	}
	checksum += converted_value[0] + converted_value[1];
}