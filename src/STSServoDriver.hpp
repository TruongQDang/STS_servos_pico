#ifndef STS_SERVO_DRIVER_HPP
#define STS_SERVO_DRIVER_HPP

#include "pico/stdlib.h"

namespace STSRegisters {
uint8_t const FIRMWARE_MAJOR = 0x00;
uint8_t const FIRMWARE_MINOR = 0x01;
uint8_t const SERVO_MAJOR = 0x03;
uint8_t const SERVO_MINOR = 0x04;
uint8_t const ID = 0x05;
uint8_t const BAUDRATE = 0x06;
uint8_t const RESPONSE_DELAY = 0x07;
uint8_t const RESPONSE_STATUS_LEVEL = 0x08;
uint8_t const MINIMUM_ANGLE = 0x09;
uint8_t const MAXIMUM_ANGLE = 0x0B;
uint8_t const MAXIMUM_TEMPERATURE = 0x0D;
uint8_t const MAXIMUM_VOLTAGE = 0x0E;
uint8_t const MINIMUM_VOLTAGE = 0x0F;
uint8_t const MAXIMUM_TORQUE = 0x10;
uint8_t const UNLOADING_CONDITION = 0x13;
uint8_t const LED_ALARM_CONDITION = 0x14;
uint8_t const POS_PROPORTIONAL_GAIN = 0x15;
uint8_t const POS_DERIVATIVE_GAIN = 0x16;
uint8_t const POS_INTEGRAL_GAIN = 0x17;
uint8_t const MINIMUM_STARTUP_FORCE = 0x18;
uint8_t const CK_INSENSITIVE_AREA = 0x1A;
uint8_t const CCK_INSENSITIVE_AREA = 0x1B;
uint8_t const CURRENT_PROTECTION_TH = 0x1C;
uint8_t const ANGULAR_RESOLUTION = 0x1E;
uint8_t const POSITION_CORRECTION = 0x1F;
uint8_t const OPERATION_MODE = 0x21;
uint8_t const TORQUE_PROTECTION_TH = 0x22;
uint8_t const TORQUE_PROTECTION_TIME = 0x23;
uint8_t const OVERLOAD_TORQUE = 0x24;
uint8_t const SPEED_PROPORTIONAL_GAIN = 0x25;
uint8_t const OVERCURRENT_TIME = 0x26;
uint8_t const SPEED_INTEGRAL_GAIN = 0x27;
uint8_t const TORQUE_SWITCH = 0x28;
uint8_t const TARGET_ACCELERATION = 0x29;
uint8_t const TARGET_POSITION = 0x2A;
uint8_t const RUNNING_TIME = 0x2C;
uint8_t const RUNNING_SPEED = 0x2E;
uint8_t const TORQUE_LIMIT = 0x30;
uint8_t const WRITE_LOCK = 0x37;
uint8_t const CURRENT_POSITION = 0x38;
uint8_t const CURRENT_SPEED = 0x3A;
uint8_t const CURRENT_DRIVE_VOLTAGE = 0x3C;
uint8_t const CURRENT_VOLTAGE = 0x3E;
uint8_t const CURRENT_TEMPERATURE = 0x3F;
uint8_t const ASYNCHRONOUS_WRITE_ST = 0x40;
uint8_t const STATUS = 0x41;
uint8_t const MOVING_STATUS = 0x42;
uint8_t const CURRENT_CURRENT = 0x45;
};  // namespace STSRegisters

enum STSMode { POSITION = 0, VELOCITY = 1, STEP = 3 };

/// \brief Driver for STS servos, using UART
class STSServoDriver
{
       public:
	/// \brief Constructor.
	STSServoDriver();

	/// \brief Initialize the servo driver through serial port
	/// \param serial_port Serial port, default is Serial
	/// \returns  True on success (at least one servo responds to ping)
	bool init(uart_inst_t *serial_port = nullptr);

	/// \brief Ping servo
	/// \param[in] servo_id ID of the servo
	/// \return True if servo responded to ping
	bool ping(uint8_t const &servo_id);

	/// \brief Change the ID of a servo.
	/// \note If the desired ID is already taken, this function does nothing
	/// and returns false. \param[in] old_servo_id old servo ID \param[in]
	/// new_servo_id new servo ID \return True if servo could successfully
	/// change ID
	bool setId(uint8_t const &old_servo_id, uint8_t const &new_servo_id);

	/// \brief Change the position offset of a servo.
	/// \param[in] servo_id servo ID
	/// \param[in] position_offset new position offset
	/// \return True if servo could successfully change position offset
	bool setPositionOffset(uint8_t const &servo_id,
	                       int const &position_offset);

	/// \brief Get current servo position.
	/// \note This function assumes that the amplification factor
	/// ANGULAR_RESOLUTION is set to 1. \param[in] servo_id ID of the servo
	/// \return Position, in counts. 0 on failure.
	int getCurrentPosition(uint8_t const &servo_id);

	/// \brief Get current servo speed.
	/// \note This function assumes that the amplification factor
	/// ANGULAR_RESOLUTION is set to 1. \param[in] servo_id ID of the servo
	/// \return Speed, in counts/s. 0 on failure.
	int getCurrentSpeed(uint8_t const &servo_id);

	/// \brief Get current servo temperature.
	/// \param[in] servo_id ID of the servo
	/// \return Temperature, in degC. 0 on failure.
	int getCurrentTemperature(uint8_t const &servo_id);

	/// \brief Get current servo current.
	/// \param[in] servo_id ID of the servo
	/// \return Current, in A.
	float getCurrentCurrent(uint8_t const &servo_id);

	/// \brief Check if the servo is moving
	/// \param[in] servo_id ID of the servo
	/// \return True if moving, false otherwise.
	bool isMoving(uint8_t const &servo_id);

	/// \brief Set target servo position.
	/// \note This function assumes that the amplification factor
	/// ANGULAR_RESOLUTION is set to 1. \param[in] servo_id ID of the servo
	/// \param[in] position Target position, in counts.
	/// \param[in] speed speed of the servo.
	/// \param[in] asynchronous If set, write is asynchronous (ACTION must
	/// be send to activate) \return True on success, false otherwise.
	bool setTargetPosition(uint8_t const &servo_id,
	                       int const &position,
	                       int const &speed = 4095,
	                       bool const &asynchronous = false);

	/// \brief Set target servo velocity.
	/// \note This function assumes that the amplification factor
	/// ANGULAR_RESOLUTION is set to 1. \param[in] servo_id ID of the servo
	/// \param[in] velocity Target velocity, in counts/s.
	/// \param[in] asynchronous If set, write is asynchronous (ACTION must
	/// be send to activate) \return True on success, false otherwise.
	bool setTargetVelocity(uint8_t const &servo_id,
	                       int const &velocity,
	                       bool const &asynchronous = false);

	/// \brief Change the target acceleration of a servo.
	/// \param[in] servo_id servo ID
	/// \param[in] acceleration target acceleration
	/// \return True if servo could successfully set target acceleration
	bool setTargetAcceleration(uint8_t const &servo_id,
	                           uint8_t const &acceleration,
	                           bool const &asynchronous = false);

	/// \brief Set servo working mode: position, velocity or step.
	/// \param[in] servo_id ID of the servo
	/// \param[in] mode Desired mode
	bool setMode(unsigned char const &servo_id, STSMode const &mode);

	/// \brief Trigger the action previously stored by an asynchronous write
	/// on all servos. \return True on success
	bool trigerAction();

	/// \brief Write to a single uint8_t register.
	/// \param[in] servo_id ID of the servo
	/// \param[in] register_id Register id.
	/// \param[in] value Register value.
	/// \param[in] asynchronous If set, write is asynchronous (ACTION must
	/// be send to activate) \return True if write was successful
	bool writeRegister(uint8_t const &servo_id,
	                   uint8_t const &register_id,
	                   uint8_t const &value,
	                   bool const &asynchronous = false);

	/// \brief Write a two-uint8_ts register.
	/// \param[in] servo_id ID of the servo
	/// \param[in] registerId Register id (LSB).
	/// \param[in] value Register value.
	/// \param[in] asynchronous If set, write is asynchronous (ACTION must
	/// be send to activate) \return True if write was successful
	bool writeTwoBytesRegister(uint8_t const &servo_id,
	                           uint8_t const &registerId,
	                           int16_t const &value,
	                           bool const &asynchronous = false);

	/// \brief Read a single register
	/// \param[in] servo_id ID of the servo
	/// \param[in] register_id Register id.
	/// \return Register value, 0 on failure.
	uint8_t readRegister(uint8_t const &servo_id,
	                     uint8_t const &register_id);

	/// \brief Read two uint8_ts, interpret result as <LSB> <MSB>
	/// \param[in] servo_id ID of the servo
	/// \param[in] register_id LSB register id.
	/// \return Register value, 0 on failure.
	int16_t readTwoBytesRegister(uint8_t const &servo_id,
	                             uint8_t const &register_id);

	/// @brief Sets the target positions for multiple servos simultaneously.
	/// @param[in] NumberOfServos Number of servo.
	/// @param[in] servo_ids Array of servo IDs to control.
	/// @param[in] positions Array of target positions (corresponds to
	/// servo_ids).
	/// @param[in] speeds Array of target speeds (corresponds to servo_ids).
	void setTargetPositions(uint8_t const &numberOfServos,
	                        const uint8_t servo_ids[],
	                        const int positions[],
	                        const int speeds[]);

       private:
	/// \brief Send a message to the servos.
	/// \param[in] servo_id ID of the servo
	/// \param[in] command_id Command id
	/// \param[in] param_length length of the parameters
	/// \param[in] parameters parameters
	/// \return Result of write.
	int sendMessage(uint8_t const &servo_id,
	                uint8_t const &command_id,
	                uint8_t const &param_length,
	                uint8_t *parameters);

	/// \brief Recieve a message from a given servo.
	/// \param[in] servo_id ID of the servo
	/// \param[in] read_length Message length
	/// \param[in] paramLength length of the parameters
	/// \param[in] output_buffer Buffer where the data is placed.
	/// \return 0 on success
	///         -1 if read failed due to timeout
	///         -2 if invalid message (no 0XFF, wrong servo id)
	///         -3 if invalid checksum
	int receiveMessage(uint8_t const &servo_id,
	                   uint8_t const &read_length,
	                   uint8_t *output_buffer);

	/// \brief Write to a sequence of consecutive registers
	/// \param[in] servo_id ID of the servo
	/// \param[in] start_register First register
	/// \param[in] write_length Number of registers to write
	/// \param[in] parameters Value of the registers
	/// \param[in] asynchronous If set, write is asynchronous (ACTION must
	/// be send to activate) \return True if write was successful
	bool writeRegisters(uint8_t const &servo_id,
	                    uint8_t const &start_register,
	                    uint8_t const &write_length,
	                    uint8_t const *parameters,
	                    bool const &asynchronous = false);

	/// \brief Read a sequence of consecutive registers.
	/// \param[in] servo_id ID of the servo
	/// \param[in] start_register First register
	/// \param[in] read_length Number of registers to write
	/// \param[out] output_buffer Buffer where to read the data (must have
	/// been allocated by the user) \return 0 on success, -1 if write
	/// failed, -2 if read failed, -3 if checksum verification failed
	int readRegisters(uint8_t const &servo_id,
	                  uint8_t const &start_register,
	                  uint8_t const &read_length,
	                  uint8_t *output_buffer);

	/// @brief Send two uint8_ts and update checksum
	/// @param[in] converted_value Converted int value
	/// @param[out] checksum Update the checksum
	void sendAndUpdateChecksum(uint8_t converted_value[],
	                           uint8_t &checksum);

	/// @brief Convert int to pair of uint8_ts
	/// @param[in] value
	/// @param[out] result
	void convertIntToBytes(uint8_t const &servo_id,
	                       int const &value,
	                       uint8_t result[2]);

       private:
	uart_inst_t *port_;
};

#endif  // STS_SERVO_DRIVER_HPP