/*
 * WayneBus.h
 *
 *  Created on: Apr 11, 2015
 *      Author: Kurt
 */

#ifndef WAYNEBUS_H_
#define WAYNEBUS_H_

#include <stdint.h>

namespace ur {

const unsigned int BUF_SIZE = 1024;

const uint32_t MOTOR_LEFT = 0x00;
const uint32_t MOTOR_RIGHT = 0x01;

const uint8_t MESSAGE_DELIMITER = 0x7E;
const uint8_t MESSAGE_VERSION = 0x02;

const uint8_t MESSAGE_READ = 0xAA;
const uint8_t MESSAGE_WRITE = 0xBB;
const uint8_t MESSAGE_RESPONSE = 0xCC;

const uint8_t REGISTER_ON = 0x00;
const uint8_t REGISTER_BRAKE = 0x01;
const uint8_t REGISTER_HALT = 0x02;
const uint8_t REGISTER_PWM_LEFT = 0x03;
const uint8_t REGISTER_PWM_RIGHT = 0x04;
const uint8_t REGISTER_SPEED_LEFT = 0x07;
const uint8_t REGISTER_SPEED_RIGHT = 0x08;
const uint8_t REGISTER_RAMP_LEFT = 0x09;
const uint8_t REGISTER_RAMP_RIGHT = 0x0A;
const uint8_t REGISTER_TICS_LEFT = 0x0B;
const uint8_t REGISTER_TICS_RIGHT = 0x0C;
const uint8_t REGISTER_DEADMAN = 0x0D;
const uint8_t REGISTER_CURRENT_LEFT = 0x0E;
const uint8_t REGISTER_CURRENT_RIGHT = 0x0F;
const uint8_t REGISTER_ERROR_COUNT = 0x10;
const uint8_t REGISTER_ERROR_MAIN_5V = 0x11;
const uint8_t REGISTER_ERROR_AUX_5V = 0x12;
const uint8_t REGISTER_ERROR_MAIN_12V = 0x13;
const uint8_t REGISTER_ERROR_AUX_12V = 0x14;
const uint8_t REGISTER_OVERLOAD_MAIN_5V = 0x15;
const uint8_t REGISTER_OVERLOAD_AUX_5V = 0x16;
const uint8_t REGISTER_OVERLOAD_MAIN_12V = 0x17;
const uint8_t REGISTER_OVERLOAD_AUX_12V = 0x18;
const uint8_t REGISTER_ERROR_LEFT = 0x19;
const uint8_t REGISTER_ERROR_RIGHT = 0x1A;
const uint8_t REGISTER_PROPORTIONAL = 0x1B;
const uint8_t REGISTER_INTEGRAL = 0x1C;
const uint8_t REGISTER_DERIVATIVE = 0x1D;
const uint8_t REGISTER_DENOMINATOR = 0x1E;
const uint8_t REGISTER_LED1 = 0x1F;
const uint8_t REGISTER_LED2 = 0x20;
const uint8_t REGISTER_HARDWARE = 0x21;
const uint8_t REGISTER_FIRMWARE = 0x22;
const uint8_t REGISTER_VOLTAGE = 0x23;
const uint8_t REGISTER_CURRENT_MAIN_5V = 0x24;
const uint8_t REGISTER_CURRENT_AUX_5V = 0x26;
const uint8_t REGISTER_CURRENT_MAIN_12V = 0x25;
const uint8_t REGISTER_CURRENT_AUX_12V = 0x27;

union msgdata {
	int32_t i;
	uint32_t u;
	uint8_t c[4];
};

// ROS Nodes that communicate with the WayneBus MUST NOT deal with
// either the endianness of the msgdata nor the cyclic redundancy
// check for the message.  Both of these are handled by the daemon.
// msgdata will be either an int or unsigned int in the format of the
// host.  Messages placed on the command queue by a node will have
// the CRC generated for them.  Messages from a board that do not
// pass the checksum will be discarded.
struct message {
	uint8_t delimiter;
	uint8_t version;
	uint8_t type;
	uint8_t reg;
	msgdata val;
	uint8_t crc8;
};

extern boost::asio::serial_port g_port;

class WayneBus {
public:
	WayneBus(boost::asio::io_service& ios);
	virtual ~WayneBus();

	virtual bool start(const char *com_port_name,
	                   int baud_rate=9600,
	                   int charsize=8,
	                   int stop=1,
                     boost::asio::serial_port::parity::type parity=boost::asio::serial_port::parity::none,
                     boost::asio::serial_port::flow_control::type flow=boost::asio::serial_port::flow_control::none);
	virtual void stop();

	int write_board(message msg);
//	int write_board(const char *buf, const int &size);


protected:
	boost::asio::io_service io_service_;
	boost::thread *tlb_ptr;
	boost::thread *tln_ptr;
	boost::mutex mutex_;

	std::vector<ur::message> cmd_queue;  // Inbound queue from one or more software nodes
    boost::mutex mutex_cmd;
	std::vector<ur::message> odom_queue; // Outbound queue to software node supporting navigation and odometry
    boost::mutex mutex_odom;
	std::vector<ur::message> diag_queue; // Outbound queue to sofware node supporting diagnostics
    boost::mutex mutex_diag;

	char read_buf_raw_[BUF_SIZE];

	virtual void listen_board(const boost::system::error_code& error, std::size_t transferred);
	virtual void on_cmd_queue(void);
	virtual void on_board_message(message msg);

private:
	boost::asio::serial_port m_port;

};

} /* namespace ur */

#endif /* WAYNEBUS_H_ */

