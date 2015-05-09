/*
 * WayneBus.cpp
 *
 *  Created on: Apr 11, 2015
 *      Author: Kurt
 */

#include <endian.h>
#include <syslog.h>

#include "WayneBus.h"
#include "CRC.h"

namespace ur {

WayneBus::WayneBus(boost::asio::io_service ios) :
		io_service_(ios),
		tlb_ptr(new boost::thread *),
		tln_ptr(new boost::thread *)
{
	// TODO Auto-generated constructor stub
}

WayneBus::~WayneBus()
{
	// TODO Auto-generated destructor stub

	boost::mutex::scoped_lock look(mutex_);

		if (port_) {
			port_->cancel();
			port_->close();
			port_.reset();
		}
		io_service_.stop();
		io_service_.reset();

		~tlb_ptr;
		~tln_ptr;
}

bool WayneBus::start(const char *com_port_name, int baud_rate=9600, int charsize, int stop=1, char parity='n', char flow='n')
{
	boost::system::error_code ec;

	// connect once and only once
	if ( port_ ) {
        syslog(LOG_ERR | LOG_USER, "Attempted to open connection that is already open.\n");
		return false;
	}

	// connect to the motor controller
	port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
	port_->open(com_port_name, ec);
	if ( ec ) {
        syslog(LOG_ERR | LOG_USER, "port_->open() failed...com_port_name= \"%s\"\n", com_port_name);
		return false;
	}

	// option settings...
	port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
	port_->set_option(boost::asio::serial_port_base::character_size(charsize));
	if ( stop == 1 )
      port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
	else if ( stop == 2 )
	  port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::two));
	else
	{
        syslog(LOG_ERR | LOG_USER, "port_->open() configuration failed...invalid stopbit setting = \"%d\"\n", stop);
		return false;
	}
	switch ( parity )
	{
	case 'n' :
	case 'N' :
		port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
		break;
	case 'o' :
	case 'O' :
		port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::odd));
		break;
	case 'e' :
	case 'E' :
		port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::even));
		break;
	default:
        syslog(LOG_ERR | LOG_USER, "port_->open() configuration failed...invalid parity setting = \"%c\"\n", parity);
		return false;
		break;
	}
	switch ( flow )
	{
	case 'n' :
	case 'N' :
		port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
		break;
	case 'h' :
	case 'H' :
		port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::hardware));
		break;
	case 's' :
	case 'S' :
		port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::software));
		break;
	default:
        syslog(LOG_ERR | LOG_USER, "port_->open() configuration failed...invalid flow control setting = \"%c\"\n", flow);
		return false;
		break;
	}

	// start thread to listen for messages from the board
	boost::thread tlb(boost::bind(&ur::WayneBus::listen_board, this));
	tlb_ptr = tlb;

	// start thread to send command messages to the board
	boost::thread tln(boost::bind(&ur::WayneBus::on_cmd_queue, this));
	tln_ptr = tln;

	return true;
}

int write_board(message msg)
{
	boost::system::error_code ec;
	ur::CRC crc;

	if ( !port_ ) return -1;

	// convert to big endian for the controller
	msg.val.i = htobe32(msg.val.i);

	// generate the checksum
	msg.crc8 = crc.checksum(&msg, 1, 7);

	return port_->write_some(boost::asio::buffer(&msg, sizeof(message)), ec);
}

int write_board(const char *buf, const int &size)
{
	boost::system::error_code ec;

		if ( !port_ ) return -1;
		if ( size == 0 ) return 0;

		return port_->write_some(boost::asio::buffer(buf, size), ec);
}

void WayneBus::listen_board(void)
{
	struct message *msg = new message;
	size_t fill = 0;

	if (port_.get() == NULL || !port_->is_open()) return;

    for (;; fill = 0)
    {
	    while (fill < sizeof(message))
	    {
		  if (1 != 	port_->async_read_some( boost::asio::buffer(&msg + fill, 1),
	                boost::bind(&SerialPort::on_receive_,
		            this,
				    boost::asio::placeholders::error,
		            boost::asio::placeholders::bytes_transferred) ) )
		  {
	        syslog(LOG_ERR | LOG_USER, "port_->async_read_some() failed...");
		    return;
		  }
	      ++fill;
	      if (fill == 1 && msg.delimiter != MESSAGE_DELIMITER)
	        fill = 0;
	      else if (fill == 2 && msg.version != MESSAGE_VERSION)
	        fill = 0;
	    }

	    // have a complete message
	    on_board_message(msg);
    }
}

void WayneBus::on_board_message(message msg)
{
  ur::CRC crc;

  if ( msg.crc8 != crc.checksum(&msg, 1, 7) )
	  return;  // bad packet

  // convert values to host format
  msg.val.i = be32toh(msg.val.i);

  switch ( msg.reg )
  {
  case REGISTER_ON :
  case REGISTER_BRAKE :
  case REGISTER_HALT :
  case REGISTER_PWM_LEFT :
  case REGISTER_PWM_RIGHT :
  case REGISTER_SPEED_LEFT :
  case REGISTER_SPEED_RIGHT :
  case REGISTER_RAMP_LEFT :
  case REGISTER_RAMP_RIGHT :
	  break;
  case REGISTER_TICS_LEFT :
  case REGISTER_TICS_RIGHT :
	  mutex_odom.lock();
	  odom_queue.push(msg);
	  mutex_odom.unlock();
	  break;
  case REGISTER_DEADMAN :
	  break;
  case REGISTER_CURRENT_LEFT :
  case REGISTER_CURRENT_RIGHT :
  case REGISTER_ERROR_COUNT :
  case REGISTER_ERROR_MAIN_5V :
  case REGISTER_ERROR_AUX_5V :
  case REGISTER_ERROR_MAIN_12V :
  case REGISTER_ERROR_AUX_12V :
  case REGISTER_OVERLOAD_MAIN_5V :
  case REGISTER_OVERLOAD_AUX_5V :
  case REGISTER_OVERLOAD_MAIN_12V :
  case REGISTER_OVERLOAD_AUX_12V :
  case REGISTER_ERROR_LEFT :
  case REGISTER_ERROR_RIGHT :
  case REGISTER_PROPORTIONAL :
  case REGISTER_INTEGRAL :
  case REGISTER_DERIVATIVE :
  case REGISTER_DENOMINATOR :
  case REGISTER_LED1 :
  case REGISTER_LED2 :
  case REGISTER_HARDWARE :
  case REGISTER_FIRMWARE :
  case REGISTER_VOLTAGE :
  case REGISTER_CURRENT_MAIN_5V :
  case REGISTER_CURRENT_AUX_5V :
  case REGISTER_CURRENT_MAIN_12V :
  case REGISTER_CURRENT_AUX_12V :
	  mutex_diag.lock();
	  diag_queue.push(msg);
	  mutex_diab.unlock();
	  break;
  default :
	  break;
  }
}

void WayneBus::on_cmd_queue(void)
{
	struct message *msg;

// if named queue in heap does not exist	if (port_.get() == NULL || !port_->is_open()) return;

    for ( ; ; )
    {
    	if ( !cmd_queue.empty() )
    	{
    		for(std::list<ur::message>::iterator iter = cmd_queue.begin();
    		    iter != cmd_queue.end(); iter++)
    		{
    			mutex_cmd.lock();
    		    if ( write_board(*iter) != sizeof(ur::message) )
    		        syslog(LOG_ERR | LOG_USER, "on_cmd_queue failed to write complete packet to board.\n");
    		    cmd_queue.remove(*iter);
    		    mutex_cmd.unlock();
    		}
    	}
    	boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
    }
}

} /* namespace ur */

