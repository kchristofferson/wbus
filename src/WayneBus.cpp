/*
 * WayneBus.cpp
 *
 *  Created on: Apr 11, 2015
 *      Author: Kurt
 */

#include <endian.h>
#include <syslog.h>
#include <stdint.h>
#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/thread.hpp>

#include "WayneBus.h"
#include "CRC.h"

namespace ur {

WayneBus::WayneBus(boost::asio::io_service& ios) :
		m_port(ios)
{
	// TODO Auto-generated constructor stub
  tlb_ptr = new boost::thread;
  tln_ptr = new boost::thread;
}

WayneBus::~WayneBus()
{
  // TODO Auto-generated destructor stub

  boost::system::error_code ec;

	boost::mutex::scoped_lock look(mutex_);
	m_port.cancel();
 	m_port.close();
	io_service_.stop();
	io_service_.reset();

	delete tlb_ptr;
	delete tln_ptr;
}

bool WayneBus::start(const char *com_port_name,
                     int baud_rate,
                     int charsize,
                     int stop,
                     boost::asio::serial_port::parity::type parity,
                     boost::asio::serial_port::flow_control::type flow)
{
	boost::system::error_code ec;

	// connect to the motor controll
	m_port.open(com_port_name, ec);
	if ( ec ) {
        syslog(LOG_ERR | LOG_USER, "m_port.open() failed...com_port_name= \"%s\"\n", com_port_name);
		return false;
	}

	// option settings...
	m_port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
	m_port.set_option(boost::asio::serial_port_base::character_size(charsize));
	if ( stop == 1 )
      m_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
	else if ( stop == 2 )
	  m_port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::two));
	else
	{
        syslog(LOG_ERR | LOG_USER, "m_port.open() configuration failed...invalid stopbit setting = \"%d\"\n", stop);
		return false;
	}
	m_port.set_option(boost::asio::serial_port_base::parity(parity));
	m_port.set_option(boost::asio::serial_port_base::flow_control(flow));

	// start thread to listen for messages from the board
	boost::thread tlb(boost::bind(&ur::WayneBus::listen_board,
	                              boost::asio::placeholders::error,
                                (td::size_t) 1);
	tlb_ptr = &tlb;

	// start thread to send command messages to the board
	boost::thread tln(boost::bind(&ur::WayneBus::on_cmd_queue, this));
	tln_ptr = &tln;

	return true;
}

int WayneBus::write_board(message msg)
{
	boost::system::error_code ec;
	ur::CRC crc;

	if ( !m_port.is_open() ) return -1;

	// convert to big endian for the controller
	msg.val.i = htobe32(msg.val.i);

	// generate the checksum
	msg.crc8 = crc.checksum(&msg, 1, 7);

	return m_port.write_some(boost::asio::buffer(&msg, sizeof(message)), ec);
}

void WayneBus::listen_board(const boost::system::error_code& error, std::size_t transferred)
{
	struct message *msg = new message;
	size_t fill = 0;

	if ( !m_port.is_open() ) return;

    for (;; fill = 0)
    {
	    while (fill < sizeof(message))
	    {
		  m_port.async_read_some(boost::asio::buffer(msg + fill, BUF_SIZE),
	           boost::bind(&WayneBus::listen_board,
			       boost::asio::placeholders::error,
		         transferred));
		  if ( transferred != 1 )
		  {
	        syslog(LOG_ERR | LOG_USER, "m_port.async_read_some() failed...");
		    return;
		  }
	      ++fill;
	      if (fill == 1 && msg->delimiter != MESSAGE_DELIMITER)
	        fill = 0;
	      else if (fill == 2 && msg->version != MESSAGE_VERSION)
	        fill = 0;
	    }

	    // have a complete message
	    on_board_message(*msg);
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
	  odom_queue.push_back(msg);
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
	  diag_queue.push_back(msg);
	  mutex_diag.unlock();
	  break;
  default :
	  break;
  }
}

void WayneBus::on_cmd_queue(void)
{
  if ( !m_port.is_open() ) return;

    for ( ; ; )
    {
    	if ( !cmd_queue.empty() )
    	{
    		for(std::vector<ur::message>::iterator iter = cmd_queue.begin();
    		    iter != cmd_queue.end(); iter++)
    		{
    			mutex_cmd.lock();
    		    if ( write_board(*iter) != sizeof(ur::message) )
    		        syslog(LOG_ERR | LOG_USER, "on_cmd_queue failed to write complete packet to board.\n");
    		    cmd_queue.erase(iter);
    		    mutex_cmd.unlock();
    		}
    	}
    	boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
    }
}

} /* namespace ur */

