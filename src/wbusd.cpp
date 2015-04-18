/*
 * wbusd.cpp
 *
 *   Created on: Apr 10, 2015
 *  Last Change: April 10, 2015
 *       Author: Kurt Christofferson
 *  Description: Service interface to Wayne Gramlich's dual h-bridge
 *
 */

#include <boost/exception.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <boost/program_options.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <ctime>
#include <iostream>
#include <fstream>
#include <syslog.h>
#include <unistd.h>

#include "WayneBus.h"

// global variables

int main(int argc, char *argv[])
{
	unsigned int baud_rate_;
	unsigned int char_size_;
	unsigned int stop_bits_;
	std::string parity_;
	boost::asio::serial_port::parity::type parity;
	std::string flow_control_;
	boost::asio::serial_port::flow_control::type flow_control;
	std::string device_name_;
  boost::program_options::variables_map vm;
  std::string sCfgFile = "/etc/wbusd/wbusd.conf";

  boost::asio::io_service io_service;
  ur::WayneBus server(io_service);

	try
	{

	// Pick up parameters
	// command line
	boost::program_options::options_description generic("Generic options");
	generic.add_options()
	    ("version,v", "print version string")
	    ("help", "produce help message")
	    ;

	// config file
	boost::program_options::options_description config("Configuration");
	config.add_options()
    ("baud", boost::program_options::value<int>()->default_value(115200), "baud rate")
    ("size", boost::program_options::value<int>()->default_value(8), "character size")
    ("stopbits", boost::program_options::value<int>()->default_value(1), "Number of stop bits")
    ("parity", boost::program_options::value<std::string>()->default_value("n"), "Parity - (n)one, (o)dd, or (e)ven")
    ("flow", boost::program_options::value<std::string>()->default_value("n"), "Flow control - (n)one, (s)oftware, or (h)ardware")
    ("device", boost::program_options::value<std::string>(), "Device name for the bus board")
	;

	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic);

	boost::program_options::options_description config_options;
	config_options.add(config);

	boost::program_options::store(parse_command_line(argc, argv, cmdline_options), vm);

  std::ifstream cfgfile(sCfgFile.c_str(), std::ios::in);
  if (cfgfile.fail())
    syslog(LOG_ERR | LOG_USER, "wbusd failed to open the configuration file \"%s\"", sCfgFile.c_str());
	boost::program_options::store(parse_config_file(cfgfile, config_options, true), vm);
	cfgfile.close();

	boost::program_options::notify(vm);
	}
	catch (boost::system::error_code &e)
	{
        syslog(LOG_ERR | LOG_USER, "wbusd obtaining configuration failed: %m", e.message());
        return 1;
	}

	try
	{
		if (vm.count("device"))
		  device_name_ = (const std::string&) vm["device"].as<std::string>();

		baud_rate_ = (unsigned int) vm["baud"].as<int>();
		char_size_ = (unsigned int) vm["size"].as<int>();
		stop_bits_ = (unsigned int) vm["stopbits"].as<int>();

		parity_ = vm["parity"].as<std::string>();
		if ( parity_.compare(0,1,"e") == 0 || parity_.compare(0,1,"E") == 0 )
		  parity = boost::asio::serial_port::parity::even;
		else if ( parity_.compare(0,1,"o") == 0 || parity_.compare(0,1,"O") == 0 )
      parity = boost::asio::serial_port::parity::odd;
		else  // If config file is messed up, accept no parity
      parity = boost::asio::serial_port::parity::none;

		flow_control_ = vm["parity"].as<std::string>();
    if ( flow_control_.compare(0,1,"s") == 0 || flow_control_.compare(0,1,"S") == 0 )
      flow_control = boost::asio::serial_port::flow_control::software;
    else if ( flow_control_.compare(0,1,"h") == 0 || flow_control_.compare(0,1,"H") == 0 )
      flow_control = boost::asio::serial_port::flow_control::hardware;
    else  // If config file is messed up, accept no flow control
      flow_control = boost::asio::serial_port::flow_control::none;
	}
	catch (boost::system::error_code& e)
	{
        syslog(LOG_ERR | LOG_USER, "Invalid configuration: %m", e.message());
        return 1;
	}

	// become a daemon
	  try
	  {
	    boost::asio::signal_set signals(io_service, SIGINT, SIGTERM);
	    signals.async_wait(
	        boost::bind(&boost::asio::io_service::stop, &io_service));

	    // Inform the io_service that we are about to become a daemon. The
	    // io_service cleans up any internal resources, such as threads, that may
	    // interfere with forking.
	    io_service.notify_fork(boost::asio::io_service::fork_prepare);

	    // Fork the process and have the parent exit. If the process was started
	    // from a shell, this returns control to the user. Forking a new process is
	    // also a prerequisite for the subsequent call to setsid().
	    if (pid_t pid = fork())
	    {
	      if (pid > 0)
	      {
	        // We're in the parent process and need to exit.
	        //
	        // When the exit() function is used, the program terminates without
	        // invoking local variables' destructors. Only global variables are
	        // destroyed. As the io_service object is a local variable, this means
	        // we do not have to call:
	        //
	        //   io_service.notify_fork(boost::asio::io_service::fork_parent);
	        //
	        // However, this line should be added before each call to exit() if
	        // using a global io_service object. An additional call:
	        //
	        //   io_service.notify_fork(boost::asio::io_service::fork_prepare);
	        //
	        // should also precede the second fork().
	        exit(0);
	      }
	      else
	      {
	        syslog(LOG_ERR | LOG_USER, "First fork failed: %m");
	        return 1;
	      }
	    }

	    // Make the process a new session leader. This detaches it from the
	    // terminal.
	    setsid();

	    // A process inherits its working directory from its parent. This could be
	    // on a mounted filesystem, which means that the running daemon would
	    // prevent this filesystem from being unmounted. Changing to the root
	    // directory avoids this problem.
	    chdir("/");

	    // The file mode creation mask is also inherited from the parent process.
	    // We don't want to restrict the permissions on files created by the
	    // daemon, so the mask is cleared.
	    umask(0);

	    // A second fork ensures the process cannot acquire a controlling terminal.
	    io_service.notify_fork(boost::asio::io_service::fork_prepare);
	    if (pid_t pid = fork())
	    {
	      if (pid > 0)
	      {
	        exit(0);
	      }
	      else
	      {
	        syslog(LOG_ERR | LOG_USER, "Second fork failed: %m");
	        return 1;
	      }
	    }

	    // Close the standard streams. This decouples the daemon from the terminal
	    // that started it.
	    close(0);
	    close(1);
	    close(2);

	    // We don't want the daemon to have any standard input.
	    if (open("/dev/null", O_RDONLY) < 0)
	    {
	      syslog(LOG_ERR | LOG_USER, "Unable to open /dev/null: %m");
	      return 1;
	    }

	    // Send standard output to a log file.
	    const char* output = "/var/log/wbus/wbus.log";
	    const int flags = O_WRONLY | O_CREAT | O_APPEND;
	    const mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
	    if (open(output, flags, mode) < 0)
	    {
	      syslog(LOG_ERR | LOG_USER, "Unable to open output file %s: %m", output);
	      return 1;
	    }

	    // Also send standard error to the same log file.
	    if (dup(1) < 0)
	    {
	      syslog(LOG_ERR | LOG_USER, "Unable to dup output descriptor: %m");
	      return 1;
	    }

	    // Inform the io_service that we have finished becoming a daemon. The
	    // io_service uses this opportunity to create any internal file descriptors
	    // that need to be private to the new process.
	    io_service.notify_fork(boost::asio::io_service::fork_child);

	    // The io_service can now be used normally.
	    syslog(LOG_INFO | LOG_USER, "wbus daemon service started");
	    io_service.run();
	    syslog(LOG_INFO | LOG_USER, "wbus daemon service stopped");
	  }
	  catch (std::exception& e)
	  {
	    syslog(LOG_ERR | LOG_USER, "Exception: %s", e.what());
	    std::cerr << "Exception: " << e.what() << std::endl;
	  }

	// Create named software node to wbusd queue
   try
   {
      //Erase any previous message queues if they exist
      boost::interprocess::message_queue::remove("ur_cmd_queue");
      boost::interprocess::message_queue::remove("ur_odom_queue");
      boost::interprocess::message_queue::remove("ur_diag_queue");
      boost::interprocess::message_queue::remove("ur_sonar_queue");

      //Create an interprocess queue to accept commands.
      boost::interprocess::message_queue mq_odom
         (boost::interprocess::create_only               //only create
         ,"ur_cmd_queue"            //name
         ,500                       //max message number
         ,sizeof(ur::message)       //max message size
         );

      //Create an interprocess queue to pass odometry.
      boost::interprocess::message_queue mq_odom
         (boost::interprocess::create_only               //only create
         ,"ur_odom_queue"              //name
         ,100                       //max message number
         ,sizeof(ur::message)       //max message size
         );

      //Create an interprocess queue to pass diagnostics.
      boost::interprocess::message_queue mq_diag
         (boost::interprocess::create_only               //only create
         ,"ur_diag_queue"              //name
         ,100                       //max message number
         ,sizeof(ur::message)       //max message size
         );

      //Create an interprocess queue to pass sonar information.
      boost::interprocess::message_queue mq_sonar
         (boost::interprocess::create_only               //only create
         ,"ur_sonar_queue"              //name
         ,100                       //max message number
         ,sizeof(ur::message)       //max message size
         );
	   }
	   catch(boost::interprocess::interprocess_exception &e){
         syslog(LOG_ERR | LOG_USER, "Exception: %s", e.what());
         std::cerr << "Exception: " << e.what() << std::endl;
         return 1;
	   }

	// open the device rw
	  server.start(device_name_.c_str(), baud_rate_, char_size_, stop_bits_, parity, flow_control);

	// loop

	return 0;
}


