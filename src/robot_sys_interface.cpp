/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot_sys_interface.h"
#include "industrial_utils/param_utils.h"

using industrial::smpl_msg_connection::SmplMsgConnection;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

namespace industrial_robot_client
{
namespace robot_robot_sys_interface
{

RobotSysInterface::RobotSysInterface()
{
  this->connection_ = NULL;
  mssg_type_ = 0;
  this->add_handler(&default_robot_sys_handler_);
}

bool RobotSysInterface::init(std::string default_ip, int default_port)
{
  std::string ip;
  int port;
  ros::param::param<std::string>("robot_ip_address", ip, default_ip);
  ros::param::param<int>("~port", port, default_port);

  // check for valid parameter values
  if (ip.empty())
  {
    ROS_ERROR("No valid robot IP address found.  Please set ROS 'robot_ip_address' param");
    return false;
  }
  if (port <= 0)
  {
    ROS_ERROR("No valid robot IP port found.  Please set ROS '~port' param");
    return false;
  }

  char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
  ROS_INFO("Robot state connecting to IP address: '%s:%d'", ip_addr, port);
  bool ret =  default_tcp_connection_.init(ip_addr, port);
  std::cout<<"net ret = "<<ret<<std::endl;
  free(ip_addr);

  return init(&default_tcp_connection_);
}

bool RobotSysInterface::init(SmplMsgConnection* connection)
{
  this->connection_ = connection;
  if(!this->connection_->isConnected()){
      connection_->makeConnect();
  }
  if (!default_robot_sys_handler_.init(connection_,mssg_type_))
      return false;
  this->add_handler(&default_robot_sys_handler_);
    return true;
}

bool RobotSysInterface::set_mssage_type(int mssg_type)
{
    if(mssg_type ==  0)
       return false;
    else
       mssg_type_ = mssg_type;
    return true;
}

void RobotSysInterface::run()
{
  manager_.spin();
}

} // robot_state_interface
} // industrial_robot_client
