#ifndef ROBOT_ENABLE_INTERFACE_H
#define ROBOT_ENABLE_INTERFACE_H

#include <vector>
#include <string>
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/message_manager.h"
#include "simple_message/message_handler.h"
#include "simple_message/socket/tcp_client.h"
#include "industrial_robot_client/joint_relay_handler.h"
#include "robot_sys_relay_handler.h"
//#include "hsr_rosi_test/ClearFaultSrv.h"
//#include "hsr_rosi_test/SetEnableSrv.h"
//#include "hsr_rosi_test/StopMoveSrv.h"
//#include "hsr_rosi_test/SetEnableSrvRequest.h"

namespace industrial_robot_client
{
namespace robot_robot_sys_interface
{

using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial::message_manager::MessageManager;
using industrial::message_handler::MessageHandler;
using industrial::tcp_client::TcpClient;
//using industrial_robot_client::joint_relay_handler::JointRelayHandler;
using industrial_robot_client::robot_sys_relay_handler::RobotSysRelayHandler;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

/**
 * \brief Generic template that reads state-data from a robot controller
 * and publishes matching messages to various ROS topics.
 *
 * Users should replace the default class members
 * to implement robot-specific behavior.
 */
//* RobotStateInterface
class RobotSysInterface
{

public:
  /**
   * \brief Default constructor.
   */
  RobotSysInterface();

  /**
   * \brief Initialize robot connection using default method.
   *
   * \param default_ip default IP address to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "robot_ip_address" cannot be read
   * \param default_port default port to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "~port" cannot be read
   *
   * \return true on success, false otherwise
   */
  bool init(std::string default_ip = "10.10.56.214", int default_port = StandardSocketPorts::MOTION);


  /**
   * \brief Initialize robot connection using specified method.
   *
   * \param connection new robot-connection instance (ALREADY INITIALIZED).
   *
   * \return true on success, false otherwise
   */
  bool init(SmplMsgConnection* connection);

  bool set_mssage_type(int mssg_type);

  /**
   * \brief Begin processing messages and publishing topics.
   */
  void run();

  /**
   * \brief get current robot-connection instance.
   *
   * \return current robot connection object
   */
  SmplMsgConnection* get_connection()
  {
    return this->connection_;
  }

  /**
   * \brief get active message-manager object
   *
   * \return current message-manager object
   */
  MessageManager* get_manager()
  {
    return &this->manager_;
  }

//  std::vector<std::string> get_joint_names()
//  {
//    return this->joint_names_;
//  }


  /**
   * \brief Add a new handler.
   *
   * \param new message-handler for a specific msg-type (ALREADY INITIALIZED).
   * \param replace existing handler (of same msg-type), if exists
   */
  void add_handler(MessageHandler* handler, bool allow_replace = true)
  {
    this->manager_.add(handler, allow_replace);
  }

//private:
  //bool set_enable_srv_callback(hsr_rosi_test::SetEnableSrv::Request &req,hsr_rosi_test::SetEnableSrv::Response &res);
//  bool stop_move_srv_callback(hsr_rosi_test::StopMoveSrv::Request &req, hsr_rosi_test::StopMoveSrv::Response &res);
//  bool clear_fault_srv_callback(hsr_rosi_test::ClearFaultSrv::Request &req, hsr_rosi_test::ClearFaultSrv::Response &res);

protected:
  TcpClient default_tcp_connection_;
 // JointRelayHandler default_joint_handler_;
  RobotSysRelayHandler default_robot_sys_handler_;

  SmplMsgConnection* connection_;
  MessageManager manager_;
  //std::vector<std::string> joint_names_;

  int mssg_type_;

//  ros::NodeHandle n_rosi;
//  ros::ServiceServer set_enable_srv;
//  ros::ServiceServer clear_fault_srv;
//  ros::ServiceServer stop_move_srv;

};//class RobotStateInterface

}//robot_state_interface
}//industrial_robot_cliet

#endif // ROBOT_ENABLE_INTERFACE_H
