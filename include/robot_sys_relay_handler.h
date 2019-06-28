#ifndef ROBOT_ENABLE_RELAY_HANDLER_H
#define ROBOT_ENABLE_RELAY_HANDLER_H

#include "ros/ros.h"
#include "simple_message/message_handler.h"
#include "robot_sys_message.h"


namespace industrial_robot_client
{
namespace robot_sys_relay_handler
{

/**
 * \brief Message handler that relays joint positions (converts simple message
 * types to ROS message types and publishes them)
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class RobotSysRelayHandler : public industrial::message_handler::MessageHandler
{
  // since this class defines a different init(), this helps find the base-class init()
  using industrial::message_handler::MessageHandler::init;

public:

  /**
* \brief Constructor
*/
  RobotSysRelayHandler() {};


 /**
  * \brief Class initializer
  *
  * \param connection simple message connection that will be used to send replies.
  *
  * \return true on success, false otherwise (an invalid message type)
  */
 bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection,int mssg_type);

protected:

  //ros::Publisher pub_robot_enable_;
  ros::NodeHandle node_;

  /**
   * \brief Callback executed upon receiving a robot  message
   *
   * \param in incoming message
   *
   * \return true on success, false otherwise
   */
  bool internalCB(industrial::robot_sys_message::RobotSysMessage & in);

  bool internal(industrial::robot_sys_message::RobotSysMessage & in);

private:
 /**
  * \brief Callback executed upon receiving a message
  *
  * \param in incoming message
  *
  * \return true on success, false otherwise
  */
 bool internalCB(industrial::simple_message::SimpleMessage& in);

 bool internal(industrial::simple_message::SimpleMessage& in,int mssg_type);

 bool sendsimple_message(int mssg_type);
};

}
}

#endif // ROBOT_ENABLE_RELAY_HANDLER_H
