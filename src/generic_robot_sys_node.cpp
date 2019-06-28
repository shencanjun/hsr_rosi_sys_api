#include "robot_sys_interface.h"
#include "ros/ros.h"
#include "hsr_rosi_test/ClearFaultSrv.h"
#include "hsr_rosi_test/SetEnableSrv.h"
#include "hsr_rosi_test/StopMoveSrv.h"

using industrial_robot_client::robot_robot_sys_interface::RobotSysInterface;

//simple_message,msg_type
#define ENABLE_ON 2600
#define ENABLE_OFF 2601
#define STOP_MOVING 2602
#define CLEAR_FAULT 2603

RobotSysInterface *rsi;

bool set_enable_srv_callback(hsr_rosi_test::SetEnableSrv::Request &req,hsr_rosi_test::SetEnableSrv::Response &res)
{
    if(req.enable)
    {
        rsi = new RobotSysInterface();
        rsi->set_mssage_type(ENABLE_ON);
    }
    else
    {
        rsi = new RobotSysInterface();
        rsi->set_mssage_type(ENABLE_OFF);
    }
    res.finsh = rsi->init();
    delete rsi;
    return res.finsh;
}
bool clear_fault_srv_callback(hsr_rosi_test::ClearFaultSrv::Request &req, hsr_rosi_test::ClearFaultSrv::Response &res)
{
    rsi = new RobotSysInterface();
    rsi->set_mssage_type(CLEAR_FAULT);
    res.finsh = rsi->init();
    delete rsi;
    return res.finsh;
}

bool stop_move_srv_callback(hsr_rosi_test::StopMoveSrv::Request &req, hsr_rosi_test::StopMoveSrv::Response &res)
{
    rsi = new RobotSysInterface();
    rsi->set_mssage_type(STOP_MOVING);
    res.finsh = rsi->init();
    delete rsi;
    return res.finsh;
}

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "hsr_robot_interface");

  ros::NodeHandle n_rosi;

  ros::ServiceServer set_enable_srv = n_rosi.advertiseService("set_robot_enable",&set_enable_srv_callback);
  ros::ServiceServer stop_move_srv = n_rosi.advertiseService("stop_robot_moving",&stop_move_srv_callback);
  ros::ServiceServer clear_fault_srv = n_rosi.advertiseService("clear_robot_fault",&clear_fault_srv_callback);
  ros::spin();
  return 0;
}
