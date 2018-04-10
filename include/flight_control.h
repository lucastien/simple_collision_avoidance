#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>

//DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>

#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

class FlightControler
{
public:
  FlightControler();
  void init(ros::NodeHandlePtr nh);
  bool takeoff();
  bool land();
  void ctrl_flight(const double x, const double y, const double z, const double yaw);
private:
  ros::NodeHandlePtr _nh;
  ros::Subscriber _flight_status_sub;
  ros::Subscriber _attitude_sub;
  ros::Publisher _ctrl_generic;
  ros::ServiceClient _sdk_ctrl_authority_service;
  ros::ServiceClient _drone_task_service;

  uint8_t _flight_status;
  //Define private function here
  bool obtain_control();
  bool takeoff_land(int task);
  void stop_flight(const int duration = 0); //duration in second
  void ctrl_yaw(const double yaw_angle); //yaw_angle in degree
};

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
#endif // FLIGHT_CONTROL_H
