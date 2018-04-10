#include "flight_control.h"
#include "dji_sdk/dji_sdk.h"
#include <tf/tf.h>
geometry_msgs::Quaternion current_atti;

static uint8_t flight_status = 255;

FlightControler::FlightControler(){}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void FlightControler::init(ros::NodeHandlePtr nh)
{
  _nh = nh;
  _flight_status_sub = _nh->subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  _attitude_sub = _nh->subscribe("dji_sdk/attitude", 10, &attitude_callback);
  _ctrl_generic = _nh->advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  _sdk_ctrl_authority_service = _nh->serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  _drone_task_service         = _nh->serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  obtain_control();
}

bool FlightControler::takeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }
  return true;
}

bool FlightControler::land()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND))
  {
      ROS_ERROR("Landing failed");
      return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // If M100 is not on the ground after 60 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(20))
  {
      ros::Duration(0.01).sleep();
      ros::spinOnce();
      if(flight_status == DJISDK::M100FlightStatus::M100_STATUS_ON_GROUND)
          break;
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_ON_GROUND)
  {
      ROS_ERROR("Landing failed.");
      return false;
  }
  else
  {
      start_time = ros::Time::now();
      ROS_INFO("Successful landing!");
      ros::spinOnce();
  }

  return true;
}

void FlightControler::ctrl_flight(const double x, const double y, const double z, const double yaw)
{
  //First stop the aircraft
  stop_flight(1);
  ctrl_yaw(yaw);
  //Rotate the aircraft by yaw angle first then move forwarding
  //Using yaw_rate and guidance pose topic to confirm aircraft meet angle rotation
}

bool FlightControler::obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  _sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }
  return true;
}

bool FlightControler::takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  _drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

void FlightControler::stop_flight(const int duration)
{
  ros::Time start_time = ros::Time::now();
  do {
    sensor_msgs::Joy controlVelYawRate;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                    DJISDK::HORIZONTAL_VELOCITY |
                    DJISDK::YAW_RATE            |
                    DJISDK::HORIZONTAL_BODY   |
                    DJISDK::STABLE_ENABLE);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(0);
    controlVelYawRate.axes.push_back(flag);
    _ctrl_generic.publish(controlVelYawRate);
    ros::Duration(0.02).sleep();
  } while ((ros::Time::now() - start_time) < ros::Duration(duration));
}

void FlightControler::ctrl_yaw(const double yaw_angle)
{
  double yawThresholdInDeg = 2;
  double yawDesiredRad     = DEG2RAD(yaw_angle);
  double yawThresholdInRad = DEG2RAD(yawThresholdInDeg);

  double yawInRad = 0;
  do {
    yawInRad = toEulerAngle(current_atti).z;
    sensor_msgs::Joy controlVelYawAngle;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                    DJISDK::HORIZONTAL_VELOCITY |
                    DJISDK::YAW_ANGLE           |
                    DJISDK::HORIZONTAL_GROUND   |
                    DJISDK::STABLE_ENABLE);
    controlVelYawAngle.axes.push_back(0);
    controlVelYawAngle.axes.push_back(0);
    controlVelYawAngle.axes.push_back(0);
    controlVelYawAngle.axes.push_back(yawDesiredRad);
    controlVelYawAngle.axes.push_back(flag);
    _ctrl_generic.publish(controlVelYawAngle);
    ros::Duration(0.02).sleep();
  } while (std::abs(yawInRad - yawDesiredRad) > yawThresholdInRad);

}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}
