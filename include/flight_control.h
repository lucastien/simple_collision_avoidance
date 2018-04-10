#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
//DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>

class FlightControler
{
public:
  FlightControler();
};

#endif // FLIGHT_CONTROL_H
