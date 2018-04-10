#include "simplecollisionavoidance.h"
#include <ros/ros.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "flight_control.h"
//#include "dji_sdk/dji_sdk.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


using namespace cv;
using namespace std;

#define WIDTH 640
#define HEIGHT 480
#define WIN_SIZE_W 70
#define WIN_SIZE_H 70
#define HORIZ_FOV 60.0  //Horizontal field of view of Orbbec depth camera
#define VERT_FOV  49.5  //Vertical field of view of Orbbec depth camera



ros::Subscriber depth_img_sub;
ros::Subscriber rgb_img_sub;
ros::Publisher ctrl_roll_pitch_yawrate_pub;
ros::Publisher ctrl_generic_pub;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

cv::Rect targetRect;
cv::Rect refRect;
const int thresh = 25;


void rgb_image_callback(const sensor_msgs::Image& rgb_img)
{
  cv_bridge::CvImagePtr cv_ptr;

  try {
      cv_ptr = cv_bridge::toCvCopy(rgb_img);
  }
  catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  if(!targetRect.empty()){
    rectangle(cv_ptr->image, targetRect, Scalar(0, 255, 0), 1);
  }
  namedWindow( "RGB", CV_WINDOW_AUTOSIZE );
  imshow( "RGB", cv_ptr->image );
  cv::waitKey(1);
}


void depth_image_callback(const sensor_msgs::Image& depth_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    //cv_ptr->image.getMaxPixelValue();
    //cv::Mat depth8(HEIGHT, WIDTH, CV_8UC1);
    //double min, max;
    //minMaxIdx(cv_ptr->image, &min, &max);
    //cv_ptr->image.convertTo(depth8, CV_8UC1, 255.0/(max-min), -min);
    cv::Mat depth8 = cv_ptr->image*256/16384;
    depth8.convertTo(depth8, CV_8UC1);
    SimpleCollisionAvoidance simpleCollisionAvoidance(depth_img.width, depth_img.height,
                                                      WIN_SIZE_W, WIN_SIZE_H, HORIZ_FOV, VERT_FOV, 200.0);
    if(targetRect.empty()){
      refRect = Rect(Point(depth_img.width/2 - WIN_SIZE_W/2, depth_img.height/2 - WIN_SIZE_H/2), Size(WIN_SIZE_W, WIN_SIZE_H));
    }else{
      refRect = targetRect;
    }
    int direction = simpleCollisionAvoidance.startDetection(depth8, thresh, refRect);
    if(direction == STOP){
      targetRect = Rect(Point(0, 0), Size(-1, -1));
    }
    simpleCollisionAvoidance.visualizeDirection(FINAL_RESULT_ONLY);
    targetRect = simpleCollisionAvoidance.getTargetRect();
    cv::waitKey(1);
}

/** @function main */
int main( int argc, char** argv )
{
  ros::init(argc, argv, "collision_avoidance_demonstration");
  ros::NodeHandle nh;
  depth_img_sub = nh.subscribe("camera/depth/image_raw", 1, depth_image_callback);
  rgb_img_sub = nh.subscribe("camera/rgb/image_raw", 1, rgb_image_callback);
  while (ros::ok())
      ros::spinOnce();
  return(0);
}
