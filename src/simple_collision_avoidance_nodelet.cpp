#include "simple_collision_avoidance_nodelet.h"
#include "simplecollisionavoidance.h"
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


using namespace cv;
using namespace std;
cv::Rect targetRect;
cv::Rect refRect;
const int thresh = 25;
namespace collision_avoidance_pkg {

void SimpleCollisionAvoidanceNodelet::rgb_image_cb(const sensor_msgs::Image& rgb_img)
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


void SimpleCollisionAvoidanceNodelet::depth_image_cb(const sensor_msgs::Image& depth_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat depth8 = cv_ptr->image*256/16384;
    depth8.convertTo(depth8, CV_8UC1);
    SimpleCollisionAvoidance simpleCollisionAvoidance(depth_img.width, depth_img.height,
                                                      win_size_w_, win_size_h_, horiz_fov_, vert_fov_, 200.0);
    if(targetRect.empty()){
      refRect = Rect(Point(depth_img.width/2 - win_size_w_/2, depth_img.height/2 - win_size_h_/2), Size(win_size_w_, win_size_h_));
    }else{
      refRect = targetRect;
    }
    int direction = simpleCollisionAvoidance.startDetection(depth8, thresh, refRect);
    if(direction == STOP){
      targetRect = Rect(Point(0, 0), Size(-1, -1));
    }else{
      float x, y, z, yaw;
      x = y = z = yaw = 0.0f;
      simpleCollisionAvoidance.getTargetCoordinate(x, y, z, yaw);
      //controller.ctrl_flight(x, y, z, yaw);
    }
    simpleCollisionAvoidance.visualizeDirection(FINAL_RESULT_ONLY);
    targetRect = simpleCollisionAvoidance.getTargetRect();
    cv::waitKey(1);
}
    SimpleCollisionAvoidanceNodelet::SimpleCollisionAvoidanceNodelet()
    {

    }


    void SimpleCollisionAvoidanceNodelet::onInit()
    {
        ROS_INFO("Initializing simple collision avoidance nodelet here");
        nh_ = getNodeHandle();
        nh_private_ = getPrivateNodeHandle();
        nh_private_.param<double>("horizontal_fov", horiz_fov_, (double) 60.0);
        nh_private_.param<double>("vertical_fov", horiz_fov_, (double) 49.5);
        nh_private_.param<double>("win_size_width", win_size_w_, (double) 70.0);
        nh_private_.param<double>("win_size_height", win_size_h_, (double) 70.0);
        ros::Subscriber depth_img_sub = nh_.subscribe("camera/depth/image_raw", 1, &SimpleCollisionAvoidanceNodelet::depth_image_cb, this);
        ros::Subscriber rgb_img_sub = nh_.subscribe("camera/rgb/image_raw", 1, &SimpleCollisionAvoidanceNodelet::rgb_image_cb, this);
    }

}

PLUGINLIB_EXPORT_CLASS(collision_avoidance_pkg::SimpleCollisionAvoidanceNodelet, nodelet::Nodelet);
