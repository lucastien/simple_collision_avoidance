#ifndef SIMPLE_COLLISION_AVOIDANCE_NODELET_H
#define SIMPLE_COLLISION_AVOIDANCE_NODELET_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
namespace collision_avoidance_pkg {
    class SimpleCollisionAvoidanceNodelet: public nodelet::Nodelet
    {
    public:
        SimpleCollisionAvoidanceNodelet();

        // Nodelet interface
    private:
        virtual void onInit();
        void depth_image_cb(const sensor_msgs::Image& depth);
        void rgb_image_cb(const sensor_msgs::Image& rgb);
        ros::NodeHandle nh_, nh_private_;
        double horiz_fov_, vert_fov_, win_size_h_, win_size_w_;
    };
}


#endif // SIMPLE_COLLISION_AVOIDANCE_NODELET_H
