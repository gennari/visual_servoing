#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "apriltags_ros/AprilTagDetectionArray.h"
#include <sstream>

ros::Publisher cmd_vel_pub;
float _max_tv,_max_rv,kt,kr;
void tagCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg)
{   geometry_msgs::Twist cmd_vel;
    float tv = 0;
    float rv = 0;


    geometry_msgs::Twist cmd_vel_;

    if(msg->detections.size()>0 && msg->detections[0].id==54){
        apriltags_ros::AprilTagDetection tag=msg->detections[0];
        ROS_INFO("I heard: [%f]", tag.pose.pose.position.x);
        tv=kt*(tag.pose.pose.position.z-0.5);
        rv=kr*tag.pose.pose.position.x;
    }
    if (fabs(rv)>_max_rv){
        float scale=_max_rv/fabs(rv);
        tv*=scale;
        rv*=scale;
    }

    if (fabs(tv)>_max_tv){
        float scale=_max_tv/fabs(tv);
        tv*=scale;
        rv*=scale;
    }
    if(fabs(tv)<0.05){
        tv=0;
    }
    if(fabs(rv)<0.01){
        rv=0;
    }
    cmd_vel_.linear.x = tv;
    cmd_vel_.angular.z = rv;
    cmd_vel_pub.publish(cmd_vel_);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "visual_servoing");

    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    nh.param("max_rv", _max_rv, 1.0f);
    nh.param("max_tv", _max_tv, 0.5f);
    nh.param("kt", kt, 3.0f);
    nh.param("kr", kr, -5.0f);

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe("/tag_detections", 1, tagCallback);
    ros::spin();


    return 0;
}
