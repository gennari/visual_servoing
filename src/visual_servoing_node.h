#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "apriltags_ros/AprilTagDetectionArray.h"
#include <sstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher cmd_vel_pub;

double _max_tv,_max_rv,kt,kr;
geometry_msgs::Twist cmd_vel;
MoveBaseClient* ac;
std::vector<geometry_msgs::Pose2D> pose;
int next_pose;

void initWayPoints();

void followTag(apriltags_ros::AprilTagDetection tag);

void sendNewGoal();

void doneCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);


void tagCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg);

