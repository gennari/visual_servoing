#include <tf/transform_listener.h>
#include "visual_servoing_node.h"

void initWayPoints(){
	geometry_msgs::Pose2D primo;
	primo.x=-30;
	primo.y=2;
	primo.theta=1.57;
	pose.push_back(primo);
	primo.x=-32;
	primo.y=2;
	primo.theta=1.57;
	pose.push_back(primo);
	primo.x=-32;
	primo.y=0;
	primo.theta=1.57;
	pose.push_back(primo);
	primo.x=-30;
	primo.y=0;
	primo.theta=1.57;
	pose.push_back(primo);
	next_pose=0;

}

void followTag(apriltags_ros::AprilTagDetection tag){
	ROS_INFO("I heard: [%f]", tag.pose.pose.position.x);

    	float tv = 0;
    	float rv = 0;
        tv=kt*(tag.pose.pose.position.z-0.7);
        rv=kr*tag.pose.pose.position.x;
    
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
    	cmd_vel.linear.x = tv;
    	cmd_vel.angular.z = rv;
    	cmd_vel_pub.publish(cmd_vel);

}
void sendNewGoal()
{

   
   move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = pose[next_pose].x;
  goal.target_pose.pose.position.y = pose[next_pose].y;

  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose[next_pose].theta);

  ROS_INFO("Sending goal");
  ac->sendGoal(goal,&doneCb);


}
void doneCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result)
{
	if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("The goal was reached!");
		next_pose=(int)(next_pose+1)%((int)pose.size());
		sendNewGoal();
	}
	if(state.state_ == actionlib::SimpleClientGoalState::ABORTED)
		ROS_WARN("Failed to reach the goal...");

}

void tagCallback(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg)
{   

    if(msg->detections.size()>0 ){
	ac->cancelAllGoals();
	switch(msg->detections[0].id)
	{
	    case (0):
		//DO SOMETHING
		break;

	    case (1):
		followTag(msg->detections[0]);
		break;
	     
	    case (2):
		//DO SOMETHING
		break;

	    default:   
		cmd_vel.linear.x = 0;
	    	cmd_vel.angular.z = 0;
	    	cmd_vel_pub.publish(cmd_vel);
	}
        
    }else{
    	
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "visual_servoing");

    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    nh.param("max_rv", _max_rv, 1.0);
    nh.param("max_tv", _max_tv, 0.3);
    nh.param("kt", kt, 3.0);
    nh.param("kr", kr, 5.0);
    initWayPoints();    

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe("/tag_detections", 1, tagCallback);
    ac=new MoveBaseClient("move_base", true);

    //wait for the action server to come up
    while(!ac->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    sendNewGoal();


    ros::spin();


    return 0;
}
