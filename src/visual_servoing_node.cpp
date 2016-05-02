#include <tf/transform_listener.h>
#include "visual_servoing_node.h"
enum Mode { STOP=0, FOLLOW=1 ,SQUARE=2, WORK=3, OBSTACLE=4 };
Mode operation_mode=STOP;
void initWayPoints(){
	geometry_msgs::Pose2D primo;
	primo.x=0;
	primo.y=1;
	primo.theta=1.57;
	pose.push_back(primo);
	primo.x=1;
	primo.y=1;
	primo.theta=0;
	pose.push_back(primo); 
	primo.x=1;
	primo.y=0;
	primo.theta=-1.57;
	pose.push_back(primo);
	primo.x=0;
	primo.y=0;
	primo.theta=3.14;
	pose.push_back(primo);
	next_pose=0;
	
	work_pose=0;
	primo.x=0;
	primo.y=0.5;
	primo.theta=0;
	work.push_back(primo);
	primo.x=0;
	primo.y=-0.5;
	primo.theta=-1.57;
	work.push_back(primo);
        primo.x=0;
	primo.y=-0.5;
	primo.theta=0;
	work.push_back(primo);
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

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	if(operation_mode!=FOLLOW){	
                bool free=true;
		for(size_t i=0; i<msg->ranges.size();i++){
			float r=msg->ranges[i];
			if (r<msg->range_min || r>msg->range_max)
				continue;
			if (r<male){
				ROS_INFO("STOP");
				ac->cancelAllGoals();
				cmd_vel.linear.x = 0;
			    	cmd_vel.angular.z = 0;
			    	cmd_vel_pub.publish(cmd_vel);
				operation_mode=OBSTACLE;
				free=false;
				break;
					
			}	
		}
		if (free && operation_mode==OBSTACLE){
			operation_mode=SQUARE;
			sendNewGoal();		
		}
	}

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
	if (msg->detections[0].id!=operation_mode || msg->detections[0].id==FOLLOW){
		switch(msg->detections[0].id)
		{
		    case (0):
  			ROS_INFO("STOP");
			ac->cancelAllGoals();
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
			cmd_vel_pub.publish(cmd_vel);
			operation_mode=STOP;
			break;
		    case (1):
			ROS_INFO("FOLLOW");
			ac->cancelAllGoals();
			operation_mode=FOLLOW;
			followTag(msg->detections[0]);
			break;
		     
		    case (2):
		    case (5):
			ROS_INFO("SQUARE");
			operation_mode=SQUARE;
			sendNewGoal();
			break;
		    case (3):
			ROS_INFO("WORK");
			ac->cancelAllGoals();
			operation_mode=WORK;
			break;

		    default:   
			cmd_vel.linear.x = 0;
		    	cmd_vel.angular.z = 0;
		    	cmd_vel_pub.publish(cmd_vel);
		}
	}
        
    }else{
	if(operation_mode==FOLLOW){
		cmd_vel.linear.x = 0;
	    	cmd_vel.angular.z = 0;
	    	cmd_vel_pub.publish(cmd_vel);	
	}else if(operation_mode==STOP || operation_mode==OBSTACLE){
		
		cmd_vel.linear.x = 0;

	    	cmd_vel.angular.z = 0;
	    	cmd_vel_pub.publish(cmd_vel);	
	}
    	
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
    nh.param("kr", kr, -5.0);
    nh.param("male", male, 0.4);
    initWayPoints();    

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe("/tag_detections", 1, tagCallback);
    ros::Subscriber lase_sub = n.subscribe("/scan", 1, scanCallback);
    ac=new MoveBaseClient("move_base", true);

    //wait for the action server to come up
    int c=0;
    while(!ac->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
      c++;
      if(c>5){
      return 0;
      }
    }
    


    ros::spin();


    return 0;
}
