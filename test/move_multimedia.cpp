#include "ros/ros.h" 
#include "geometry_msgs/PoseStamped.h" 
#include "geometry_msgs/PoseWithCovarianceStamped.h" 
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <std_msgs/String.h>
#include <iostream>

using namespace std;


string target_place;

void string_callback(const std_msgs::String &msg) {

  target_place = msg.data;
}


move_base_msgs::MoveBaseActionGoal goal_msg;

void move_M101(void) {

  goal_msgs.goal.target_pose.header.stamp = ros::Time::now();
  goal_msgs.goal.target_pose.header.frame_id = "map";
  goal_msgs.goal.target_pose.pose.postion.x = 0.0;
  goal_msgs.goal.target_pose.pose.postion.y = 0.0;
  goal_msgs.goal.target_pose.pose.postion.z = 0.0;
  goal_msgs.goal.target_pose.pose.orientation.x = 0.0;
  goal_msgs.goal.target_pose.pose.orientation.y = 0.0;
  goal_msgs.goal.target_pose.pose.orientation.z = 0.0;
  goal_msgs.goal.target_pose.pose.orientation.w = 0.0;
  
  goal_pub.publish(goal_msg);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_multimedia");
  ros::NodeHandle node;
  ros::Publisher goal_pub = node.advertise<move_base_msgs::MoveBaseActionGoal>("henes_goal", 10);

  ros::Subscriber string_sub = node.subscribe("string_multimedia", 10, string_callback);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    
    if(target_place == "M101"){

      move_M101();
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
