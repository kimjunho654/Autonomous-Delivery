#include <ros/ros.h>
#include <map>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

sensor_msgs::LaserScan scan_msg;
std_msgs::Bool detect_msg;
std_msgs::Bool avoid_function_msg;
std_msgs::Float64MultiArray avoid_heading_angle_msg;
ros::Time detection_start_time;

int INDEX;
int size;
int detect_count;
int object_right_angle_index;
double object_right_angle;
int object_left_angle_index;
double object_left_angle;

ros::Publisher detect_pub;
ros::Publisher avoid_heading_angle_pub;
ros::Publisher avoid_function_start_pub;

long map(long x, long in_min, long in_max,long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  
}

void scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    
    // object_detect
    for(int i = -330; i <= 330; i++){
        size = msg->ranges.size();
        INDEX = i + size/2;


        if(msg->ranges[INDEX] <= 2.3) {
            if(detect_count == 0) { object_right_angle_index = INDEX; }
            else if (detect_count != 0) { object_left_angle_index = INDEX; }
            detect_count ++;
        }
        if(detect_count >= 1) {
            detect_msg.data = true;
            detect_pub.publish(detect_msg);
        }
        else {
            detect_msg.data = false;
            detect_pub.publish(detect_msg);
        }
        


        //ROS_INFO("detect_count : %d", detect_count);
        ROS_INFO("detect_msg : %d", detect_msg.data);
        //ROS_INFO("range : %f", msg->ranges[INDEX]);
    }
    
    // avoid_function
    if( detect_msg.data == true && detection_start_time.isZero() ) { detection_start_time = ros::Time::now(); }
    if( detect_msg.data == false ) { detection_start_time = ros::Time(0); }

    if(detect_msg.data && (ros::Time::now() - detection_start_time).toSec() >= 0.1 ) {
            
        avoid_function_msg.data = true;
        avoid_function_start_pub.publish(avoid_function_msg);
    }
    else if(!detect_msg.data || (ros::Time::now() - detection_start_time).toSec() <= 0.1 ) { 
         
       avoid_function_msg.data = false; 
       avoid_function_start_pub.publish(avoid_function_msg);
    }

    // avoid angle calculate
    object_right_angle = map(object_right_angle_index, 0, size-1, 0, 360) - 180;
    object_left_angle = map(object_left_angle_index, 0, size-1, 0, 360) - 180;

    avoid_heading_angle_msg.data[0] = object_right_angle; // [0]
    avoid_heading_angle_msg.data[1] = object_left_angle;  // [1]
    avoid_heading_angle_pub.publish(avoid_heading_angle_msg);

    ROS_INFO("object_right_angle : %f", object_right_angle);
    ROS_INFO("object_left_angle : %f", object_left_angle);

    detect_msg.data = false;
    detect_count = 0;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_detect");
    ros::NodeHandle nh;

    detect_msg.data = false;
    avoid_function_msg.data = false;
    detect_count = 0;
    avoid_heading_angle_msg.data.resize(2);

    detect_pub = nh.advertise<std_msgs::Bool>("lidar_object_detect", 1);
    avoid_function_start_pub = nh.advertise<std_msgs::Bool>("avoid_function_start", 10);
    avoid_heading_angle_pub = nh.advertise<std_msgs::Float64MultiArray>("avoid_heading_angle", 10);
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scan_Callback);

    ros::spin();
    return 0;


}
