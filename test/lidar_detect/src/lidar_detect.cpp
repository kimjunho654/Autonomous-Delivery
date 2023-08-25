#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

sensor_msgs::LaserScan scan_msg;
std_msgs::Bool detect_msg;


int INDEX;
int size;
int detect_count;

ros::Publisher detect_pub;

void scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){

    for(int i = -30; i <= 30; i++){
        size = msg->ranges.size();
        INDEX = i + size/2;
        if(msg->ranges[INDEX] <= 1.5) {detect_count ++;}
        if(detect_count >= 1) {
            detect_msg.data = true;
            detect_pub.publish(detect_msg);
        }
        else {
            detect_msg.data = false;
            detect_pub.publish(detect_msg);
        }

        ROS_INFO("detect_count : %d", detect_count);
        ROS_INFO("detect_msg : %d", detect_msg.data);
        //ROS_INFO("range : %f", msg->ranges[INDEX]);
    }
    detect_msg.data = false;
    detect_count = 0;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_detect");
    ros::NodeHandle nh;

    detect_msg.data = false;
    detect_count = 0;


    detect_pub = nh.advertise<std_msgs::Bool>("lidar_object_detect", 10);
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scan_Callback);

    ros::spin();
    return 0;


}
