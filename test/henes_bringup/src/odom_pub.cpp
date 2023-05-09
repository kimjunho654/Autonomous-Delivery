#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <math.h>

geometry_msgs::Pose2D curr_locate;
geometry_msgs::Pose2D prev_locate;
geometry_msgs::Pose2D tf_frame;

void current_position_callback(const geometry_msgs::PoseStamped &msg){

    curr_locate.x = msg.pose.position.x;
    curr_locate.y = msg.pose.position.y;

}

double delta_x;
double delta_y;

double seta_angle;


void tf_calculate(){

    delta_x = curr_locate.x - prev_locate.x;
    delta_y = curr_locate.y - prev_locate.y;
    seta_angle = atan2(delta_y, delta_x);

    tf_frame.x = delta_x * cos(seta_angle) - delta_y * sin(seta_angle);
    tf_frame.y = delta_x * sin(seta_angle) + delta_y * cos(seta_angle);

    prev_locate.x = curr_locate.x;
}

double plus_x, plus_y;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher_node");

    ros::NodeHandle n;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
    ros::Subscriber sub = n.subscribe("initial_2d", 1, current_position_callback);

    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    //odom.pose.pose.position.x = real_value;
    odom.pose.pose.position.x = curr_locate.x;
    odom.pose.pose.position.y = curr_locate.y;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;

    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = 1.0;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        tf_calculate();

        odom.header.stamp = ros::Time::now();
  
        plus_x = tf_frame.x;
        plus_y = tf_frame.y;

        odom.pose.pose.position.x = odom.pose.pose.position.x + plus_x;
        odom.pose.pose.position.y = odom.pose.pose.position.y + plus_y;
        odom.pose.pose.orientation.z = seta_angle;

        odom_pub.publish(odom);
        loop_rate.sleep();

    }

    ros::spin();
}
