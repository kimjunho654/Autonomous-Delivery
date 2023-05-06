#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher_node");
    
    ros::NodeHandle n;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);

    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    //odom.pose.pose.position.x = real_value;
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
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

        odom.header.stamp = ros::Time::now();
        odom_pub.publish(odom);
        loop_rate.sleep();

    }
    

    ros::spin();
}
