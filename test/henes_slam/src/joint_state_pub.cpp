#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_state_publisher");
    
    ros::NodeHandle n;

    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    nav_msgs::Odometry joint_msg;
    joint_msg.header.frame_id = "base_link";
    joint_msgs.name.push_back("wheel_left_joint");
    joint_msgs.name.push_back("wheel_right_joint");


    joint_msgs.position.push_back("0.0");
    joint_msgs.position.push_back("0.0");

    joint_msgs.velocity.push_back("0.0");
    joint_msgs.velocity.push_back("0.0");

    joint_msgs.effort.push_back("0.0");
    joint_msgs.effort.push_back("0.0");

    ros::Rate loop_rate(10);
    while (ros::ok())
    {

        joint_msg.header.stamp = ros::Time::now();
        joint_pub.publish(joint_msg);
        loop_rate.sleep();

    }
    

    ros::spin();
}
