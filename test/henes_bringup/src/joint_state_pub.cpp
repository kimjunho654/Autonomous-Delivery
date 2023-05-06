#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_state_publisher");
    
    ros::NodeHandle nh;

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

    sensor_msgs::JointState joint_msg;

    joint_msg.name.resize(2);
    joint_msg.position.resize(2);
    joint_msg.velocity.resize(2);
    joint_msg.effort.resize(2);

    ros::Rate loop_rate(1.0);
    while (ros::ok())
    {

        joint_msg.header.stamp = ros::Time::now();

        joint_msg.header.frame_id = "base_link";

        joint_msg.name[0] = "wheel_left_joint";
        joint_msg.name[1] = "wheel_right_joint";

        joint_msg.position[0] = 0.0;
        joint_msg.position[1] = 0.0;

        joint_msg.velocity[0] = 0.0;
        joint_msg.velocity[1] = 0.0;

        joint_msg.effort[0] = -1.47479;
        joint_msg.effort[1] = -2.12403;

        joint_pub.publish(joint_msg);

        ros::spinOnce();
        loop_rate.sleep();

    }
    


}
