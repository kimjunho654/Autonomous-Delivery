#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flat_world_imu_node");
    
    ros::NodeHandle n;

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/flat_imu", 100);

    sensor_msgs::Imu imu_out;


    imu_out.orientation.x = 1.0;
    imu_out.orientation.y = 0.0;
    imu_out.orientation.z = 0.0;
    imu_out.orientation.w = 1.0;

    imu_out.orientation_covariance[0] = 0.0025;
    imu_out.orientation_covariance[4] = 0.0025;
    imu_out.orientation_covariance[8] = 0.0025;

    imu_out.angular_velocity.x = 0.0;
    imu_out.angular_velocity.y = 0.0;
    imu_out.angular_velocity.z = 0.0;

    imu_out.angular_velocity_covariance[0] = 0.02;
    imu_out.angular_velocity_covariance[4] = 0.02;
    imu_out.angular_velocity_covariance[8] = 0.02;

    imu_out.linear_acceleration.x = 0.0;
    imu_out.linear_acceleration.y = 0.0;
    imu_out.linear_acceleration.z = 9.8;

    imu_out.linear_acceleration_covariance[0] = 0.04;
    imu_out.linear_acceleration_covariance[4] = 0.04;
    imu_out.linear_acceleration_covariance[8] = 0.04;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        imu_out.header.frame_id = "imu_link";
        imu_out.header.stamp = ros::Time::now();
        imu_pub.publish(imu_out);
        loop_rate.sleep();
    }
    


    ros::spin();
}
