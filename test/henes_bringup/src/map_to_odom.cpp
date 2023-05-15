#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_to_odom_publisher");
    ros::NodeHandle nh;

    tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transformStamped;

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "odom";
        transformStamped.transform.translation.x = 0.0; ///
        transformStamped.transform.translation.y = 0.0;  ///
        transformStamped.transform.translation.z = -0.078; 
        transformStamped.transform.rotation.x = 0.0;  
        transformStamped.transform.rotation.y = 0.0;   
        transformStamped.transform.rotation.z = 0.0;   ////
        transformStamped.transform.rotation.w = 1.0;    

        broadcaster.sendTransform(transformStamped);

        loop_rate.sleep();

        ros::spinOnce();
    }

    return 0;
}
