#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>





int main(int argc, char** argv){
    ros::init(argc, argv, "odom_base_footprint_tf_publisher");
    ros::NodeHandle nh;

    //ros::Subscriber odom_sub = node.subscribe("odom", 1, odom_callback);

    tf2_ros::TransformBroadcaster tf_msg;
    geometry_msgs::TransformStamped transformStamped;

    ros::Rate loop_rate(10);
    while(nh.ok()){
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_footprint";
        transformStamped.transform.translation.x = 0.0; ////
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;
        tf_msg.sendTransform(transformStamped);

        loop_rate.sleep();
    }

    ros::spin();

    return 0;
}

