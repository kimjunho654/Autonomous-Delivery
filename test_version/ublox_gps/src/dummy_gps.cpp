#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <random>

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "navsatfix_publisher");
    ros::NodeHandle nh;

    // Create a publisher for /ublox_gps/fix topic
    ros::Publisher fix_pub = nh.advertise<sensor_msgs::NavSatFix>("/ublox_gps/fix", 10);

    // Set the loop rate (in Hz)
    ros::Rate rate(1); // 1 Hz in this example

    while (ros::ok())
    {
        // Create a NavSatFix message
        sensor_msgs::NavSatFix fix_msg;

        // Generate random latitude, longitude, and altitude values for testing
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> lat_dist(36, 35); // Random latitude between 0 and 90 degrees
        std::uniform_real_distribution<> lon_dist(126, 125); // Random longitude between -180 and 180 degrees
        std::uniform_real_distribution<> alt_dist(0, 0); // Random altitude between 0 and 1000 meters

        fix_msg.latitude = lat_dist(gen);
        fix_msg.longitude = lon_dist(gen);
        fix_msg.altitude = alt_dist(gen);

        // Publish the NavSatFix message
        fix_pub.publish(fix_msg);

        // Sleep for the desired loop rate
        rate.sleep();

        ros::spinOnce();
    }

    return 0;
}
