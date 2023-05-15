#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

// 전역 변수
double tickToRad = 0.001533981;  // 엔코더 틱을 미터로 변환하는 상수
double WHEEL_RADIUS = 0.033;            // 로봇의 바퀴 간격 (미터)

double x = 0.0;                    // 로봇의 x 위치 (미터)
double y = 0.0;                    // 로봇의 y 위치 (미터)
double theta = 0.0;                // 로봇의 각도 (라디안)
double last_theta = 0.0; 

ros::Publisher odom_pub;

nav_msgs::Odometry odom;
geometry_msgs::Quaternion odom_quat;
tf2::Quaternion q;


int last_ticks = 0;
int ticks;
int dummy_tick;
double wheel;
double delta_s;
float orientation[4];
double delta_theta;
double delta_x = 0.0;
double delta_y = 0.0;
float odom_pose[3];

double step_time = 0.0;
ros::Time current_time;
double curr_time;
double prev_time = 0.0;
double v, w;

void encoderCallback(const std_msgs::Int32::ConstPtr& msg)
{

  dummy_tick = msg->data;
  ticks = dummy_tick - last_ticks;

}

void updateOdometry() {

  current_time = ros::Time::now();
  curr_time = current_time.toSec();
  // 엔코더 틱을 거리로 변환
  wheel = ticks * tickToRad;
  delta_s = wheel * WHEEL_RADIUS;
  orientation[0] = orientation[1] = orientation[2] = 0.0;
  orientation[3] = 1.0;
  
  theta = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 0.5 - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta;  

  // 로봇의 이동 거리 계산
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  step_time = curr_time - prev_time;

  v = delta_s / step_time;
  w = delta_theta / step_time;

  prev_time = curr_time;
  
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;

  // Quaternion을 사용하여 로봇의 방향 표현
  q.setRPY(0, 0, theta);
  odom_quat.x = q.x();
  odom_quat.y = q.y();
  odom_quat.z = q.z();
  odom_quat.w = q.w();
  odom.pose.pose.orientation = odom_quat;
  odom.header.stamp = current_time;
  // 토픽 발행


}


int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_test_node");
    ros::NodeHandle node;

    ros::Subscriber subForLeftCounts = node.subscribe("encoder", 1, encoderCallback, ros::TransportHints().tcpNoDelay());

    odom_pub = node.advertise<nav_msgs::Odometry>("odom", 1); 



    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped odom_tf;

    ros::Rate loop_rate(10); 
    while(ros::ok()) {
/*
      odom_tf.header.stamp = ros::Time::now();
      odom_tf.header.frame_id = "map";
      odom_tf.child_frame_id = "odom";
      odom_tf.transform.translation.x = odom.pose.pose.position.x;
      odom_tf.transform.translation.y = odom.pose.pose.position.y;
      odom_tf.transform.translation.z = 0.0;
      odom_tf.transform.rotation.x = 0;
      odom_tf.transform.rotation.y = 0;
      odom_tf.transform.rotation.z = 0.0;
      odom_tf.transform.rotation.w = 1.0;
      tf_broadcaster.sendTransform(odom_tf);
*/
      updateOdometry();
      odom_pub.publish(odom);

      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}
