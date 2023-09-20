#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sigmoid_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float64>("sigmoid_values", 1000);
    ros::Rate rate(10);  // 퍼블리시 메시지 속도 (10 Hz)

    ros::Time init_time = ros::Time::now();

    while (ros::ok())
    {
        // 현재 ROS 시간을 사용하여 x 값을 생성 (ROS 시간을 사용하려면 roscore가 실행 중이어야 함)
        ros::Time current_time = ros::Time::now();
        double delta_time =  current_time.toSec() - init_time.toSec();  // x 값을 계산
        double x = delta_time - 4;

        // 시그모이드 함수 계산
        double sigmoid_value = 1.0 / (1.0 + std::exp(-x));


        // 메시지 생성 및 발행
        std_msgs::Float64 msg;
        msg.data = sigmoid_value;
        pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
