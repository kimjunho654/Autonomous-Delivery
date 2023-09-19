#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

ros::Publisher speed_pub;
ros::Publisher pid_error_pub;
ros::Publisher measure_speed_pub;

double current_speed = 0;
double measure_msg = 0;
double desired_msg = 0;
double desired_speed = 0.0;  
double measure_speed = 0.0;

double prev_error = 0.0;
double error_integral = 0.0;
double error_derivative = 0.0;
double error = 0;

int pid_output = 0;
int control_output = 0;

std_msgs::Float64 measure_speed_msg;
std_msgs::Int16 speed_msg;
std_msgs::Float64 pid_error_msg;


double Kp = 4;  // P 게인
double Ki = 2;  // I 게인
double Kd = 0; // D 게인

// 속도 PID 제어 함수
double speed_pid_control(double current_speed) {

    error = desired_speed - current_speed;
    error_integral += error;
    error_integral = (error_integral >= 255) ? 255 : error_integral;
    error_integral = (error_integral <= -255) ? -255 : error_integral;
    error_derivative = error - prev_error;

    pid_output = Kp * error + Ki * error_integral + Kd * error_derivative;

    if(desired_speed > 0){
        if(  (current_speed > 1.1) || (error >= -0.1) && (error <= 0.1) ){
            error_integral = error_integral - 3;
        }
    }
    else if(desired_speed == 0){
        if(  (error >= -0.01) && (error <= 0.01) ){
            error_integral = 0;
        }
    }
        
    prev_error = error;

    // 속도 값이 특정 범위를 벗어나면 조정
    if (pid_output > 255.0) {
        pid_output = 255.0;
    } else if (pid_output < -255.0) {
        pid_output = -255.0;
    }

    return pid_output;
}


// /odom 콜백 함수
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    measure_msg = msg->twist.twist.linear.x;

    measure_speed_msg.data = measure_msg;
    measure_speed_pub.publish(measure_speed_msg);


    // PID 제어 수행
    control_output = speed_pid_control(measure_msg);

    // 제어 출력값을 std_msgs::Int16 유형으로 변환하여 pub
    speed_msg.data = control_output;
    speed_pub.publish(speed_msg);

    pid_error_msg.data = error;
    pid_error_pub.publish(pid_error_msg);


}

void desire_speed_callback(const std_msgs::Int16::ConstPtr& msg) {
    desired_msg = msg->data;


    if (desired_msg == 230) {
        desired_speed = 1.1;
    }
    else if (desired_msg == 60) {
        desired_speed = 0.44;
    }
    else if (desired_msg == 0) {
        desired_speed = 0;
    }



}

int main(int argc, char** argv) {
    ros::init(argc, argv, "speed_pid_control_node");
    ros::NodeHandle nh;

    ros::Subscriber desire_sub = nh.subscribe("/Car_Control_cmd/Speed_Int16", 10, desire_speed_callback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odom_callback);


    speed_pub = nh.advertise<std_msgs::Int16>("PID_car_speed", 10);
    pid_error_pub = nh.advertise<std_msgs::Float64>("PID_error", 10);
    measure_speed_pub = nh.advertise<std_msgs::Float64>("measure_speed", 10);

    ros::spin();

    return 0;
}
