#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <cmath>

ros::Publisher speed_pub;
ros::Publisher pid_error_pub;
ros::Publisher measure_speed_pub;
ros::Publisher desire_speed_pub;

double current_speed = 0;
double measure_msg = 0;
double desired_msg = 0;
double desired_speed = 0.0;  
double measure_speed = 0.0;

double prev_error = 0.0;
double error_integral = 0.0;
double error_derivative = 0.0;
double error = 0;
double sigmoid_value = 0;
double delta_time = 0;

int pid_output = 0;
int control_output = 0;

bool speed_230 = false;
bool speed_60 = false;
bool speed_0 = false;

bool sp_60_to_230 = false;

bool init_start_230 = true;
bool init_start_60 = true;
bool init_start_0 = true;

double x = 0;

double a = 0;
double b = 0;

double a0 = 0;
double a1 = 0;
double a2 = 0;
double a3 = 0;
double a4 = 0;
double a5 = 0;
double a6 = 0;

double prev_desire_speed = 0;

std_msgs::Float64 measure_speed_msg;
std_msgs::Float64 desire_speed_msg;
std_msgs::Int16 speed_msg;
std_msgs::Float64 pid_error_msg;
ros::Time init_time;

double Kp = 0; //190;  // P 게인  // pid test indoor 70
double Ki = 12; //3.5;  // I 게인
double Kd = 0; //4; // D 게인

long map(long x, long in_min, long in_max,long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}

// 속도 PID 제어 함수
double speed_pid_control(double current_speed) {

    error = desired_speed - current_speed;
    error_integral += error;
    error_integral = (error_integral >= 255) ? 255 : error_integral;
    error_integral = (error_integral <= -255) ? -255 : error_integral;
    error_derivative = error - prev_error;

    pid_output = Kp * error + Ki * error_integral + Kd * error_derivative;

    if(speed_230 == true){

        if(current_speed >= 0.1){
            if(  (current_speed > desired_speed) || (error >= -0.05) && (error <= 0.05) ){
                error_integral = error_integral - 0.1;
            }
            else if( pid_output > 200 && current_speed > 0.8 ) {
                error_integral = error_integral - 3;
            }
        }
    }
    else if(speed_0 == true){
        if(  (current_speed >= -0.05) && (current_speed <= 0.05) ){
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

    desire_speed_msg.data = desired_speed;
    desire_speed_pub.publish(desire_speed_msg);



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

    if(init_start_60 == false && desired_msg == 230){
        a = desired_speed;
        sp_60_to_230 = true;
        prev_desire_speed = desired_speed;
    }
    /*
    else {
        sp_60_to_230 = false;
    }
    */



    if (desired_msg == 230) {
        //desired_speed = 1.1;
        speed_230 = true;
        speed_60 = false;
        speed_0 = false;
        if(init_start_230 == true){
            init_time = ros::Time::now();
            init_start_230 = false;
            init_start_60 = true;
            init_start_0 = true;
        }

    }
    else if (desired_msg == 60) {
        //desired_speed = 0.44;
        speed_230 = false;
        speed_60 = true;
        speed_0 = false;
        if(init_start_60 == true){
            init_time = ros::Time::now();
            init_start_230 = true;
            init_start_60 = false;
            init_start_0 = true;
        }
    }
    else if (desired_msg == 0) {
        //desired_speed = 0;
        speed_230 = false;
        speed_60 = false;
        speed_0 = true;
        if(init_start_0 == true){
            init_time = ros::Time::now();
            init_start_230 = true;
            init_start_60 = true;
            init_start_0 = false;
        }
    }



}

int main(int argc, char** argv) {
    ros::init(argc, argv, "speed_pid_control_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(5);  // 퍼블리시 메시지 속도 (10 Hz)

    ros::Subscriber desire_sub = nh.subscribe("/Car_Control_cmd/Speed_Int16", 10, desire_speed_callback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odom_callback);


    speed_pub = nh.advertise<std_msgs::Int16>("PID_car_speed", 10);
    pid_error_pub = nh.advertise<std_msgs::Float64>("PID_error", 10);
    measure_speed_pub = nh.advertise<std_msgs::Float64>("measure_speed", 10);
    desire_speed_pub = nh.advertise<std_msgs::Float64>("desired_speed", 10);


    while (ros::ok())
    {
        if(speed_230 == true){
            // 현재 ROS 시간을 사용하여 x 값을 생성 (ROS 시간을 사용하려면 roscore가 실행 중이어야 함)
            ros::Time current_time = ros::Time::now();
            delta_time =  current_time.toSec() - init_time.toSec();  // x 값을 계산

            x = delta_time - 4;

            // 시그모이드 함수 계산
            if( sp_60_to_230 == true ){
                a0 = 2619.444444447213*pow(a, 6);
                a1 = -7320.833333339944*pow(a, 5);
                a2 = 8401.527777783953*pow(a, 4);
                a3 = -5102.29166666951*pow(a, 3);
                a4 = 1757.7727777784412*pow(a, 2);
                a5 = -340.15000000007126*a;
                a6 = 32.77000000000234;

                b = a0 + a1 + a2 + a3 + a4 + a5 + a6;

                sigmoid_value = a*( b / (1.0 + std::exp(-x))) + a;
            }
            else {
                sigmoid_value = 1.3 / (1.0 + std::exp(-x));
            }

            if(desired_speed < sigmoid_value){
                desired_speed = sigmoid_value;
            }

        }
        else if(speed_60 == true){
            // 현재 ROS 시간을 사용하여 x 값을 생성 (ROS 시간을 사용하려면 roscore가 실행 중이어야 함)
            ros::Time current_time = ros::Time::now();
            delta_time =  current_time.toSec() - init_time.toSec();  // x 값을 계산


            x = delta_time - 4;


            // 시그모이드 함수 계산
            sigmoid_value = 0.7 / (1.0 + std::exp(-x));

            desired_speed = sigmoid_value;
        }
        else if(speed_0 == true){
            // 현재 ROS 시간을 사용하여 x 값을 생성 (ROS 시간을 사용하려면 roscore가 실행 중이어야 함)
            ros::Time current_time = ros::Time::now();
            delta_time =  current_time.toSec() - init_time.toSec();  // x 값을 계산
            x = delta_time - 4;

            // 시그모이드 함수 계산
            sigmoid_value = 1.3 / (0.65 + 20*std::exp(x));
            desired_speed = sigmoid_value;

        }

        ros::spinOnce();
        //loop_rate.sleep();
    }
    init_start_230 = true;
    init_start_60 = true;
    init_start_0 = true;

    loop_rate.sleep();
    //ros::spin();

    return 0;
}

