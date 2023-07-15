#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <iostream>

using namespace std;

void initializeGoal(move_base_msgs::MoveBaseActionGoal& goal_msg, double x, double y, double z, double w) {
    // 헤더 정보 설정
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = "map";

    // 목표 위치 설정
    goal_msg.goal.target_pose.header.stamp = ros::Time::now();
    goal_msg.goal.target_pose.header.frame_id = "map";
    goal_msg.goal.target_pose.pose.position.x = x;  // x 좌표
    goal_msg.goal.target_pose.pose.position.y = y;  // y 좌표
    goal_msg.goal.target_pose.pose.position.z = z;
    goal_msg.goal.target_pose.pose.orientation.x = 0.0;
    goal_msg.goal.target_pose.pose.orientation.y = 0.0;
    goal_msg.goal.target_pose.pose.orientation.z = 0.0;
    goal_msg.goal.target_pose.pose.orientation.w = w;
}


string target_string;

// std_msgs/String 토픽 콜백 함수
void stringCallback(const std_msgs::String::ConstPtr& msg) {
    // 받은 문자열 메시지 출력
    ROS_INFO("Received String: %s", msg->data.c_str());
    target_string = msg->data;
}

int main(int argc, char** argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "move_multimedia");
    ros::NodeHandle nh;

    // 발행자 생성
    ros::Publisher goal_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);

    // 구독자 생성
    ros::Subscriber string_sub = nh.subscribe("/move_target", 10, stringCallback);

    // 발행 주기 설정 (여기서는 1Hz)
    ros::Rate rate(1.0);

    while (ros::ok()) {
        // MoveBaseActionGoal 메시지 생성
        move_base_msgs::MoveBaseActionGoal goal_msg;

        // 목표 위치 초기화
        if(target_string == "M101") {  
           
          initializeGoal(goal_msg, 1.0, 2.0, 0.0, 1.0);  
          goal_pub.publish(goal_msg);
        }
        else {
          initializeGoal(goal_msg, 0.0, 0.0, 0.0, 0.0);  
          goal_pub.publish(goal_msg);
        }



        // 토픽에 메시지 발행


        // 로그 출력
        ROS_INFO("Goal published!");

        // 발행 주기 대기
        rate.sleep();

        // ROS 콜백 처리
        ros::spinOnce();
    }

    return 0;
}


