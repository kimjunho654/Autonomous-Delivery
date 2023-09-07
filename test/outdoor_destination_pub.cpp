#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <iostream>
#include <string>

#include <sstream>

using namespace std;

void initializeGoal(std_msgs::String& goal_msg, string msg) {

    // 목표 위치 설정
    goal_msg.data = msg;

}

string target_string = ""; // 초기값을 빈 문자열로 설정

// std_msgs/String 토픽 콜백 함수
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    // 받은 문자열 메시지 출력 (ROS_INFO() 사용 시 한국어가 출력되지 않아서 cout을 사용)
    cout << "Received String : " << msg->data.c_str() << endl;
    target_string = msg->data; // 받은 메시지를 target_string에 저장
}

int main(int argc, char** argv) {
    // ROS 노드 초기화
    ros::init(argc, argv, "outdoor_destination_pub");

    ros::NodeHandle nh;

    // 발행자 생성
    ros::Publisher goal_pub = nh.advertise<std_msgs::String>("/destination", 10);

    // 구독자 생성
    ros::Subscriber chatter_sub = nh.subscribe("chatter", 10, chatterCallback);

    // 발행자 생성
    ros::Publisher sound_pub = nh.advertise<std_msgs::String>("sound_play", 10);

    // 발행 주기 설정 (여기서는 1Hz)
    ros::Rate rate(1.0);

    std_msgs::String goal_msg;

    int count = 0;

    while (ros::ok()) {

    std_msgs::String msg;

    std::stringstream ss;

    if(target_string.find("M101") != string::npos || target_string.find("m100일") != string::npos || target_string.find("m101") != string::npos) {
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M101";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M102") != string::npos || target_string.find("m100이") != string::npos || target_string.find("m102") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M102" << count;
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M103") != string::npos || target_string.find("m100삼") != string::npos || target_string.find("m103") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M103";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M104") != string::npos || target_string.find("m100사") != string::npos || target_string.find("m104") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M104";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M105") != string::npos || target_string.find("m100오") != string::npos || target_string.find("m105") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M105";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M106") != string::npos || target_string.find("m100육") != string::npos || target_string.find("m106") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M106";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M107") != string::npos || target_string.find("m100칠") != string::npos || target_string.find("m107") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M107";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }
        else if(target_string.find("M108") != string::npos || target_string.find("m100팔") != string::npos || target_string.find("m108") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M108";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M109") != string::npos || target_string.find("m100구") != string::npos || target_string.find("m109") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M109";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M110") != string::npos || target_string.find("m100십") != string::npos || target_string.find("m110") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M110";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M111") != string::npos || target_string.find("m100십일") != string::npos || target_string.find("m111") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M111";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M112") != string::npos || target_string.find("m100십이") != string::npos || target_string.find("m112") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M112";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M113") != string::npos || target_string.find("m100십삼") != string::npos || target_string.find("m113") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M113";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M114") != string::npos || target_string.find("m100십사") != string::npos || target_string.find("m114") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M114";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M115") != string::npos || target_string.find("m100십오") != string::npos || target_string.find("m115") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M115";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M116") != string::npos || target_string.find("m100십육") != string::npos || target_string.find("m116") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M116";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M117") != string::npos || target_string.find("m100십칠") != string::npos || target_string.find("m117") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M117";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M118") != string::npos || target_string.find("m100십팔") != string::npos || target_string.find("m118") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "M118";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());

      if (count == 0) sound_pub.publish(msg);
      count++;
        }
        else if(target_string.find("elevator") != string::npos || target_string.find("엘레베이터") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "elevator";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("toilet") != string::npos || target_string.find("화장실") != string::npos){
          initializeGoal(goal_msg, "M101");
          goal_pub.publish(goal_msg);
      ss << "toilet";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else {
          initializeGoal(goal_msg, 0, 0, 0, 0);
          //goal_pub.publish(goal_msg);
      count = 0;
        }

        // 로그 출력
        ROS_INFO("Goal published!");

        // ROS 콜백 처리
        ros::spinOnce();

        ros::Duration(1.0).sleep(); // 1초 동안 대기
    }

    return 0;
}

