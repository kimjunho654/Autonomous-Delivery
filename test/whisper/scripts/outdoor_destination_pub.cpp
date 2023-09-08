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

    if(target_string.find("유니토피아") != string::npos || target_string.find("유니토피야") != string::npos || target_string.find("유니토피하") != string::npos || target_string.find("유니토피안") != string::npos) {
          initializeGoal(goal_msg, "Unitophia");
          goal_pub.publish(goal_msg);
      ss << "Unitophia";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("멀티미디어") != string::npos || target_string.find("멀티미디아") != string::npos || target_string.find("머티미디어") != string::npos || target_string.find("머티미디아") != string::npos || target_string.find("멀티 미디어") != string::npos){
          initializeGoal(goal_msg, "Multi_Media_Gwan");
          goal_pub.publish(goal_msg);
      ss << "Multi_Media_Gwan" << count;
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("학예관") != string::npos || target_string.find("하계관") != string::npos || target_string.find("하계광") != string::npos || target_string.find("하게광") != string::npos){
          initializeGoal(goal_msg, "Hak_Ye_Gwan");
          goal_pub.publish(goal_msg);
      ss << "Hak_Ye_Gwan";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("브릭스") != string::npos || target_string.find("브릿스") != string::npos || target_string.find("브릭쓰") != string::npos || target_string.find("브릿쓰") != string::npos){
          initializeGoal(goal_msg, "BRIX_Gwan");
          goal_pub.publish(goal_msg);
      ss << "BRIX_Gwan";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("지역혁신관") != string::npos || target_string.find("지역혁신과") != string::npos || target_string.find("지역혁신광") != string::npos || target_string.find("지역형신관") != string::npos){
          initializeGoal(goal_msg, "regionalinnovationgwan");
          goal_pub.publish(goal_msg);
      ss << "regionalinnovationgwan";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("산학협력관") != string::npos || target_string.find("사낙협력관") != string::npos || target_string.find("사낙협력광") != string::npos || target_string.find("산학협렵관") != string::npos || target_string.find("사낙협렵관") != string::npos){
          initializeGoal(goal_msg, "San_Hak_Hyeop_Ryeok_Gwan");
          goal_pub.publish(goal_msg);
      ss << "San_Hak_Hyeop_Ryeok_Gwan";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("공학관") != string::npos || target_string.find("공항관") != string::npos || target_string.find("공하깐") != string::npos || target_string.find("공학깐") != string::npos){
          initializeGoal(goal_msg, "Gong_Hak_Gwan");
          goal_pub.publish(goal_msg);
      ss << "Gong_Hak_Gwan";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }
        else if(target_string.find("도서관") != string::npos || target_string.find("도서간") != string::npos || target_string.find("도서강") != string::npos || target_string.find("도서광") != string::npos){
          initializeGoal(goal_msg, "Library");
          goal_pub.publish(goal_msg);
      ss << "Library";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("앙뜨레프레너") != string::npos || target_string.find("앙뜨래프레너") != string::npos || target_string.find("앙뜨래프래너") != string::npos || target_string.find("앙뜨레프래너") != string::npos){
          initializeGoal(goal_msg, "Antire_preneur_Gwan");
          goal_pub.publish(goal_msg);
      ss << "Antire_preneur_Gwan";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("한마루") != string::npos || target_string.find("한마룩") != string::npos || target_string.find("한마록") != string::npos || target_string.find("한마로") != string::npos){
          initializeGoal(goal_msg, "hanmaru");
          goal_pub.publish(goal_msg);
      ss << "hanmaru";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("자연과학관") != string::npos || target_string.find("자연과학꽌") != string::npos || target_string.find("자연과학깐") != string::npos || target_string.find("자연가학관") != string::npos){
          initializeGoal(goal_msg, "Naturel_Science_Gwan");
          goal_pub.publish(goal_msg);
      ss << "Naturel_Science_Gwan";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("인문과학관") != string::npos || target_string.find("인문대") != string::npos || target_string.find("인문과학꽌") != string::npos || target_string.find("인문과학깐") != string::npos){
          initializeGoal(goal_msg, "Humanities_Social_Science_Gwan");
          goal_pub.publish(goal_msg);
      ss << "Humanities_Social_Science_Gwan";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("대학본부") != string::npos || target_string.find("대학봉부") != string::npos || target_string.find("대학본부") != string::npos){
          initializeGoal(goal_msg, "Main_University");
          goal_pub.publish(goal_msg);
      ss << "Main_University";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("미디어랩스") != string::npos || target_string.find("미디어랩쓰") != string::npos || target_string.find("미디어렙스") != string::npos || target_string.find("미디어렙쓰") != string::npos){
          initializeGoal(goal_msg, "medialaps");
          goal_pub.publish(goal_msg);
      ss << "medialaps";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("글로벌빌리지") != string::npos || target_string.find("글로벌빌리쥐") != string::npos || target_string.find("글로벌빌래지") != string::npos || target_string.find("글로벌빌레지") != string::npos){
          initializeGoal(goal_msg, "Global_Village");
          goal_pub.publish(goal_msg);
      ss << "Global_Village";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("향설삼") != string::npos || target_string.find("향설3") != string::npos || target_string.find("향설 삼") != string::npos || target_string.find("향설 3") != string::npos){
          initializeGoal(goal_msg, "Hyang_333");
          goal_pub.publish(goal_msg);
      ss << "Hyang_333";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else {
          initializeGoal(goal_msg, "");
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
