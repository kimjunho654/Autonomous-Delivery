#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <iostream>
#include <string>

#include <sstream>

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
    goal_msg.goal.target_pose.pose.position.z = 0.0;
    goal_msg.goal.target_pose.pose.orientation.x = 0.0;
    goal_msg.goal.target_pose.pose.orientation.y = 0.0;
    goal_msg.goal.target_pose.pose.orientation.z = z;
    goal_msg.goal.target_pose.pose.orientation.w = w;
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
    ros::init(argc, argv, "move_multimedia");
    ros::init(argc, argv, "speech");
    ros::NodeHandle nh;

    // 발행자 생성
    ros::Publisher goal_pub = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 10);

    // 구독자 생성
    ros::Subscriber chatter_sub = nh.subscribe("chatter", 10, chatterCallback);

    // 발행자 생성
    ros::Publisher sound_pub = nh.advertise<std_msgs::String>("sound_play", 10);

    // 발행 주기 설정 (여기서는 1Hz)
    ros::Rate rate(1.0);

    move_base_msgs::MoveBaseActionGoal goal_msg;

    int count = 0;

    while (ros::ok()) {

    std_msgs::String msg;

    std::stringstream ss;

    if(target_string.find("M101") != string::npos || target_string.find("m100일") != string::npos || target_string.find("m101") != string::npos) {
          initializeGoal(goal_msg, 39.7713661194, 10.74492836, 0.703293382062, 0.710899724819);
          goal_pub.publish(goal_msg);
      ss << "M101";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M102") != string::npos || target_string.find("m100이") != string::npos || target_string.find("m102") != string::npos){
          initializeGoal(goal_msg, 39.594581604, 1.08796322346, 0.707384616201, 0.706828836963);
          goal_pub.publish(goal_msg);
      ss << "M102" << count;
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M103") != string::npos || target_string.find("m100삼") != string::npos || target_string.find("m103") != string::npos){
          initializeGoal(goal_msg, 39.4695205688, -4.99704694748, 0.710767948012, 0.703426559122);
          goal_pub.publish(goal_msg);
      ss << "M103";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M104") != string::npos || target_string.find("m100사") != string::npos || target_string.find("m104") != string::npos){
          initializeGoal(goal_msg, 39.3844413757, -7.21077442169, 0.699160183599, 0.714965060454);
          goal_pub.publish(goal_msg);
      ss << "M104";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M105") != string::npos || target_string.find("m100오") != string::npos || target_string.find("m105") != string::npos){
          initializeGoal(goal_msg, 39.4179039001, -9.93680667877, 0.709021329049, 0.705187035441);
          goal_pub.publish(goal_msg);
      ss << "M105";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M106") != string::npos || target_string.find("m100육") != string::npos || target_string.find("m106") != string::npos){
          initializeGoal(goal_msg, 28.8886928558, -5.32010173798, -0.000719826606338, 0.999999740925);
          goal_pub.publish(goal_msg);
      ss << "M106";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M107") != string::npos || target_string.find("m100칠") != string::npos || target_string.find("m107") != string::npos){
          initializeGoal(goal_msg, 24.117105484, -5.25540542603, 0.00477599259845, 0.999988594882);
          goal_pub.publish(goal_msg);
      ss << "M107";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }
        else if(target_string.find("M108") != string::npos || target_string.find("m100팔") != string::npos || target_string.find("m108") != string::npos){
          initializeGoal(goal_msg, 21.7005691528, -5.20102500916, 0.00477601820953, 0.99998859476);
          goal_pub.publish(goal_msg);
      ss << "M108";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M109") != string::npos || target_string.find("m100구") != string::npos || target_string.find("m109") != string::npos){
          initializeGoal(goal_msg, 16.7889938354, -5.25853443146, 0.00644216747323, 0.999979249024);
          goal_pub.publish(goal_msg);
      ss << "M109";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M110") != string::npos || target_string.find("m100십") != string::npos || target_string.find("m110") != string::npos){
          initializeGoal(goal_msg, 14.669511795, -0.751161634922, 0.00746751574558, 0.999972117716);
          goal_pub.publish(goal_msg);
      ss << "M110";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M111") != string::npos || target_string.find("m100십일") != string::npos || target_string.find("m111") != string::npos){
          initializeGoal(goal_msg, 9.71412849426, -0.702483057976, -0.000224102723166, 0.999999974889);
          goal_pub.publish(goal_msg);
      ss << "M111";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M112") != string::npos || target_string.find("m100십이") != string::npos || target_string.find("m112") != string::npos){
          initializeGoal(goal_msg, 7.34471130371, -0.840794801712, -0.000224083631054, 0.999999974893);
          goal_pub.publish(goal_msg);
      ss << "M112";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M113") != string::npos || target_string.find("m100십삼") != string::npos || target_string.find("m113") != string::npos){
          initializeGoal(goal_msg, 1.96411705017, -0.937937855721, -0.00272415334564, 0.999996289487);
          goal_pub.publish(goal_msg);
      ss << "M113";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M114") != string::npos || target_string.find("m100십사") != string::npos || target_string.find("m114") != string::npos){
          initializeGoal(goal_msg, 0.137761116028, 0.965855538845, 0.000517469458471, 0.999999866113);
          goal_pub.publish(goal_msg);
      ss << "M114";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M115") != string::npos || target_string.find("m100십오") != string::npos || target_string.find("m115") != string::npos){
          initializeGoal(goal_msg, 2.41346216202, 1.02584314346, 0.00977547170733, 0.999952218935);
          goal_pub.publish(goal_msg);
      ss << "M115";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M116") != string::npos || target_string.find("m100십육") != string::npos || target_string.find("m116") != string::npos){
          initializeGoal(goal_msg, 7.70515537262, 1.10299348831, 0.00727569132504, 0.999973531808);
          goal_pub.publish(goal_msg);
      ss << "M116";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M117") != string::npos || target_string.find("m100십칠") != string::npos || target_string.find("m117") != string::npos){
          initializeGoal(goal_msg, 9.54574680328, 4.99851131439, 0.0000298423310, 0.999999999555);
          goal_pub.publish(goal_msg);
      ss << "M117";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("M118") != string::npos || target_string.find("m100십팔") != string::npos || target_string.find("m118") != string::npos){
          initializeGoal(goal_msg, 28.7269687653, 4.72274589539, -0.0227849079794, 0.999740390286);
          goal_pub.publish(goal_msg);
      ss << "M118";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());

      if (count == 0) sound_pub.publish(msg);
      count++;
        }
        else if(target_string.find("elevator") != string::npos || target_string.find("엘레베이터") != string::npos){
          initializeGoal(goal_msg, 36.0193634033, -5.46648740768, -0.00839813239766, 0.999964735064);
          goal_pub.publish(goal_msg);
      ss << "elevator";
      msg.data = ss.str();
      ROS_INFO("speech : %s", msg.data.c_str());
      if (count == 0) sound_pub.publish(msg);
      count++;
        }

        else if(target_string.find("toilet") != string::npos || target_string.find("화장실") != string::npos){
          initializeGoal(goal_msg, 33.814907074, -5.24557733536, -0.00241715342055, 0.99999707868);
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

