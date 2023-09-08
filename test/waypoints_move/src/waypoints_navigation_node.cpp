#include <math.h>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h" ///////////////////////////
#include "std_msgs/Bool.h" /////////////
#include <std_msgs/Float64MultiArray.h> /////////////////

#include <iostream>
#include <string>
#include <sstream>

using namespace std;

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)
#define WayPoints_NO 42
#define WayPoint_X_Tor 0.3
#define WayPoint_Y_Tor 0.5



int no_waypoints = WayPoints_NO;
double waypoint_line_angle = 0.0;
float imu_offset = -53;
double car_angle;
int set_delivery_id = 0; //////////////////////
bool received_product = false; ////////////
bool lidar_object_detect = false; ////////////
bool avoid_function_start = false; ////////////
double avoid_heading_angle_left = 0; //////////////
double avoid_heading_angle_right = 0; //////////////
int wp_go_id = 0;
int wp_finish_id = 0;
double roll,pitch,yaw;
bool start_command = false; ////////////
bool emergency_stop = false; ////////////
double depth; /////////////




geometry_msgs::Pose2D my_pose;
geometry_msgs::Pose2D my_target_pose_goal;
geometry_msgs::Pose2D my_target_pose_goal_prev;
geometry_msgs::PoseStamped utm_fix;

std_msgs::String destination_data;
std::string detect_name;
 


struct WayPoints
{
        double x;
        double y;
};

struct WayPoints my_waypoints_list[WayPoints_NO];





//GPS의경우 UTM 좌표를 따라서 XY가 다름

void utm_Callback(const geometry_msgs::PoseStamped::ConstPtr& msgs)
{
        utm_fix.header = msgs->header;
        utm_fix.pose.position.x     =  msgs->pose.position.x ;
        utm_fix.pose.position.y     =  msgs->pose.position.y ;

        my_pose.x     =  utm_fix.pose.position.x;      //UTM 좌표의 경우 X,Y 좌표가 90도 회전되어 있음
        my_pose.y     =  utm_fix.pose.position.y;      //UTM 좌표의 경우 X,Y 좌표가 90도 회전되어 있음
}

void imu_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
        tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

        tf2::Matrix3x3 m(q);

        m.getRPY(roll, pitch, yaw);

        yaw = yaw+ DEG2RAD(imu_offset);
        my_pose.theta = yaw;
        //printf("%6.3lf(rad)  %6.3lf \n",yaw, yaw*180/3.14159);
}

void init_waypoint(void)
{
        my_waypoints_list[0].x = 315719.89;
        my_waypoints_list[0].y = 4071242.41;

        my_waypoints_list[1].x = 315723.16;
        my_waypoints_list[1].y = 4071239.60;

        my_waypoints_list[2].x = 315727.92;
        my_waypoints_list[2].y = 4071232.56;

        my_waypoints_list[3].x = 315730.07;
        my_waypoints_list[3].y = 4071225.90;

        my_waypoints_list[4].x = 315723.32;
        my_waypoints_list[4].y = 4071219.55;

        my_waypoints_list[5].x = 315719.06;
        my_waypoints_list[5].y = 4071217.23;

        my_waypoints_list[6].x = 315718.17;
        my_waypoints_list[6].y = 4071215.08;  // son nim

        my_waypoints_list[7].x = 315720.44;
        my_waypoints_list[7].y = 4071215.01;

        my_waypoints_list[8].x = 315724.41;
        my_waypoints_list[8].y = 4071216.97;

        my_waypoints_list[9].x = 315729.18;
        my_waypoints_list[9].y = 4071220.21;

        my_waypoints_list[10].x = 315730.79;
        my_waypoints_list[10].y = 4071227.73;

        my_waypoints_list[11].x = 315727.98;
        my_waypoints_list[11].y = 4071232.84;

        my_waypoints_list[12].x = 315721.12;
        my_waypoints_list[12].y = 4071244.21;

        set_delivery_id = 6;
        no_waypoints = 13;
        wp_finish_id = 12;
}

void Unitophia(void)
{
        my_waypoints_list[0].x = 315439.887872;
        my_waypoints_list[0].y = 4017316.728305;

        my_waypoints_list[1].x = 315458.201930;
        my_waypoints_list[1].y = 4071252.597692;

        my_waypoints_list[2].x = 315502.84127;
        my_waypoints_list[2].y = 4071269.81511;

        my_waypoints_list[3].x = 315504.004636;
        my_waypoints_list[3].y = 4071274.474232;

        my_waypoints_list[4].x = 315503.395912;
        my_waypoints_list[4].y = 4071273.703714;

        my_waypoints_list[5].x = 315503.395912;
        my_waypoints_list[5].y = 4071273.703714;

        my_waypoints_list[6].x = 315476.865139;
        my_waypoints_list[6].y = 4071317.429893;

        my_waypoints_list[7].x = 315479.466137;
        my_waypoints_list[7].y = 4071324.933992;

        my_waypoints_list[8].x = 315479.750606;
        my_waypoints_list[8].y = 4071322.807718;

        my_waypoints_list[9].x = 315481.969678;
        my_waypoints_list[9].y = 4071329.664052;

        my_waypoints_list[10].x = 315482.972224;
        my_waypoints_list[10].y = 4071331.013255;

        my_waypoints_list[11].x = 315496.080166;
        my_waypoints_list[11].y = 4071317.136784;

        my_waypoints_list[12].x = 315517.165063;
        my_waypoints_list[12].y = 4071300.511648;

        my_waypoints_list[13].x = 315520.395969;
        my_waypoints_list[13].y = 4071297.905463;

        my_waypoints_list[14].x = 315532.530903;
        my_waypoints_list[14].y = 4071300.783531;

        my_waypoints_list[15].x = 315572.913426;
        my_waypoints_list[15].y = 4071313.854113;

        my_waypoints_list[16].x = 315576.133565;
        my_waypoints_list[16].y = 4071312.361526;

        my_waypoints_list[17].x = 315580.085779;
        my_waypoints_list[17].y = 4071310.239292;

        my_waypoints_list[18].x = 315582.94788;
        my_waypoints_list[18].y = 4071307.012831;

        my_waypoints_list[19].x = 315606.159708;
        my_waypoints_list[19].y = 4071303.890809;

        my_waypoints_list[20].x = 315584.108858;
        my_waypoints_list[20].y = 4071320.246864;

        my_waypoints_list[21].x = 315604.634973;
        my_waypoints_list[21].y = 4071301.641548;

        my_waypoints_list[22].x = 315621.971242;
        my_waypoints_list[22].y = 4071262.85076;

        my_waypoints_list[23].x = 315624.4353775;
        my_waypoints_list[23].y = 4071259.741134;

        my_waypoints_list[24].x = 315626.899513;
        my_waypoints_list[24].y = 4071256.631508;

        my_waypoints_list[25].x = 315680.609426;
        my_waypoints_list[25].y = 4071278.219467;

        my_waypoints_list[26].x = 315680.455721;
        my_waypoints_list[26].y = 4071271.098725;

        my_waypoints_list[27].x = 315690.573353;
        my_waypoints_list[27].y = 4071270.256509;

        my_waypoints_list[28].x = 315716.951680;
        my_waypoints_list[28].y = 4071269.951680;

        my_waypoints_list[29].x = 315713.744796;
        my_waypoints_list[29].y = 4071257.768267;

        set_delivery_id = 29;
        no_waypoints = 30;
        wp_finish_id = 29;
}

void Multi_Media_Gwan(void)
{
        my_waypoints_list[0].x = 315435.697968;
        my_waypoints_list[0].y = 4071228.980568;

        my_waypoints_list[1].x = 315459.264668;
        my_waypoints_list[1].y = 4071243.866586;

        my_waypoints_list[2].x = 315501.151300;
        my_waypoints_list[2].y = 4071267.871898;

        my_waypoints_list[3].x = 315502.573051;
        my_waypoints_list[3].y = 4071270.092712;

        my_waypoints_list[4].x = 315502.627913;
        my_waypoints_list[4].y = 4071272.717188;

        my_waypoints_list[5].x = 315501.813174;
        my_waypoints_list[5].y = 4071275.609902;

        my_waypoints_list[6].x = 315488.512653;
        my_waypoints_list[6].y = 4071297.268012;

        my_waypoints_list[7].x = 315477.446009;
        my_waypoints_list[7].y = 4071318.129249;

        my_waypoints_list[8].x = 315476.912571;
        my_waypoints_list[8].y = 4071322.516440;

        my_waypoints_list[9].x = 315478.737985;
        my_waypoints_list[9].y = 4071326.104142;

        my_waypoints_list[10].x = 315483.127787;
        my_waypoints_list[10].y = 4071326.762554;
////////////////// 15 -> , 36 -> 32 ////////////////////////////
        my_waypoints_list[11].x = 315490.235691;
        my_waypoints_list[11].y = 4071325.863791;

        my_waypoints_list[12].x = 315494.916819;
        my_waypoints_list[12].y = 4071322.515163;

        my_waypoints_list[13].x = 315509.459676;
        my_waypoints_list[13].y = 4071306.457417;

        my_waypoints_list[14].x = 315514.372466;
        my_waypoints_list[14].y = 4071302.228741;

        my_waypoints_list[15].x = 315519.217755;
        my_waypoints_list[15].y = 4071300.752128;

        my_waypoints_list[16].x = 315524.586457;
        my_waypoints_list[16].y = 4071300.389840;

        my_waypoints_list[17].x = 315529.353797;
        my_waypoints_list[17].y = 4071301.165391;

        my_waypoints_list[18].x = 315534.298361;
        my_waypoints_list[18].y = 4071304.437829;

        my_waypoints_list[19].x = 315545.921465;
        my_waypoints_list[19].y = 4071310.196281;

        my_waypoints_list[20].x = 315560.942407;
        my_waypoints_list[20].y = 4071317.008971;

        my_waypoints_list[21].x = 315566.352908;
        my_waypoints_list[21].y = 4071318.646283;

        my_waypoints_list[22].x = 315565.597833;
        my_waypoints_list[22].y = 4071318.412008;

        my_waypoints_list[23].x = 315569.617934;
        my_waypoints_list[23].y = 4071319.328208;

        my_waypoints_list[24].x = 315575.252260;
        my_waypoints_list[24].y = 4071319.710546;

        my_waypoints_list[25].x = 315579.251461;
        my_waypoints_list[25].y = 4071319.626945;

        my_waypoints_list[26].x = 315582.615336;
        my_waypoints_list[26].y = 4071319.056508;

        my_waypoints_list[27].x = 315585.599061;
        my_waypoints_list[27].y = 4071318.243957;

        my_waypoints_list[28].x = 315589.434099;
        my_waypoints_list[28].y = 4071316.288344;

        my_waypoints_list[29].x = 315593.508637;
        my_waypoints_list[29].y = 4071313.827607;

        my_waypoints_list[30].x = 315597.075438;
        my_waypoints_list[30].y = 4071311.002394;

        my_waypoints_list[31].x = 315599.876713;
        my_waypoints_list[31].y = 4071307.443006;

        my_waypoints_list[32].x = 315602.654476;
        my_waypoints_list[32].y = 4071302.758843;  // 36
////////////////// 42 -> 33, 61 ->  ////////////////////////////

        my_waypoints_list[33].x = 315607.587739;
        my_waypoints_list[33].y = 4071293.528555;

        my_waypoints_list[34].x = 315626.399368;
        my_waypoints_list[34].y = 4071260.377558;

        my_waypoints_list[35].x = 315628.864906;
        my_waypoints_list[35].y = 4071258.700634;

        my_waypoints_list[36].x = 315631.827731;
        my_waypoints_list[36].y = 4071256.888284;

        my_waypoints_list[37].x = 315639.612755;
        my_waypoints_list[37].y = 4071258.475961;

        my_waypoints_list[38].x = 315658.247938;
        my_waypoints_list[38].y = 4071270.714400;

        my_waypoints_list[39].x = 315670.516809;
        my_waypoints_list[39].y = 4071277.459588;

        my_waypoints_list[40].x = 315678.093682;
        my_waypoints_list[40].y = 4071281.052089;  // 49

        my_waypoints_list[41].x = 315680.345844;
        my_waypoints_list[41].y = 4071281.130039;

        my_waypoints_list[42].x = 315683.069168;
        my_waypoints_list[42].y = 4071279.822814;

        my_waypoints_list[43].x = 315686.490092;
        my_waypoints_list[43].y = 4071276.000416;

        my_waypoints_list[44].x = 315689.804329;
        my_waypoints_list[44].y = 4071273.055455;

        my_waypoints_list[45].x = 315693.618466;
        my_waypoints_list[45].y = 4071270.100044;

        my_waypoints_list[46].x = 315697.81510;
        my_waypoints_list[46].y = 4071267.511726;

        my_waypoints_list[47].x = 315700.897939;
        my_waypoints_list[47].y = 4071265.446810;

        my_waypoints_list[48].x = 315704.347601;
        my_waypoints_list[48].y = 4071262.999136;

        my_waypoints_list[49].x = 315707.669675;
        my_waypoints_list[49].y = 4071260.429100;

        my_waypoints_list[50].x = 315712.043375;
        my_waypoints_list[50].y = 4071254.336254;

        my_waypoints_list[51].x = 315711.327064;
        my_waypoints_list[51].y = 4071249.975193;

        my_waypoints_list[52].x = 315713.053201;
        my_waypoints_list[52].y = 4071248.813844;  // 61

        set_delivery_id = 52;
        no_waypoints = 53;
        wp_finish_id = 52;
}

void Hak_Ye_Gwan(void)
{
        my_waypoints_list[0].x = 315439.887872;
        my_waypoints_list[0].y = 4017316.728305;

        my_waypoints_list[1].x = 315458.201930;
        my_waypoints_list[1].y = 4071252.597692;

        my_waypoints_list[2].x = 315502.84127;
        my_waypoints_list[2].y = 4071269.81511;

        my_waypoints_list[3].x = 315504.004636;
        my_waypoints_list[3].y = 4071274.474232;

        my_waypoints_list[4].x = 315503.395912;
        my_waypoints_list[4].y = 4071273.703714;

        my_waypoints_list[5].x = 315503.395912;
        my_waypoints_list[5].y = 4071273.703714;

        my_waypoints_list[6].x = 315476.865139;
        my_waypoints_list[6].y = 4071317.429893;

        my_waypoints_list[7].x = 315479.466137;
        my_waypoints_list[7].y = 4071324.933992;

        my_waypoints_list[8].x = 315479.750606;
        my_waypoints_list[8].y = 4071322.807718;

        my_waypoints_list[9].x = 315481.969678;
        my_waypoints_list[9].y = 4071329.664052;

        my_waypoints_list[10].x = 315482.972224;
        my_waypoints_list[10].y = 4071331.013255;

        my_waypoints_list[11].x = 315496.080166;
        my_waypoints_list[11].y = 4071317.136784;

        my_waypoints_list[12].x = 315517.165063;
        my_waypoints_list[12].y = 4071300.511648;

        my_waypoints_list[13].x = 315520.395969;
        my_waypoints_list[13].y = 4071297.905463;

        my_waypoints_list[14].x = 315532.530903;
        my_waypoints_list[14].y = 4071300.783531;

        my_waypoints_list[15].x = 315572.913426;
        my_waypoints_list[15].y = 4071313.854113;

        my_waypoints_list[16].x = 315576.133565;
        my_waypoints_list[16].y = 4071312.361526;

        my_waypoints_list[17].x = 315580.085779;
        my_waypoints_list[17].y = 4071310.239292;

        my_waypoints_list[18].x = 315582.94788;
        my_waypoints_list[18].y = 4071307.012831;

        my_waypoints_list[19].x = 315606.159708;
        my_waypoints_list[19].y = 4071303.890809;

        my_waypoints_list[20].x = 315584.108858;
        my_waypoints_list[20].y = 4071320.246864;

        my_waypoints_list[21].x = 315604.634973;
        my_waypoints_list[21].y = 4071301.641548;

        my_waypoints_list[22].x = 315621.971242;
        my_waypoints_list[22].y = 4071262.85076;

        my_waypoints_list[23].x = 315624.4353775;
        my_waypoints_list[23].y = 4071259.741134;

        my_waypoints_list[24].x = 315626.899513;
        my_waypoints_list[24].y = 4071256.631508;

        my_waypoints_list[25].x = 315680.609426;
        my_waypoints_list[25].y = 4071278.219467;

        my_waypoints_list[26].x = 315680.455721;
        my_waypoints_list[26].y = 4071271.098725;
/////////////// 31->27,  42->38 ///////////////////////
        my_waypoints_list[27].x = 315692.039925;
        my_waypoints_list[27].y = 4071283.589401;

        my_waypoints_list[28].x = 315684.329337;
        my_waypoints_list[28].y = 4071286.308861;

        my_waypoints_list[29].x = 315677.949829;
        my_waypoints_list[29].y = 4071292.773638;

        my_waypoints_list[30].x = 315677.992938;
        my_waypoints_list[30].y = 4071294.77073;

        my_waypoints_list[31].x = 315679.036425;
        my_waypoints_list[31].y = 4071301.741283;

        my_waypoints_list[32].x = 315676.715655;
        my_waypoints_list[32].y = 4071301.79138;

        my_waypoints_list[33].x = 315671.282103;
        my_waypoints_list[33].y = 4071306.570724;

        my_waypoints_list[34].x = 315663.823673;
        my_waypoints_list[34].y = 4071316.832846;

        my_waypoints_list[35].x = 315656.197448;
        my_waypoints_list[35].y = 4071335.867699;

        my_waypoints_list[36].x = 315649.478991;
        my_waypoints_list[36].y = 4071334.902733;

        my_waypoints_list[37].x = 315645.105248;
        my_waypoints_list[37].y = 4071334.997162;

        my_waypoints_list[38].x = 315659.253361;
        my_waypoints_list[38].y = 4071324.368595;

        set_delivery_id = 38;
        no_waypoints = 39;
        wp_finish_id = 38;
}

void BRIX_Gwan(void)
{
        my_waypoints_list[0].x = 315439.887872;
        my_waypoints_list[0].y = 4017316.728305;

        my_waypoints_list[1].x = 315458.201930;
        my_waypoints_list[1].y = 4071252.597692;

        my_waypoints_list[2].x = 315502.84127;
        my_waypoints_list[2].y = 4071269.81511;

        my_waypoints_list[3].x = 315504.004636;
        my_waypoints_list[3].y = 4071274.474232;

        my_waypoints_list[4].x = 315503.395912;
        my_waypoints_list[4].y = 4071273.703714;

        my_waypoints_list[5].x = 315503.395912;
        my_waypoints_list[5].y = 4071273.703714;

        my_waypoints_list[6].x = 315476.865139;
        my_waypoints_list[6].y = 4071317.429893;

        my_waypoints_list[7].x = 315479.466137;
        my_waypoints_list[7].y = 4071324.933992;

        my_waypoints_list[8].x = 315479.750606;
        my_waypoints_list[8].y = 4071322.807718;

        my_waypoints_list[9].x = 315481.969678;
        my_waypoints_list[9].y = 4071329.664052;

        my_waypoints_list[10].x = 315482.972224;
        my_waypoints_list[10].y = 4071331.013255;

        my_waypoints_list[11].x = 315496.080166;
        my_waypoints_list[11].y = 4071317.136784;

        my_waypoints_list[12].x = 315517.165063;
        my_waypoints_list[12].y = 4071300.511648;

        my_waypoints_list[13].x = 315520.395969;
        my_waypoints_list[13].y = 4071297.905463;

        my_waypoints_list[14].x = 315532.530903;
        my_waypoints_list[14].y = 4071300.783531;

        my_waypoints_list[15].x = 315572.913426;
        my_waypoints_list[15].y = 4071313.854113;

        my_waypoints_list[16].x = 315576.133565;
        my_waypoints_list[16].y = 4071312.361526;

        my_waypoints_list[17].x = 315580.085779;
        my_waypoints_list[17].y = 4071310.239292;

        my_waypoints_list[18].x = 315582.94788;
        my_waypoints_list[18].y = 4071307.012831;

        my_waypoints_list[19].x = 315606.159708;
        my_waypoints_list[19].y = 4071303.890809;

        my_waypoints_list[20].x = 315584.108858;
        my_waypoints_list[20].y = 4071320.246864;

        my_waypoints_list[21].x = 315604.634973;
        my_waypoints_list[21].y = 4071301.641548;

        my_waypoints_list[22].x = 315621.971242;
        my_waypoints_list[22].y = 4071262.85076;

        my_waypoints_list[23].x = 315624.4353775;
        my_waypoints_list[23].y = 4071259.741134;

        my_waypoints_list[24].x = 315626.899513;
        my_waypoints_list[24].y = 4071256.631508;

        my_waypoints_list[25].x = 315680.609426;
        my_waypoints_list[25].y = 4071278.219467;

        my_waypoints_list[26].x = 315680.455721;
        my_waypoints_list[26].y = 4071271.098725;
/////////////// 31->27,  40->36 ///////////////////////
        my_waypoints_list[27].x = 315692.039925;
        my_waypoints_list[27].y = 4071283.589401;

        my_waypoints_list[28].x = 315684.329337;
        my_waypoints_list[28].y = 4071286.308861;

        my_waypoints_list[29].x = 315677.949829;
        my_waypoints_list[29].y = 4071292.773638;

        my_waypoints_list[30].x = 315677.992938;
        my_waypoints_list[30].y = 4071294.77073;

        my_waypoints_list[31].x = 315679.036425;
        my_waypoints_list[31].y = 4071301.741283;

        my_waypoints_list[32].x = 315676.715655;
        my_waypoints_list[32].y = 4071301.79138;

        my_waypoints_list[33].x = 315671.282103;
        my_waypoints_list[33].y = 4071306.570724;

        my_waypoints_list[34].x = 315663.823673;
        my_waypoints_list[34].y = 4071316.832846;

        my_waypoints_list[35].x = 315656.197448;
        my_waypoints_list[35].y = 4071335.867699;

        my_waypoints_list[36].x = 315649.478991;
        my_waypoints_list[36].y = 4071334.902733;
/////////////// 43->37,  44->38 ///////////////////////
        my_waypoints_list[37].x = 315653.551419;
        my_waypoints_list[37].y = 4071333.260794;

        my_waypoints_list[38].x = 315641.241744;
        my_waypoints_list[38].y = 4071358.723849;

        set_delivery_id = 38;
        no_waypoints = 39;
        wp_finish_id = 38;
}

void San_Hak_Hyeop_Ryeok_Gwan(void)
{
        my_waypoints_list[0].x = 315439.887872;
        my_waypoints_list[0].y = 4017316.728305;

        my_waypoints_list[1].x = 315458.201930;
        my_waypoints_list[1].y = 4071252.597692;

        my_waypoints_list[2].x = 315502.84127;
        my_waypoints_list[2].y = 4071269.81511;

        my_waypoints_list[3].x = 315504.004636;
        my_waypoints_list[3].y = 4071274.474232;

        my_waypoints_list[4].x = 315503.395912;
        my_waypoints_list[4].y = 4071273.703714;

        my_waypoints_list[5].x = 315503.395912;
        my_waypoints_list[5].y = 4071273.703714;

        my_waypoints_list[6].x = 315476.865139;
        my_waypoints_list[6].y = 4071317.429893;

        my_waypoints_list[7].x = 315479.466137;
        my_waypoints_list[7].y = 4071324.933992;

        my_waypoints_list[8].x = 315479.750606;
        my_waypoints_list[8].y = 4071322.807718;

        my_waypoints_list[9].x = 315481.969678;
        my_waypoints_list[9].y = 4071329.664052;

        my_waypoints_list[10].x = 315482.972224;
        my_waypoints_list[10].y = 4071331.013255;
/////////////// 45->11,  47->13 ///////////////////////
        my_waypoints_list[11].x = 315504.947753;
        my_waypoints_list[11].y = 4071337.136333;

        my_waypoints_list[12].x = 315510.733444;
        my_waypoints_list[12].y = 4071332.127265;

        my_waypoints_list[13].x = 315509.790502;
        my_waypoints_list[13].y = 4071329.816609;

        set_delivery_id = 13;
        no_waypoints = 14;
        wp_finish_id = 13;

}

void Gong_Hak_Gwan(void)
{
        my_waypoints_list[0].x = 315435.697968;
        my_waypoints_list[0].y = 4071228.980568;

        my_waypoints_list[1].x = 315459.264668;
        my_waypoints_list[1].y = 4071243.866586;

        my_waypoints_list[2].x = 315501.151300;
        my_waypoints_list[2].y = 4071267.871898;

        my_waypoints_list[3].x = 315502.573051;
        my_waypoints_list[3].y = 4071270.092712;

        my_waypoints_list[4].x = 315502.627913;
        my_waypoints_list[4].y = 4071272.717188;

        my_waypoints_list[5].x = 315501.813174;
        my_waypoints_list[5].y = 4071275.609902;

        my_waypoints_list[6].x = 315488.512653;
        my_waypoints_list[6].y = 4071297.268012;

        my_waypoints_list[7].x = 315477.446009;
        my_waypoints_list[7].y = 4071318.129249;
////////////////// 115 -> 8, 122 -> 15 ////////////////////////////
        my_waypoints_list[8].x = 315472.283268; 
        my_waypoints_list[8].y = 4071322.363152;  // 115

        my_waypoints_list[9].x = 315469.932254; 
        my_waypoints_list[9].y = 4071323.537566;

        my_waypoints_list[10].x = 315466.466914; 
        my_waypoints_list[10].y = 4071325.235392;

        my_waypoints_list[11].x = 315462.840025; 
        my_waypoints_list[11].y = 4071325.186180;

        my_waypoints_list[12].x = 315459.179172; 
        my_waypoints_list[12].y = 4071323.512291;

        my_waypoints_list[13].x = 315451.690691; 
        my_waypoints_list[13].y = 4071318.167525;

        my_waypoints_list[14].x = 315442.678997; 
        my_waypoints_list[14].y = 4071311.729333;

        my_waypoints_list[15].x = 315432.274289; 
        my_waypoints_list[15].y = 4071304.445053;  // 122

        set_delivery_id = 15;
        no_waypoints = 16;
        wp_finish_id = 15;
}

void Library(void)
{
        my_waypoints_list[0].x = 315439.887872;
        my_waypoints_list[0].y = 4017316.728305;

        my_waypoints_list[1].x = 315458.201930;
        my_waypoints_list[1].y = 4071252.597692;

        my_waypoints_list[2].x = 315502.84127;
        my_waypoints_list[2].y = 4071269.81511;

        my_waypoints_list[3].x = 315504.004636;
        my_waypoints_list[3].y = 4071274.474232;

        my_waypoints_list[4].x = 315503.395912;
        my_waypoints_list[4].y = 4071273.703714;

        my_waypoints_list[5].x = 315503.395912;
        my_waypoints_list[5].y = 4071273.703714;

        my_waypoints_list[6].x = 315476.865139;
        my_waypoints_list[6].y = 4071317.429893;
/////////////// 48->7,  59->18 ///////////////////////
        my_waypoints_list[7].x = 315474.399013;
        my_waypoints_list[7].y = 4071320.258221;

        my_waypoints_list[8].x = 315467.684765;
        my_waypoints_list[8].y = 4071323.62236;

        my_waypoints_list[9].x = 315465.390374;
        my_waypoints_list[9].y = 4071324.89296;

        my_waypoints_list[10].x = 315389.545466;
        my_waypoints_list[10].y = 4071269.921647;

        my_waypoints_list[11].x = 315385.193781;
        my_waypoints_list[11].y = 4071266.907689;

        my_waypoints_list[12].x = 315370.986907;
        my_waypoints_list[12].y = 4071266.548847;

        my_waypoints_list[13].x = 315352.184048;
        my_waypoints_list[13].y = 4071268.398439;

        my_waypoints_list[14].x = 315344.637662;
        my_waypoints_list[14].y = 4071270.448651;

        my_waypoints_list[15].x = 315331.613737;
        my_waypoints_list[15].y = 4071262.849195;

        my_waypoints_list[16].x = 315324.247793;
        my_waypoints_list[16].y = 4071269.113581;

        my_waypoints_list[17].x = 315318.015854;
        my_waypoints_list[17].y = 4071274.13243;

        my_waypoints_list[18].x = 315297.922494;
        my_waypoints_list[18].y = 4071253.476743;

        set_delivery_id = 18;
        no_waypoints = 19;
        wp_finish_id = 18;
}

void Antire_preneur_Gwan(void)
{
        my_waypoints_list[0].x = 315435.697968;
        my_waypoints_list[0].y = 4071228.980568;

        my_waypoints_list[1].x = 315459.264668;
        my_waypoints_list[1].y = 4071243.866586;

        my_waypoints_list[2].x = 315501.151300;
        my_waypoints_list[2].y = 4071267.871898;

        my_waypoints_list[3].x = 315502.573051;
        my_waypoints_list[3].y = 4071270.092712;

        my_waypoints_list[4].x = 315502.627913;
        my_waypoints_list[4].y = 4071272.717188;

        my_waypoints_list[5].x = 315501.813174;
        my_waypoints_list[5].y = 4071275.609902;

        my_waypoints_list[6].x = 315488.512653;
        my_waypoints_list[6].y = 4071297.268012;

        my_waypoints_list[7].x = 315477.446009;
        my_waypoints_list[7].y = 4071318.129249;

        my_waypoints_list[8].x = 315476.912571;
        my_waypoints_list[8].y = 4071322.516440;

        my_waypoints_list[9].x = 315478.737985;
        my_waypoints_list[9].y = 4071326.104142;

        my_waypoints_list[10].x = 315483.127787;
        my_waypoints_list[10].y = 4071326.762554;
////////////////// 15 -> 11, 36 -> 32 ////////////////////////////
        my_waypoints_list[11].x = 315490.235691;
        my_waypoints_list[11].y = 4071325.863791;

        my_waypoints_list[12].x = 315494.916819;
        my_waypoints_list[12].y = 4071322.515163;

        my_waypoints_list[13].x = 315509.459676;
        my_waypoints_list[13].y = 4071306.457417;

        my_waypoints_list[14].x = 315514.372466;
        my_waypoints_list[14].y = 4071302.228741;

        my_waypoints_list[15].x = 315519.217755;
        my_waypoints_list[15].y = 4071300.752128;

        my_waypoints_list[16].x = 315524.586457;
        my_waypoints_list[16].y = 4071300.389840;

        my_waypoints_list[17].x = 315529.353797;
        my_waypoints_list[17].y = 4071301.165391;

        my_waypoints_list[18].x = 315534.298361;
        my_waypoints_list[18].y = 4071304.437829;

        my_waypoints_list[19].x = 315545.921465;
        my_waypoints_list[19].y = 4071310.196281;

        my_waypoints_list[20].x = 315560.942407;
        my_waypoints_list[20].y = 4071317.008971;

        my_waypoints_list[21].x = 315566.352908;
        my_waypoints_list[21].y = 4071318.646283;

        my_waypoints_list[22].x = 315565.597833;
        my_waypoints_list[22].y = 4071318.412008;

        my_waypoints_list[23].x = 315569.617934;
        my_waypoints_list[23].y = 4071319.328208;

        my_waypoints_list[24].x = 315575.252260;
        my_waypoints_list[24].y = 4071319.710546;

        my_waypoints_list[25].x = 315579.251461;
        my_waypoints_list[25].y = 4071319.626945;

        my_waypoints_list[26].x = 315582.615336;
        my_waypoints_list[26].y = 4071319.056508;

        my_waypoints_list[27].x = 315585.599061;
        my_waypoints_list[27].y = 4071318.243957;

        my_waypoints_list[28].x = 315589.434099;
        my_waypoints_list[28].y = 4071316.288344;

        my_waypoints_list[29].x = 315593.508637;
        my_waypoints_list[29].y = 4071313.827607;

        my_waypoints_list[30].x = 315597.075438;
        my_waypoints_list[30].y = 4071311.002394;

        my_waypoints_list[31].x = 315599.876713;
        my_waypoints_list[31].y = 4071307.443006;

        my_waypoints_list[32].x = 315602.654476;
        my_waypoints_list[32].y = 4071302.758843;  // 36
////////////////// 42 -> 33, 43 -> 34 ////////////////////////////

        my_waypoints_list[33].x = 315607.587739;
        my_waypoints_list[33].y = 4071293.528555;

        my_waypoints_list[34].x = 315626.399368;
        my_waypoints_list[34].y = 4071260.377558;  // 43
////////////////// 73 -> 35, 78 -> 40 ////////////////////////////
        my_waypoints_list[35].x = 315632.800983; 
        my_waypoints_list[35].y = 4071249.616223;

        my_waypoints_list[36].x = 315640.142311; 
        my_waypoints_list[36].y = 4071235.959564;

        my_waypoints_list[37].x = 315647.140065; 
        my_waypoints_list[37].y = 4071223.810442;  // 75

        my_waypoints_list[38].x = 315647.462741; 
        my_waypoints_list[38].y = 4071221.303105;

        my_waypoints_list[39].x = 315646.171192; 
        my_waypoints_list[39].y = 4071219.329630;

        my_waypoints_list[40].x = 315642.757680; 
        my_waypoints_list[40].y = 4071217.525542;

        set_delivery_id = 40;
        no_waypoints = 41;
        wp_finish_id = 40;


}

void Han_maru(void)
{

}


void Medical_Science_Gwan(void)
{

}

void Naturel_Science_Gwan(void)
{
        my_waypoints_list[0].x = 315435.697968;
        my_waypoints_list[0].y = 4071228.980568;

        my_waypoints_list[1].x = 315459.264668;
        my_waypoints_list[1].y = 4071243.866586;

        my_waypoints_list[2].x = 315501.151300;
        my_waypoints_list[2].y = 4071267.871898;

        my_waypoints_list[3].x = 315502.573051;
        my_waypoints_list[3].y = 4071270.092712;

        my_waypoints_list[4].x = 315502.627913;
        my_waypoints_list[4].y = 4071272.717188;

        my_waypoints_list[5].x = 315501.813174;
        my_waypoints_list[5].y = 4071275.609902;

        my_waypoints_list[6].x = 315488.512653;
        my_waypoints_list[6].y = 4071297.268012;

        my_waypoints_list[7].x = 315477.446009;
        my_waypoints_list[7].y = 4071318.129249;
////////////////// 115 -> 8, 156 -> 49 ////////////////////////////
        my_waypoints_list[8].x = 315472.283268; 
        my_waypoints_list[8].y = 4071322.363152;  // 115

        my_waypoints_list[9].x = 315469.932254; 
        my_waypoints_list[9].y = 4071323.537566;

        my_waypoints_list[10].x = 315466.466914; 
        my_waypoints_list[10].y = 4071325.235392;

        my_waypoints_list[11].x = 315462.840025; 
        my_waypoints_list[11].y = 4071325.186180;

        my_waypoints_list[12].x = 315459.179172; 
        my_waypoints_list[12].y = 4071323.512291;

        my_waypoints_list[13].x = 315451.690691; 
        my_waypoints_list[13].y = 4071318.167525;

        my_waypoints_list[14].x = 315442.678997; 
        my_waypoints_list[14].y = 4071311.729333;

        my_waypoints_list[15].x = 315432.274289; 
        my_waypoints_list[15].y = 4071304.445053;

        my_waypoints_list[16].x = 315426.561584; 
        my_waypoints_list[16].y = 4071300.313462;

        my_waypoints_list[17].x = 315420.468729; 
        my_waypoints_list[17].y = 4071295.939758;

        my_waypoints_list[18].x = 315420.213554; 
        my_waypoints_list[18].y = 4071295.695032;

        my_waypoints_list[19].x = 315410.701959; 
        my_waypoints_list[19].y = 4071289.267288;

        my_waypoints_list[20].x = 315401.807401; 
        my_waypoints_list[20].y = 4071282.451556;

        my_waypoints_list[21].x = 315393.811181; 
        my_waypoints_list[21].y = 4071276.742312;

        my_waypoints_list[22].x = 315386.960638; 
        my_waypoints_list[22].y = 4071272.009357;  // 129

        my_waypoints_list[23].x = 315382.539484; 
        my_waypoints_list[23].y = 4071269.851241;

        my_waypoints_list[24].x = 315378.904755; 
        my_waypoints_list[24].y = 4071269.427103;

        my_waypoints_list[25].x = 315374.405650; 
        my_waypoints_list[25].y = 4071269.521152;

        my_waypoints_list[26].x = 315365.662616; 
        my_waypoints_list[26].y = 4071269.953976;

        my_waypoints_list[27].x = 315353.545252; 
        my_waypoints_list[27].y = 4071270.457337;

        my_waypoints_list[28].x = 315342.805227; 
        my_waypoints_list[28].y = 4071271.056936;

        my_waypoints_list[29].x = 315335.046317; 
        my_waypoints_list[29].y = 4071270.719009;

        my_waypoints_list[30].x = 315327.802983; 
        my_waypoints_list[30].y = 4071271.120483;

        my_waypoints_list[31].x = 315321.442312; 
        my_waypoints_list[31].y = 4071271.878596;

        my_waypoints_list[32].x = 315316.682806; 
        my_waypoints_list[32].y = 4071271.477970;

        my_waypoints_list[33].x = 315311.027574; 
        my_waypoints_list[33].y = 4071270.095829;

        my_waypoints_list[34].x = 315303.854352; 
        my_waypoints_list[34].y = 4071267.870212;

        my_waypoints_list[35].x = 315295.795854; 
        my_waypoints_list[35].y = 4071265.162982;

        my_waypoints_list[36].x = 315283.946300; 
        my_waypoints_list[36].y = 4071260.534523;

        my_waypoints_list[37].x = 315272.609709; 
        my_waypoints_list[37].y = 4071256.520490;

        my_waypoints_list[38].x = 315261.023167; 
        my_waypoints_list[38].y = 4071252.511681;

        my_waypoints_list[39].x = 315240.500686; 
        my_waypoints_list[39].y = 4071245.688952;

        my_waypoints_list[40].x = 315225.646500; 
        my_waypoints_list[40].y = 4071240.873240;

        my_waypoints_list[41].x = 315211.560451; 
        my_waypoints_list[41].y = 4071236.916678;

        my_waypoints_list[42].x = 315200.367120; 
        my_waypoints_list[42].y = 4071233.774857;

        my_waypoints_list[43].x = 315191.811330; 
        my_waypoints_list[43].y = 4071231.203049;

        my_waypoints_list[44].x = 315185.393181; 
        my_waypoints_list[44].y = 4071229.211706;

        my_waypoints_list[45].x = 315179.222370; 
        my_waypoints_list[45].y = 4071227.090162;

        my_waypoints_list[46].x = 315174.293474; 
        my_waypoints_list[46].y = 4071224.567568;

        my_waypoints_list[47].x = 315161.485489; 
        my_waypoints_list[47].y = 4071215.958183;

        my_waypoints_list[48].x = 315151.979110; 
        my_waypoints_list[48].y = 4071209.780379;

        my_waypoints_list[49].x = 315143.881420; 
        my_waypoints_list[49].y = 4071205.198516;  // 156
////////////////// 157 -> 50, 166 -> 59 ////////////////////////////
        my_waypoints_list[50].x = 315139.189412; 
        my_waypoints_list[50].y = 4071202.045820;

        my_waypoints_list[51].x = 315131.838961; 
        my_waypoints_list[51].y = 4071197.323306;

        my_waypoints_list[52].x = 315125.626350; 
        my_waypoints_list[52].y = 4071193.202157;

        my_waypoints_list[53].x = 315120.168815; 
        my_waypoints_list[53].y = 4071189.315283;

        my_waypoints_list[54].x = 315116.028956; 
        my_waypoints_list[54].y = 4071188.651642;

        my_waypoints_list[55].x = 315111.673109; 
        my_waypoints_list[55].y = 4071189.617904;

        my_waypoints_list[56].x = 315108.703945; 
        my_waypoints_list[56].y = 4071191.555239;

        my_waypoints_list[57].x = 315106.757696; 
        my_waypoints_list[57].y = 4071193.721611;

        my_waypoints_list[58].x = 315102.756296; 
        my_waypoints_list[58].y = 4071199.681660;

        my_waypoints_list[59].x = 315095.708888; 
        my_waypoints_list[59].y = 4071209.456279;  // 166
////////////////// 174 -> 60, 193 -> 79 ////////////////////////////
        my_waypoints_list[60].x = 315093.514617; 
        my_waypoints_list[60].y = 4071218.129211;  // 174

        my_waypoints_list[61].x = 315095.194157; 
        my_waypoints_list[61].y = 4071220.719730;

        my_waypoints_list[62].x = 315096.738273; 
        my_waypoints_list[62].y = 4071222.812960;

        my_waypoints_list[63].x = 315099.532141; 
        my_waypoints_list[63].y = 4071224.880066;

        my_waypoints_list[64].x = 315104.599076; 
        my_waypoints_list[64].y = 4071228.024925;

        my_waypoints_list[65].x = 315108.905710; 
        my_waypoints_list[65].y = 4071230.685557;

        my_waypoints_list[66].x = 315115.115710; 
        my_waypoints_list[66].y = 4071234.681731;

        my_waypoints_list[67].x = 315121.448072; 
        my_waypoints_list[67].y = 4071238.550316;

        my_waypoints_list[68].x = 315129.683801; 
        my_waypoints_list[68].y = 4071243.754442;

        my_waypoints_list[69].x = 315138.049730; 
        my_waypoints_list[69].y = 4071249.205906;

        my_waypoints_list[70].x = 315145.910533; 
        my_waypoints_list[70].y = 4071254.417869;

        my_waypoints_list[71].x = 315152.883446; 
        my_waypoints_list[71].y = 4071259.023243;

        my_waypoints_list[72].x = 315159.989172; 
        my_waypoints_list[72].y = 4071264.000929;

        my_waypoints_list[73].x = 315166.712134; 
        my_waypoints_list[73].y = 4071268.611527;

        my_waypoints_list[74].x = 315172.682633; 
        my_waypoints_list[74].y = 4071273.112824;

        my_waypoints_list[75].x = 315180.795998; 
        my_waypoints_list[75].y = 4071278.444536;

        my_waypoints_list[76].x = 315189.417101; 
        my_waypoints_list[76].y = 4071284.140723;

        my_waypoints_list[77].x = 315192.046806; 
        my_waypoints_list[77].y = 4071284.335811;

        my_waypoints_list[78].x = 315194.140035; 
        my_waypoints_list[78].y = 4071282.791696;

        my_waypoints_list[79].x = 315197.763892; 
        my_waypoints_list[79].y = 4071276.714510; // 193

        set_delivery_id = 79;
        no_waypoints = 80;
        wp_finish_id = 79;
}

void Humanities_Social_Science_Gwan(void)
{
        my_waypoints_list[0].x = 315435.697968;
        my_waypoints_list[0].y = 4071228.980568;

        my_waypoints_list[1].x = 315459.264668;
        my_waypoints_list[1].y = 4071243.866586;

        my_waypoints_list[2].x = 315501.151300;
        my_waypoints_list[2].y = 4071267.871898;

        my_waypoints_list[3].x = 315502.573051;
        my_waypoints_list[3].y = 4071270.092712;

        my_waypoints_list[4].x = 315502.627913;
        my_waypoints_list[4].y = 4071272.717188;

        my_waypoints_list[5].x = 315501.813174;
        my_waypoints_list[5].y = 4071275.609902;

        my_waypoints_list[6].x = 315488.512653;
        my_waypoints_list[6].y = 4071297.268012;

        my_waypoints_list[7].x = 315477.446009;
        my_waypoints_list[7].y = 4071318.129249;
////////////////// 115 -> 8, 156 -> 49 ////////////////////////////
        my_waypoints_list[8].x = 315472.283268; 
        my_waypoints_list[8].y = 4071322.363152;  // 115

        my_waypoints_list[9].x = 315469.932254; 
        my_waypoints_list[9].y = 4071323.537566;

        my_waypoints_list[10].x = 315466.466914; 
        my_waypoints_list[10].y = 4071325.235392;

        my_waypoints_list[11].x = 315462.840025; 
        my_waypoints_list[11].y = 4071325.186180;

        my_waypoints_list[12].x = 315459.179172; 
        my_waypoints_list[12].y = 4071323.512291;

        my_waypoints_list[13].x = 315451.690691; 
        my_waypoints_list[13].y = 4071318.167525;

        my_waypoints_list[14].x = 315442.678997; 
        my_waypoints_list[14].y = 4071311.729333;

        my_waypoints_list[15].x = 315432.274289; 
        my_waypoints_list[15].y = 4071304.445053;

        my_waypoints_list[16].x = 315426.561584; 
        my_waypoints_list[16].y = 4071300.313462;

        my_waypoints_list[17].x = 315420.468729; 
        my_waypoints_list[17].y = 4071295.939758;

        my_waypoints_list[18].x = 315420.213554; 
        my_waypoints_list[18].y = 4071295.695032;

        my_waypoints_list[19].x = 315410.701959; 
        my_waypoints_list[19].y = 4071289.267288;

        my_waypoints_list[20].x = 315401.807401; 
        my_waypoints_list[20].y = 4071282.451556;

        my_waypoints_list[21].x = 315393.811181; 
        my_waypoints_list[21].y = 4071276.742312;

        my_waypoints_list[22].x = 315386.960638; 
        my_waypoints_list[22].y = 4071272.009357;  // 129

        my_waypoints_list[23].x = 315382.539484; 
        my_waypoints_list[23].y = 4071269.851241;

        my_waypoints_list[24].x = 315378.904755; 
        my_waypoints_list[24].y = 4071269.427103;

        my_waypoints_list[25].x = 315374.405650; 
        my_waypoints_list[25].y = 4071269.521152;

        my_waypoints_list[26].x = 315365.662616; 
        my_waypoints_list[26].y = 4071269.953976;

        my_waypoints_list[27].x = 315353.545252; 
        my_waypoints_list[27].y = 4071270.457337;

        my_waypoints_list[28].x = 315342.805227; 
        my_waypoints_list[28].y = 4071271.056936;

        my_waypoints_list[29].x = 315335.046317; 
        my_waypoints_list[29].y = 4071270.719009;

        my_waypoints_list[30].x = 315327.802983; 
        my_waypoints_list[30].y = 4071271.120483;

        my_waypoints_list[31].x = 315321.442312; 
        my_waypoints_list[31].y = 4071271.878596;

        my_waypoints_list[32].x = 315316.682806; 
        my_waypoints_list[32].y = 4071271.477970;

        my_waypoints_list[33].x = 315311.027574; 
        my_waypoints_list[33].y = 4071270.095829;

        my_waypoints_list[34].x = 315303.854352; 
        my_waypoints_list[34].y = 4071267.870212;

        my_waypoints_list[35].x = 315295.795854; 
        my_waypoints_list[35].y = 4071265.162982;

        my_waypoints_list[36].x = 315283.946300; 
        my_waypoints_list[36].y = 4071260.534523;

        my_waypoints_list[37].x = 315272.609709; 
        my_waypoints_list[37].y = 4071256.520490;

        my_waypoints_list[38].x = 315261.023167; 
        my_waypoints_list[38].y = 4071252.511681;

        my_waypoints_list[39].x = 315240.500686; 
        my_waypoints_list[39].y = 4071245.688952;

        my_waypoints_list[40].x = 315225.646500; 
        my_waypoints_list[40].y = 4071240.873240;

        my_waypoints_list[41].x = 315211.560451; 
        my_waypoints_list[41].y = 4071236.916678;

        my_waypoints_list[42].x = 315200.367120; 
        my_waypoints_list[42].y = 4071233.774857;

        my_waypoints_list[43].x = 315191.811330; 
        my_waypoints_list[43].y = 4071231.203049;

        my_waypoints_list[44].x = 315185.393181; 
        my_waypoints_list[44].y = 4071229.211706;

        my_waypoints_list[45].x = 315179.222370; 
        my_waypoints_list[45].y = 4071227.090162;

        my_waypoints_list[46].x = 315174.293474; 
        my_waypoints_list[46].y = 4071224.567568;

        my_waypoints_list[47].x = 315161.485489; 
        my_waypoints_list[47].y = 4071215.958183;

        my_waypoints_list[48].x = 315151.979110; 
        my_waypoints_list[48].y = 4071209.780379;

        my_waypoints_list[49].x = 315143.881420; 
        my_waypoints_list[49].y = 4071205.198516;  // 156
////////////////// 157 -> 50, 173 -> 66 ////////////////////////////
        my_waypoints_list[50].x = 315139.189412; 
        my_waypoints_list[50].y = 4071202.045820;

        my_waypoints_list[51].x = 315131.838961; 
        my_waypoints_list[51].y = 4071197.323306;

        my_waypoints_list[52].x = 315125.626350; 
        my_waypoints_list[52].y = 4071193.202157;

        my_waypoints_list[53].x = 315120.168815; 
        my_waypoints_list[53].y = 4071189.315283;

        my_waypoints_list[54].x = 315116.028956; 
        my_waypoints_list[54].y = 4071188.651642;

        my_waypoints_list[55].x = 315111.673109; 
        my_waypoints_list[55].y = 4071189.617904;

        my_waypoints_list[56].x = 315108.703945; 
        my_waypoints_list[56].y = 4071191.555239;

        my_waypoints_list[57].x = 315106.757696; 
        my_waypoints_list[57].y = 4071193.721611;

        my_waypoints_list[58].x = 315102.756296; 
        my_waypoints_list[58].y = 4071199.681660;

        my_waypoints_list[59].x = 315095.708888; 
        my_waypoints_list[59].y = 4071209.456279;  // 166

        my_waypoints_list[60].x = 315093.238119; 
        my_waypoints_list[60].y = 4071210.883256;

        my_waypoints_list[61].x = 315090.121575; 
        my_waypoints_list[61].y = 4071211.323493;

        my_waypoints_list[62].x = 315085.632914; 
        my_waypoints_list[62].y = 4071211.917443;

        my_waypoints_list[63].x = 315081.141641; 
        my_waypoints_list[63].y = 4071212.386417;

        my_waypoints_list[64].x = 315073.978862; 
        my_waypoints_list[64].y = 4071210.660698;

        my_waypoints_list[65].x = 315063.610714; 
        my_waypoints_list[65].y = 4071205.126055;

        my_waypoints_list[66].x = 315058.171465; 
        my_waypoints_list[66].y = 4071202.114007;  // 173

        set_delivery_id = 66;
        no_waypoints = 67;
        wp_finish_id = 66;
}

void Main_University(void)
{
        my_waypoints_list[0].x = 315435.697968;
        my_waypoints_list[0].y = 4071228.980568;

        my_waypoints_list[1].x = 315459.264668;
        my_waypoints_list[1].y = 4071243.866586;

        my_waypoints_list[2].x = 315501.151300;
        my_waypoints_list[2].y = 4071267.871898;

        my_waypoints_list[3].x = 315502.573051;
        my_waypoints_list[3].y = 4071270.092712;

        my_waypoints_list[4].x = 315502.627913;
        my_waypoints_list[4].y = 4071272.717188;

        my_waypoints_list[5].x = 315501.813174;
        my_waypoints_list[5].y = 4071275.609902;

        my_waypoints_list[6].x = 315488.512653;
        my_waypoints_list[6].y = 4071297.268012;

        my_waypoints_list[7].x = 315477.446009;
        my_waypoints_list[7].y = 4071318.129249;
////////////////// 115 -> 8, 156 -> 49 ////////////////////////////
        my_waypoints_list[8].x = 315472.283268; 
        my_waypoints_list[8].y = 4071322.363152;

        my_waypoints_list[9].x = 315469.932254; 
        my_waypoints_list[9].y = 4071323.537566;

        my_waypoints_list[10].x = 315466.466914; 
        my_waypoints_list[10].y = 4071325.235392;

        my_waypoints_list[11].x = 315462.840025; 
        my_waypoints_list[11].y = 4071325.186180;

        my_waypoints_list[12].x = 315459.179172; 
        my_waypoints_list[12].y = 4071323.512291;

        my_waypoints_list[13].x = 315451.690691; 
        my_waypoints_list[13].y = 4071318.167525;

        my_waypoints_list[14].x = 315442.678997; 
        my_waypoints_list[14].y = 4071311.729333;

        my_waypoints_list[15].x = 315432.274289; 
        my_waypoints_list[15].y = 4071304.445053;

        my_waypoints_list[16].x = 315426.561584; 
        my_waypoints_list[16].y = 4071300.313462;

        my_waypoints_list[17].x = 315420.468729; 
        my_waypoints_list[17].y = 4071295.939758;

        my_waypoints_list[18].x = 315420.213554; 
        my_waypoints_list[18].y = 4071295.695032;

        my_waypoints_list[19].x = 315410.701959; 
        my_waypoints_list[19].y = 4071289.267288;

        my_waypoints_list[20].x = 315401.807401; 
        my_waypoints_list[20].y = 4071282.451556;

        my_waypoints_list[21].x = 315393.811181; 
        my_waypoints_list[21].y = 4071276.742312;

        my_waypoints_list[22].x = 315386.960638; 
        my_waypoints_list[22].y = 4071272.009357;  // 129

        my_waypoints_list[23].x = 315382.539484; 
        my_waypoints_list[23].y = 4071269.851241;

        my_waypoints_list[24].x = 315378.904755; 
        my_waypoints_list[24].y = 4071269.427103;

        my_waypoints_list[25].x = 315374.405650; 
        my_waypoints_list[25].y = 4071269.521152;

        my_waypoints_list[26].x = 315365.662616; 
        my_waypoints_list[26].y = 4071269.953976;

        my_waypoints_list[27].x = 315353.545252; 
        my_waypoints_list[27].y = 4071270.457337;

        my_waypoints_list[28].x = 315342.805227; 
        my_waypoints_list[28].y = 4071271.056936;

        my_waypoints_list[29].x = 315335.046317; 
        my_waypoints_list[29].y = 4071270.719009;

        my_waypoints_list[30].x = 315327.802983; 
        my_waypoints_list[30].y = 4071271.120483;

        my_waypoints_list[31].x = 315321.442312; 
        my_waypoints_list[31].y = 4071271.878596;

        my_waypoints_list[32].x = 315316.682806; 
        my_waypoints_list[32].y = 4071271.477970;

        my_waypoints_list[33].x = 315311.027574; 
        my_waypoints_list[33].y = 4071270.095829;

        my_waypoints_list[34].x = 315303.854352; 
        my_waypoints_list[34].y = 4071267.870212;

        my_waypoints_list[35].x = 315295.795854; 
        my_waypoints_list[35].y = 4071265.162982;

        my_waypoints_list[36].x = 315283.946300; 
        my_waypoints_list[36].y = 4071260.534523;

        my_waypoints_list[37].x = 315272.609709; 
        my_waypoints_list[37].y = 4071256.520490;

        my_waypoints_list[38].x = 315261.023167; 
        my_waypoints_list[38].y = 4071252.511681;

        my_waypoints_list[39].x = 315240.500686; 
        my_waypoints_list[39].y = 4071245.688952;

        my_waypoints_list[40].x = 315225.646500; 
        my_waypoints_list[40].y = 4071240.873240;

        my_waypoints_list[41].x = 315211.560451; 
        my_waypoints_list[41].y = 4071236.916678;

        my_waypoints_list[42].x = 315200.367120; 
        my_waypoints_list[42].y = 4071233.774857;

        my_waypoints_list[43].x = 315191.811330; 
        my_waypoints_list[43].y = 4071231.203049;

        my_waypoints_list[44].x = 315185.393181; 
        my_waypoints_list[44].y = 4071229.211706;

        my_waypoints_list[45].x = 315179.222370; 
        my_waypoints_list[45].y = 4071227.090162;

        my_waypoints_list[46].x = 315174.293474; 
        my_waypoints_list[46].y = 4071224.567568;

        my_waypoints_list[47].x = 315161.485489; 
        my_waypoints_list[47].y = 4071215.958183;

        my_waypoints_list[48].x = 315151.979110; 
        my_waypoints_list[48].y = 4071209.780379;

        my_waypoints_list[49].x = 315143.881420; 
        my_waypoints_list[49].y = 4071205.198516;  // 156

        set_delivery_id = 49;
        no_waypoints = 50;
        wp_finish_id = 49;
}

void Media_Laps_Gwan(void)
{

}

void Global_Village(void)
{
        my_waypoints_list[0].x = 315435.697968;
        my_waypoints_list[0].y = 4071228.980568;

        my_waypoints_list[1].x = 315459.264668;
        my_waypoints_list[1].y = 4071243.866586;

        my_waypoints_list[2].x = 315501.151300;
        my_waypoints_list[2].y = 4071267.871898;

        my_waypoints_list[3].x = 315502.573051;
        my_waypoints_list[3].y = 4071270.092712;

        my_waypoints_list[4].x = 315502.627913;
        my_waypoints_list[4].y = 4071272.717188;

        my_waypoints_list[5].x = 315501.813174;
        my_waypoints_list[5].y = 4071275.609902;

        my_waypoints_list[6].x = 315488.512653;
        my_waypoints_list[6].y = 4071297.268012;

        my_waypoints_list[7].x = 315477.446009;
        my_waypoints_list[7].y = 4071318.129249;

        my_waypoints_list[8].x = 315476.912571;
        my_waypoints_list[8].y = 4071322.516440;

        my_waypoints_list[9].x = 315478.737985;
        my_waypoints_list[9].y = 4071326.104142;

        my_waypoints_list[10].x = 315483.127787;
        my_waypoints_list[10].y = 4071326.762554;
////////////////// 15 -> , 36 -> 32 ////////////////////////////
        my_waypoints_list[11].x = 315490.235691;
        my_waypoints_list[11].y = 4071325.863791;

        my_waypoints_list[12].x = 315494.916819;
        my_waypoints_list[12].y = 4071322.515163;

        my_waypoints_list[13].x = 315509.459676;
        my_waypoints_list[13].y = 4071306.457417;

        my_waypoints_list[14].x = 315514.372466;
        my_waypoints_list[14].y = 4071302.228741;

        my_waypoints_list[15].x = 315519.217755;
        my_waypoints_list[15].y = 4071300.752128;

        my_waypoints_list[16].x = 315524.586457;
        my_waypoints_list[16].y = 4071300.389840;

        my_waypoints_list[17].x = 315529.353797;
        my_waypoints_list[17].y = 4071301.165391;

        my_waypoints_list[18].x = 315534.298361;
        my_waypoints_list[18].y = 4071304.437829;

        my_waypoints_list[19].x = 315545.921465;
        my_waypoints_list[19].y = 4071310.196281;

        my_waypoints_list[20].x = 315560.942407;
        my_waypoints_list[20].y = 4071317.008971;

        my_waypoints_list[21].x = 315566.352908;
        my_waypoints_list[21].y = 4071318.646283;

        my_waypoints_list[22].x = 315565.597833;
        my_waypoints_list[22].y = 4071318.412008;

        my_waypoints_list[23].x = 315569.617934;
        my_waypoints_list[23].y = 4071319.328208;

        my_waypoints_list[24].x = 315575.252260;
        my_waypoints_list[24].y = 4071319.710546;

        my_waypoints_list[25].x = 315579.251461;
        my_waypoints_list[25].y = 4071319.626945;

        my_waypoints_list[26].x = 315582.615336;
        my_waypoints_list[26].y = 4071319.056508;

        my_waypoints_list[27].x = 315585.599061;
        my_waypoints_list[27].y = 4071318.243957;

        my_waypoints_list[28].x = 315589.434099;
        my_waypoints_list[28].y = 4071316.288344;

        my_waypoints_list[29].x = 315593.508637;
        my_waypoints_list[29].y = 4071313.827607;

        my_waypoints_list[30].x = 315597.075438;
        my_waypoints_list[30].y = 4071311.002394;

        my_waypoints_list[31].x = 315599.876713;
        my_waypoints_list[31].y = 4071307.443006;

        my_waypoints_list[32].x = 315602.654476;
        my_waypoints_list[32].y = 4071302.758843;  // 36
////////////////// 42 -> 33, 43 -> 34 ////////////////////////////

        my_waypoints_list[33].x = 315607.587739;
        my_waypoints_list[33].y = 4071293.528555;

        my_waypoints_list[34].x = 315626.399368;
        my_waypoints_list[34].y = 4071260.377558;  // 43
////////////////// 73 -> 35, 75 -> 37 ////////////////////////////
        my_waypoints_list[35].x = 315632.800983; 
        my_waypoints_list[35].y = 4071249.616223;

        my_waypoints_list[36].x = 315640.142311; 
        my_waypoints_list[36].y = 4071235.959564;

        my_waypoints_list[37].x = 315647.140065; 
        my_waypoints_list[37].y = 4071223.810442;  // 75
////////////////// 79 -> 38, 91 -> 50 ////////////////////////////

        my_waypoints_list[38].x = 315652.554943; 
        my_waypoints_list[38].y = 4071213.694884;

        my_waypoints_list[39].x = 315656.181407; 
        my_waypoints_list[39].y = 4071207.742686;

        my_waypoints_list[40].x = 315658.706609; 
        my_waypoints_list[40].y = 4071202.938776;

        my_waypoints_list[41].x = 315667.422664; 
        my_waypoints_list[41].y = 4071189.253383;

        my_waypoints_list[42].x = 315669.218916; 
        my_waypoints_list[42].y = 4071185.464947;

        my_waypoints_list[43].x = 315670.648080; 
        my_waypoints_list[43].y = 4071182.059274;

        my_waypoints_list[44].x = 315674.308085; 
        my_waypoints_list[44].y = 4071171.730341;

        my_waypoints_list[45].x = 315674.974337; 
        my_waypoints_list[45].y = 4071167.715467;

        my_waypoints_list[46].x = 315674.244965; 
        my_waypoints_list[46].y = 4071162.729531;

        my_waypoints_list[47].x = 315672.823217; 
        my_waypoints_list[47].y = 4071160.508718;

        my_waypoints_list[48].x = 315666.805711; 
        my_waypoints_list[48].y = 4071153.757878;

        my_waypoints_list[49].x = 315662.465127; 
        my_waypoints_list[49].y = 4071149.472576;

        my_waypoints_list[50].x = 315657.869369; 
        my_waypoints_list[50].y = 4071144.942548;  // 91
////////////////// 92 -> 51, 114 ->  ////////////////////////////
        my_waypoints_list[51].x = 315652.258136; 
        my_waypoints_list[51].y = 4071139.683570;

        my_waypoints_list[52].x = 315647.133741; 
        my_waypoints_list[52].y = 4071133.789267;

        my_waypoints_list[53].x = 315641.504222; 
        my_waypoints_list[53].y = 4071127.655463;

        my_waypoints_list[54].x = 315637.033438; 
        my_waypoints_list[54].y = 4071123.122823;

        my_waypoints_list[55].x = 315631.528894; 
        my_waypoints_list[55].y = 4071116.986406;

        my_waypoints_list[56].x = 315627.302836; 
        my_waypoints_list[56].y = 4071112.198590;

        my_waypoints_list[57].x = 315622.295579; 
        my_waypoints_list[57].y = 4071105.926749;

        my_waypoints_list[58].x = 315619.090220; 
        my_waypoints_list[58].y = 4071102.117833;

        my_waypoints_list[59].x = 315612.856724; 
        my_waypoints_list[59].y = 4071096.996890;

        my_waypoints_list[60].x = 315606.365442; 
        my_waypoints_list[60].y = 4071091.506246;

        my_waypoints_list[61].x = 315599.514909; 
        my_waypoints_list[61].y = 4071086.773289;

        my_waypoints_list[62].x = 315594.713611; 
        my_waypoints_list[62].y = 4071084.373059;

        my_waypoints_list[63].x = 315588.839785; 
        my_waypoints_list[63].y = 4071084.495841;

        my_waypoints_list[64].x = 315584.142982; 
        my_waypoints_list[64].y = 4071087.094612;

        my_waypoints_list[65].x = 315579.555479; 
        my_waypoints_list[65].y = 4071088.940920;

        my_waypoints_list[66].x = 315574.931403; 
        my_waypoints_list[66].y = 4071089.037578;

        my_waypoints_list[67].x = 315569.882767; 
        my_waypoints_list[67].y = 4071086.767548;

        my_waypoints_list[68].x = 315567.429870; 
        my_waypoints_list[68].y = 4071083.067932;

        my_waypoints_list[69].x = 315566.216274; 
        my_waypoints_list[69].y = 4071078.842293;

        my_waypoints_list[70].x = 315567.486504; 
        my_waypoints_list[70].y = 4071073.814555;

        my_waypoints_list[71].x = 315570.753723; 
        my_waypoints_list[71].y = 4071068.620045;

        my_waypoints_list[72].x = 315575.211026; 
        my_waypoints_list[72].y = 4071066.526399;

        my_waypoints_list[73].x = 315582.964702; 
        my_waypoints_list[73].y = 4071066.614383;

        set_delivery_id = 73;
        no_waypoints = 74;
        wp_finish_id = 73;
}

void Hyang_333(void)
{
        my_waypoints_list[0].x = 315435.697968;
        my_waypoints_list[0].y = 4071228.980568;

        my_waypoints_list[1].x = 315459.264668;
        my_waypoints_list[1].y = 4071243.866586;

        my_waypoints_list[2].x = 315501.151300;
        my_waypoints_list[2].y = 4071267.871898;

        my_waypoints_list[3].x = 315502.573051;
        my_waypoints_list[3].y = 4071270.092712;

        my_waypoints_list[4].x = 315502.627913;
        my_waypoints_list[4].y = 4071272.717188;

        my_waypoints_list[5].x = 315501.813174;
        my_waypoints_list[5].y = 4071275.609902;

        my_waypoints_list[6].x = 315488.512653;
        my_waypoints_list[6].y = 4071297.268012;

        my_waypoints_list[7].x = 315477.446009;
        my_waypoints_list[7].y = 4071318.129249;

        my_waypoints_list[8].x = 315476.912571;
        my_waypoints_list[8].y = 4071322.516440;

        my_waypoints_list[9].x = 315478.737985;
        my_waypoints_list[9].y = 4071326.104142;

        my_waypoints_list[10].x = 315483.127787;
        my_waypoints_list[10].y = 4071326.762554;
////////////////// 15 -> , 36 -> 32 ////////////////////////////
        my_waypoints_list[11].x = 315490.235691;
        my_waypoints_list[11].y = 4071325.863791;

        my_waypoints_list[12].x = 315494.916819;
        my_waypoints_list[12].y = 4071322.515163;

        my_waypoints_list[13].x = 315509.459676;
        my_waypoints_list[13].y = 4071306.457417;

        my_waypoints_list[14].x = 315514.372466;
        my_waypoints_list[14].y = 4071302.228741;

        my_waypoints_list[15].x = 315519.217755;
        my_waypoints_list[15].y = 4071300.752128;

        my_waypoints_list[16].x = 315524.586457;
        my_waypoints_list[16].y = 4071300.389840;

        my_waypoints_list[17].x = 315529.353797;
        my_waypoints_list[17].y = 4071301.165391;

        my_waypoints_list[18].x = 315534.298361;
        my_waypoints_list[18].y = 4071304.437829;

        my_waypoints_list[19].x = 315545.921465;
        my_waypoints_list[19].y = 4071310.196281;

        my_waypoints_list[20].x = 315560.942407;
        my_waypoints_list[20].y = 4071317.008971;

        my_waypoints_list[21].x = 315566.352908;
        my_waypoints_list[21].y = 4071318.646283;

        my_waypoints_list[22].x = 315565.597833;
        my_waypoints_list[22].y = 4071318.412008;

        my_waypoints_list[23].x = 315569.617934;
        my_waypoints_list[23].y = 4071319.328208;

        my_waypoints_list[24].x = 315575.252260;
        my_waypoints_list[24].y = 4071319.710546;

        my_waypoints_list[25].x = 315579.251461;
        my_waypoints_list[25].y = 4071319.626945;

        my_waypoints_list[26].x = 315582.615336;
        my_waypoints_list[26].y = 4071319.056508;

        my_waypoints_list[27].x = 315585.599061;
        my_waypoints_list[27].y = 4071318.243957;

        my_waypoints_list[28].x = 315589.434099;
        my_waypoints_list[28].y = 4071316.288344;

        my_waypoints_list[29].x = 315593.508637;
        my_waypoints_list[29].y = 4071313.827607;

        my_waypoints_list[30].x = 315597.075438;
        my_waypoints_list[30].y = 4071311.002394;

        my_waypoints_list[31].x = 315599.876713;
        my_waypoints_list[31].y = 4071307.443006;

        my_waypoints_list[32].x = 315602.654476;
        my_waypoints_list[32].y = 4071302.758843;  // 36
////////////////// 42 -> 33, 43 -> 34 ////////////////////////////

        my_waypoints_list[33].x = 315607.587739;
        my_waypoints_list[33].y = 4071293.528555;

        my_waypoints_list[34].x = 315626.399368;
        my_waypoints_list[34].y = 4071260.377558;  // 43
////////////////// 73 -> 35, 75 -> 37 ////////////////////////////
        my_waypoints_list[35].x = 315632.800983; 
        my_waypoints_list[35].y = 4071249.616223;

        my_waypoints_list[36].x = 315640.142311; 
        my_waypoints_list[36].y = 4071235.959564;

        my_waypoints_list[37].x = 315647.140065; 
        my_waypoints_list[37].y = 4071223.810442;  // 75
////////////////// 79 -> 38, 91 -> 50 ////////////////////////////

        my_waypoints_list[38].x = 315652.554943; 
        my_waypoints_list[38].y = 4071213.694884;

        my_waypoints_list[39].x = 315656.181407; 
        my_waypoints_list[39].y = 4071207.742686;

        my_waypoints_list[40].x = 315658.706609; 
        my_waypoints_list[40].y = 4071202.938776;

        my_waypoints_list[41].x = 315667.422664; 
        my_waypoints_list[41].y = 4071189.253383;

        my_waypoints_list[42].x = 315669.218916; 
        my_waypoints_list[42].y = 4071185.464947;

        my_waypoints_list[43].x = 315670.648080; 
        my_waypoints_list[43].y = 4071182.059274;

        my_waypoints_list[44].x = 315674.308085; 
        my_waypoints_list[44].y = 4071171.730341;

        my_waypoints_list[45].x = 315674.974337; 
        my_waypoints_list[45].y = 4071167.715467;

        my_waypoints_list[46].x = 315674.244965; 
        my_waypoints_list[46].y = 4071162.729531;

        my_waypoints_list[47].x = 315672.823217; 
        my_waypoints_list[47].y = 4071160.508718;

        my_waypoints_list[48].x = 315666.805711; 
        my_waypoints_list[48].y = 4071153.757878;

        my_waypoints_list[49].x = 315662.465127; 
        my_waypoints_list[49].y = 4071149.472576;

        my_waypoints_list[50].x = 315657.869369; 
        my_waypoints_list[50].y = 4071144.942548;  // 91

        set_delivery_id = 50;
        no_waypoints = 51;
        wp_finish_id = 50;

}



void base_link_tf_utm(void)
{
        double waypoint_pos_base_link_x     = 0.0;
        double waypoint_pos_base_link_y     = 0.0;
        double waypoint_pos_base_link_theta = 0.0;
        double tf_base_map_x,tf_base_map_y;
        double waypoint_angle, waypoint_distance;
        ///////////////////////////////////////////////////
        tf_base_map_x = -my_pose.x;   //상대좌표로 변환  no translation
        tf_base_map_y = -my_pose.y;

        tf_base_map_x += my_target_pose_goal.x;   //상대좌표로 변환  no translation
        tf_base_map_y += my_target_pose_goal.y;

        waypoint_pos_base_link_x = tf_base_map_x * cos(my_pose.theta)  + tf_base_map_y * sin(my_pose.theta);   // rotation_matrix
        waypoint_pos_base_link_y = -tf_base_map_x * sin(my_pose.theta) + tf_base_map_y * cos(my_pose.theta);

        waypoint_angle = atan2(waypoint_pos_base_link_y ,waypoint_pos_base_link_x);
        waypoint_distance = sqrt(waypoint_pos_base_link_x*waypoint_pos_base_link_x  + waypoint_pos_base_link_y*waypoint_pos_base_link_y);
        car_angle = RAD2DEG(waypoint_angle);

        ROS_INFO(" X : %6.3lf   Y : %6.3lf  Yaw : %6.3lf ", my_pose.x, my_pose.y, RAD2DEG(my_pose.theta));
        ROS_INFO(" b_x : %6.3lf  b_y : %6.3lf",waypoint_pos_base_link_x,waypoint_pos_base_link_y);

}

void emergency_stop_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    emergency_stop = msg->data;
}

void start_command_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    start_command = msg->data;
}

void Receive_Product_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    received_product = msg->data;
}

void lidar_object_detect_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    lidar_object_detect = msg->data;
}

void avoid_function_start_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    if( (avoid_function_start == false) && msg->data) { avoid_function_start = msg->data; }
}

void avoid_heading_angle_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    avoid_heading_angle_right = msg->data[0];
    avoid_heading_angle_left = msg->data[1];

}



string target_string;
bool sub_destination = false;

void destination_Callback(const std_msgs::String::ConstPtr& msg)
{
    sub_destination = true;
    target_string = msg->data;
    ROS_INFO("target_string %s", target_string.c_str());

}

void detect_name_Callback(const std_msgs::String::ConstPtr& msg)
{
    detect_name = msg->data;

}

void depth_Callback(const std_msgs::Float64::ConstPtr& msg)
{
    depth = msg->data;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Outdoor_Navigation_Drive");
    ros::NodeHandle n;

    std_msgs::Int16 s_angle;
    std_msgs::Int16 c_speed;
    std_msgs::Int16 ros_waypoint_id;

    //ros::Subscriber sub2 = n.subscribe("/gps/ublox_fix",10, &gps_utm_poseCallback); //

    ros::Subscriber utm_sub                  = n.subscribe("/utm",1,&utm_Callback);
    ros::Subscriber imu_sub                  = n.subscribe("/imu/data",10,&imu_Callback);
    ros::Subscriber receive_product_sub      = n.subscribe("/receive_product",10,&Receive_Product_Callback);
    ros::Subscriber start_command_sub      = n.subscribe("/start_command",10,&start_command_Callback);
    ros::Subscriber emergency_stop_sub      = n.subscribe("/emergency_stop",10,&emergency_stop_Callback);
    ros::Subscriber lidar_object_detect_sub  = n.subscribe("/lidar_object_detect",10,&lidar_object_detect_Callback);
    ros::Subscriber avoid_function_start_sub = n.subscribe("/avoid_function_start",10,&avoid_function_start_Callback);
    ros::Subscriber avoid_heading_angle_sub  = n.subscribe("/avoid_heading_angle",10,&avoid_heading_angle_Callback);
    ros::Subscriber destination_sub  = n.subscribe<std_msgs::String>("/destination",10,&destination_Callback);
    ros::Subscriber detect_name_sub  = n.subscribe("/detect_name",10,&detect_name_Callback);
    ros::Subscriber depth_sub  = n.subscribe("/depth",10,&depth_Callback);

    ros::Publisher SteerAngle_pub            = n.advertise<std_msgs::Int16>("Car_Control_cmd/SteerAngle_Int16", 10);
    ros::Publisher car_speed_pub             = n.advertise<std_msgs::Int16>("Car_Control_cmd/Speed_Int16", 10);
    ros::Publisher target_id_pub             = n.advertise<std_msgs::Int16>("target_id",2);
    ros::Publisher target_pos_pub            = n.advertise<geometry_msgs::Pose2D>("/pose_goal", 10);

    ros::Rate loop_rate(5);  // 10


    while (ros::ok() && (sub_destination == false) ){

        ROS_INFO("Wati sub Destination");

        ros::spinOnce();
    }



    long count = 0;
    double pos_error_x = 0.0;
    double pos_error_y = 0.0;

    double waypoint_distance = 0.0;
    double waypoint_gap_distance = 0.0;

    geometry_msgs::Pose2D pose_goal;

    if(target_string == "init"){ init_waypoint(); } ////////////////////////////////////////////////////////////////////////////////
    else if( target_string == "unitophiagwan" ){ Unitophia(); }
    else if( target_string == "multimediagwan" ){ Multi_Media_Gwan(); }
    else if( target_string == "hakyegwan" ){ Hak_Ye_Gwan(); }
    else if( target_string == "brixgwan" ){ BRIX_Gwan(); }
    else if( target_string == "sanhakhyeopryeokgGwan" ){ San_Hak_Hyeop_Ryeok_Gwan(); }
    else if( target_string == "gonghakhwan" ){ Gong_Hak_Gwan(); }
    else if( target_string == "library" ){ Library(); }
    else if( target_string == "antirepreneurgwan" ){ Antire_preneur_Gwan(); }
    else if( target_string == "naturelsciencegwan" ){ Naturel_Science_Gwan(); }
    else if( target_string == "humanitiessocialsciencegwan" ){ Humanities_Social_Science_Gwan(); }
    else if( target_string == "mainuniversity" ){ Main_University(); }
    else if( target_string == "globalvillage" ){ Global_Village(); }
    else if( target_string == "hyangthree" ){ Hyang_333(); }
    //else{ init_waypoint(); }

    int waypoint_id = 0;

    double delta_x, delta_y ;
    delta_x = delta_y = 0.0;


    pose_goal.x = my_waypoints_list[wp_go_id].x;
    pose_goal.y = my_waypoints_list[wp_go_id].y;
    pose_goal.theta = DEG2RAD(0);
    target_pos_pub.publish(pose_goal);

    while (ros::ok()) {

        if(emergency_stop == false){
            if(start_command == true){
                c_speed.data = 100;
                s_angle.data = car_angle;
            }
            else if(start_command == false){
                c_speed.data = 0;
                s_angle.data = 0;
            }
        }
        else if(emergency_stop == true){
            c_speed.data = 0;
            s_angle.data = 0;
        }


        if(waypoint_id!= -1) {

            my_target_pose_goal.x = my_waypoints_list[wp_go_id].x;
            my_target_pose_goal.y = my_waypoints_list[wp_go_id].y;

                if(wp_go_id == 0) {
                    delta_x = my_waypoints_list[1].x - my_waypoints_list[0].x;
                    delta_y = my_waypoints_list[1].y - my_waypoints_list[0].y;

                    my_target_pose_goal_prev.x = my_waypoints_list[0].x - delta_x;
                    my_target_pose_goal_prev.y = my_waypoints_list[0].y - delta_y;

                    delta_x = my_target_pose_goal.x - my_target_pose_goal_prev.x;
                    delta_y = my_target_pose_goal.y - my_target_pose_goal_prev.y;
                    waypoint_line_angle = atan2(delta_y, delta_x);
                }
                else {
                    my_target_pose_goal_prev.x = my_waypoints_list[wp_go_id-1].x;
                    my_target_pose_goal_prev.y = my_waypoints_list[wp_go_id-1].y;

                    delta_x = my_target_pose_goal.x - my_target_pose_goal_prev.x;
                    delta_y = my_target_pose_goal.y - my_target_pose_goal_prev.y;
                    waypoint_line_angle = atan2(delta_y, delta_x);
                }

            waypoint_distance = sqrt(delta_x*delta_x + delta_y*delta_y);
            waypoint_gap_distance = sqrt(delta_x*delta_x + delta_y*delta_y) - WayPoint_X_Tor;

            printf("1: angle %lf\n", RAD2DEG(waypoint_line_angle));

            pos_error_x = abs(my_pose.x - my_waypoints_list[wp_go_id].x);
            pos_error_y = abs(my_pose.y - my_waypoints_list[wp_go_id].y);

            pose_goal.x =  my_waypoints_list[wp_go_id].x; // umt coordinate ////////////////
            pose_goal.y =  my_waypoints_list[wp_go_id].y;  // umt coordinate /////////////////


            pose_goal.theta = DEG2RAD(0);
            ROS_INFO("[%3d]WayPoint goal X : %6.3lf  goal Y : %6.3lf ",wp_go_id, my_target_pose_goal.x, my_target_pose_goal.y);

            base_link_tf_utm();

            target_pos_pub.publish(pose_goal);
            ros_waypoint_id.data  = wp_go_id;


            //if( (count>=0) && (pos_error_x <= WayPoint_X_Tor) && (pos_error_y <= WayPoint_Y_Tor )  )
            if( (pos_error_x <= WayPoint_X_Tor) && (pos_error_y <= WayPoint_Y_Tor )) {
                printf("----------------------------\n");
                printf("Arrvied at My WayPoint[%3d] !\n",wp_go_id);
                printf("----------------------------\n");

                if(wp_go_id != set_delivery_id) { wp_go_id++; }

                if( (wp_go_id == set_delivery_id) && (received_product == false) ) {
                    c_speed.data = 0;
                    s_angle.data = 0;
	        }
                else if( (wp_go_id == set_delivery_id) && (received_product == true) ) {
                    c_speed.data = 100;
                    s_angle.data = car_angle;
                    wp_go_id++;
                }

            }




            // avoid function
            if( (wp_go_id < wp_finish_id) && (avoid_function_start == true) && (wp_go_id != set_delivery_id) && (start_command == true) ){
                c_speed.data = 60;
                s_angle.data = avoid_heading_angle_left + 30;

                if( (avoid_heading_angle_left > 5) && (avoid_heading_angle_right > 5)) {
                    s_angle.data = car_angle - 5;
                }

                avoid_function_start = false;

                printf("----------------------------\n");
                printf("Avoid function\n");
                printf("----------------------------\n");
                ROS_INFO("steering_angle : %d Speed : %d \n",s_angle.data ,c_speed.data);



                //SteerAngle_pub.publish(s_angle);
                //car_speed_pub.publish(c_speed);
                //ros::Duration(0.5).sleep();
            }

            if(wp_go_id > wp_finish_id) {
                         start_command = false;
                         c_speed.data = 0;
                         s_angle.data = 0;
                         //wp_go_id = wp_finish_id;
                         ROS_INFO("WP Mission Completed");
            }

        }
        //.c_str()
	if(emergency_stop == true || ((detect_name == "person") && (depth < 2.3)) ){ 
            c_speed.data = 0;
            s_angle.data = 0;
        }

        // publish topics
        target_id_pub.publish(ros_waypoint_id);
        ROS_INFO("steering_angle : %d Speed : %d \n",s_angle.data ,c_speed.data);
        //ROS_INFO("destination_data : %s\n", destination_data );

        if(count>=2) {

            SteerAngle_pub.publish(s_angle);
            car_speed_pub.publish(c_speed);
        }

        avoid_function_start = false; //////////

        loop_rate.sleep();
    ros::spinOnce();
    ++count;

  }
    return 0;
}
