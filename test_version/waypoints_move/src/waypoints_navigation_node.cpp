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
#define WayPoints_NO 200
#define WayPoint_X_Tor 0.3
#define WayPoint_Y_Tor 0.5



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
double start_joy = false; /////////////

double sub_joy_current_time;


geometry_msgs::Pose2D my_pose;
geometry_msgs::Pose2D my_target_pose_goal;
geometry_msgs::Pose2D my_target_pose_goal_prev;
geometry_msgs::PoseStamped utm_fix;

std::string destination_data;
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
        wp_finish_id = 12;
}

void Unitophia(void)
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
	
////////////////// 365 -> 33, 369 -> 37 ////////////////////////////
	my_waypoints_list[33].x = 315603.690426;
        my_waypoints_list[33].y = 4071298.486180;

	my_waypoints_list[34].x = 315601.758326;
        my_waypoints_list[34].y = 4071295.775918;

	my_waypoints_list[35].x = 315592.655625;
        my_waypoints_list[35].y = 4071290.965017;

	my_waypoints_list[36].x = 315593.545701;
        my_waypoints_list[36].y = 4071285.695166;

	my_waypoints_list[37].x = 315594.959189;
        my_waypoints_list[37].y = 4071281.539641;  // 369 guest
	
////////////////// 370 -> 38, 374 -> 42 ////////////////////////////  back_init_place
	my_waypoints_list[38].x = 315598.715853;
        my_waypoints_list[38].y = 4071275.834778;  // 370

	my_waypoints_list[39].x = 315595.592326;
        my_waypoints_list[39].y = 4071287.902917;

	my_waypoints_list[40].x = 315595.925876;
        my_waypoints_list[40].y = 4071291.896892;

	my_waypoints_list[41].x = 315603.651238;
        my_waypoints_list[41].y = 4071296.611555;

	my_waypoints_list[42].x = 315606.351476;
        my_waypoints_list[42].y = 4071300.180968;  // 374
////////////////// 283 -> 43, 326 -> 86 //////////////////////////// 
	my_waypoints_list[43].x = 315606.398501;
        my_waypoints_list[43].y = 4071302.430518;  // 283

	my_waypoints_list[44].x = 315603.865463;
        my_waypoints_list[44].y = 4071306.859506;

	my_waypoints_list[45].x = 315601.439113;
        my_waypoints_list[45].y = 4071310.411056;

	my_waypoints_list[46].x = 315598.520700;
        my_waypoints_list[46].y = 4071314.347981;

	my_waypoints_list[47].x = 315595.206463;
        my_waypoints_list[47].y = 4071317.292944;

	my_waypoints_list[48].x = 315591.012175;
        my_waypoints_list[48].y = 4071320.006245;

	my_waypoints_list[49].x = 315587.421862;
        my_waypoints_list[49].y = 4071321.706683;

	my_waypoints_list[50].x = 315582.946274;
        my_waypoints_list[50].y = 4071322.925508;

	my_waypoints_list[51].x = 315579.957323;
        my_waypoints_list[51].y = 4071323.488108;

	my_waypoints_list[52].x = 315574.336060;
        my_waypoints_list[52].y = 4071323.730646;

	my_waypoints_list[53].x = 315569.693696;
        my_waypoints_list[53].y = 4071322.952484;

	my_waypoints_list[54].x = 315564.413395;
        my_waypoints_list[54].y = 4071321.562509;

	my_waypoints_list[55].x = 315558.739881;
        my_waypoints_list[55].y = 4071319.305546;

	my_waypoints_list[56].x = 315548.642603;
        my_waypoints_list[56].y = 4071314.765495;

	my_waypoints_list[57].x = 315539.433213;
        my_waypoints_list[57].y = 4071310.832032;

	my_waypoints_list[58].x = 315530.596135;
        my_waypoints_list[58].y = 4071306.765755;

	my_waypoints_list[59].x = 315525.084170;
        my_waypoints_list[59].y = 4071306.255830;

	my_waypoints_list[60].x = 315519.478580;
        my_waypoints_list[60].y = 4071307.248218;
	
	my_waypoints_list[61].x = 315516.000179;
        my_waypoints_list[61].y = 4071308.321168;

	my_waypoints_list[62].x = 315511.298152;
        my_waypoints_list[62].y = 4071310.669994;

	my_waypoints_list[63].x = 315507.247125;
        my_waypoints_list[63].y = 4071314.255509;

	my_waypoints_list[64].x = 315500.303809;
        my_waypoints_list[64].y = 4071323.027701;

	my_waypoints_list[65].x = 315497.273483;
        my_waypoints_list[65].y = 4071327.592116;

	my_waypoints_list[66].x = 315493.961856;
        my_waypoints_list[66].y = 4071330.662055;

	my_waypoints_list[67].x = 315489.238929;
        my_waypoints_list[67].y = 4071332.011082;

	my_waypoints_list[68].x = 315483.240126;
        my_waypoints_list[68].y = 4071332.136482;

	my_waypoints_list[69].x = 315479.110723;
        my_waypoints_list[69].y = 4071331.972745;

	my_waypoints_list[70].x = 315474.215795;
        my_waypoints_list[70].y = 4071331.074833;

	my_waypoints_list[71].x = 315464.506501;
        my_waypoints_list[71].y = 4071327.151818;

	my_waypoints_list[72].x = 315458.030884;
        my_waypoints_list[72].y = 4071322.411028;

	my_waypoints_list[73].x = 315451.055366;
        my_waypoints_list[73].y = 4071317.680688;

	my_waypoints_list[74].x = 315432.141476;
        my_waypoints_list[74].y = 4071304.072740;

	my_waypoints_list[75].x = 315414.120699;
        my_waypoints_list[75].y = 4071291.321328;

	my_waypoints_list[76].x = 315401.679813;
        my_waypoints_list[76].y = 4071282.329194;

	my_waypoints_list[77].x = 315392.798318;
        my_waypoints_list[77].y = 4071276.138337;

	my_waypoints_list[78].x = 315387.856364;
        my_waypoints_list[78].y = 4071272.990870;  // 318

	my_waypoints_list[79].x = 315385.796675;
        my_waypoints_list[79].y = 4071270.158242;

	my_waypoints_list[80].x = 315386.074938;
        my_waypoints_list[80].y = 4071265.526325;

	my_waypoints_list[81].x = 315388.943717;
        my_waypoints_list[81].y = 4071259.214869;

	my_waypoints_list[82].x = 315398.464068;
        my_waypoints_list[82].y = 4071242.136842;

	my_waypoints_list[83].x = 315410.259674;
        my_waypoints_list[83].y = 4071220.260126;

	my_waypoints_list[84].x = 315413.615715;
        my_waypoints_list[84].y = 4071219.314764;

	my_waypoints_list[85].x = 315418.367382;
        my_waypoints_list[85].y = 4071219.340466;

	my_waypoints_list[86].x = 315425.957324;
        my_waypoints_list[86].y = 4071223.557847; // 326 final_goal

        set_delivery_id = 37;
        wp_finish_id = 86;
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
////////////////// 42 -> 33, 61 -> 52 ////////////////////////////

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
        my_waypoints_list[52].y = 4071248.813844;  // 61 guest

////////////////// 327 -> 53, 337 -> 63 ////////////////////////////  back_init_place

        my_waypoints_list[53].x = 315719.106437;
        my_waypoints_list[53].y = 4071245.311510;

	my_waypoints_list[54].x = 315723.836774;
        my_waypoints_list[54].y = 4071238.336002;

	my_waypoints_list[55].x = 315728.077662;
        my_waypoints_list[55].y = 4071231.870844;

	my_waypoints_list[56].x = 315729.275162;
        my_waypoints_list[56].y = 4071229.345220;

	my_waypoints_list[57].x = 315729.215075;
        my_waypoints_list[57].y = 4071226.470797;

	my_waypoints_list[58].x = 315726.259666;
        my_waypoints_list[58].y = 4071222.656660;

	my_waypoints_list[59].x = 315710.939151;
        my_waypoints_list[59].y = 4071213.474671;

	my_waypoints_list[60].x = 315695.761898;
        my_waypoints_list[60].y = 4071205.164892;

	my_waypoints_list[61].x = 315682.503680;
        my_waypoints_list[61].y = 4071198.940500;

	my_waypoints_list[62].x = 315672.911535;
        my_waypoints_list[62].y = 4071194.639947;

	my_waypoints_list[63].x = 315669.164898;
        my_waypoints_list[63].y = 4071194.843295;
	
////////////////// 273 -> 64, 326 -> 117 ////////////////////////////

	my_waypoints_list[64].x = 315667.183586;
        my_waypoints_list[64].y = 4071195.759919;  // 273

	my_waypoints_list[65].x = 315662.705808;
        my_waypoints_list[65].y = 4071202.855177;

	my_waypoints_list[66].x = 315657.996368;
        my_waypoints_list[66].y = 4071210.830486;

	my_waypoints_list[67].x = 315652.935515;
        my_waypoints_list[67].y = 4071219.938408;

	my_waypoints_list[68].x = 315650.287951;
        my_waypoints_list[68].y = 4071224.869906;  // 277

	my_waypoints_list[69].x = 315638.697896;
        my_waypoints_list[69].y = 4071244.616800;

	my_waypoints_list[70].x = 315631.960544;
        my_waypoints_list[70].y = 4071257.260597;  

	my_waypoints_list[71].x = 315629.302530;
        my_waypoints_list[71].y = 4071261.692196;  // 280

	my_waypoints_list[72].x = 315618.923040;
        my_waypoints_list[72].y = 4071279.538344;

	my_waypoints_list[73].x = 315610.488289;
        my_waypoints_list[73].y = 4071294.718218;

	my_waypoints_list[74].x = 315606.398501;
        my_waypoints_list[74].y = 4071302.430518;  // 283

	my_waypoints_list[75].x = 315603.865463;
        my_waypoints_list[75].y = 4071306.859506;

	my_waypoints_list[76].x = 315601.439113;
        my_waypoints_list[76].y = 4071310.411056;

	my_waypoints_list[77].x = 315598.520700;
        my_waypoints_list[77].y = 4071314.347981;

	my_waypoints_list[78].x = 315595.206463;
        my_waypoints_list[78].y = 4071317.292944;

	my_waypoints_list[79].x = 315591.012175;
        my_waypoints_list[79].y = 4071320.006245;

	my_waypoints_list[80].x = 315587.421862;
        my_waypoints_list[80].y = 4071321.706683;

	my_waypoints_list[81].x = 315582.946274;
        my_waypoints_list[81].y = 4071322.925508;

	my_waypoints_list[82].x = 315579.957323;
        my_waypoints_list[82].y = 4071323.488108;

	my_waypoints_list[83].x = 315574.336060;
        my_waypoints_list[83].y = 4071323.730646;

	my_waypoints_list[84].x = 315569.693696;
        my_waypoints_list[84].y = 4071322.952484;

	my_waypoints_list[85].x = 315564.413395;
        my_waypoints_list[85].y = 4071321.562509;

	my_waypoints_list[86].x = 315558.739881;
        my_waypoints_list[86].y = 4071319.305546;

	my_waypoints_list[87].x = 315548.642603;
        my_waypoints_list[87].y = 4071314.765495;

	my_waypoints_list[88].x = 315539.433213;
        my_waypoints_list[88].y = 4071310.832032;

	my_waypoints_list[89].x = 315530.596135;
        my_waypoints_list[89].y = 4071306.765755;

	my_waypoints_list[90].x = 315525.084170;
        my_waypoints_list[90].y = 4071306.255830;

	my_waypoints_list[91].x = 315519.478580;
        my_waypoints_list[91].y = 4071307.248218;
	
	my_waypoints_list[92].x = 315516.000179;
        my_waypoints_list[92].y = 4071308.321168;

	my_waypoints_list[93].x = 315511.298152;
        my_waypoints_list[93].y = 4071310.669994;

	my_waypoints_list[94].x = 315507.247125;
        my_waypoints_list[94].y = 4071314.255509;

	my_waypoints_list[95].x = 315500.303809;
        my_waypoints_list[95].y = 4071323.027701;

	my_waypoints_list[96].x = 315497.273483;
        my_waypoints_list[96].y = 4071327.592116;

	my_waypoints_list[97].x = 315493.961856;
        my_waypoints_list[97].y = 4071330.662055;

	my_waypoints_list[98].x = 315489.238929;
        my_waypoints_list[98].y = 4071332.011082;

	my_waypoints_list[99].x = 315483.240126;
        my_waypoints_list[99].y = 4071332.136482;

	my_waypoints_list[100].x = 315479.110723;
        my_waypoints_list[100].y = 4071331.972745;

	my_waypoints_list[101].x = 315474.215795;
        my_waypoints_list[101].y = 4071331.074833;

	my_waypoints_list[102].x = 315464.506501;
        my_waypoints_list[102].y = 4071327.151818;

	my_waypoints_list[103].x = 315458.030884;
        my_waypoints_list[103].y = 4071322.411028;

	my_waypoints_list[104].x = 315451.055366;
        my_waypoints_list[104].y = 4071317.680688;

	my_waypoints_list[105].x = 315432.141476;
        my_waypoints_list[105].y = 4071304.072740;

	my_waypoints_list[106].x = 315414.120699;
        my_waypoints_list[106].y = 4071291.321328;

	my_waypoints_list[107].x = 315401.679813;
        my_waypoints_list[107].y = 4071282.329194;

	my_waypoints_list[108].x = 315392.798318;
        my_waypoints_list[108].y = 4071276.138337;

	my_waypoints_list[109].x = 315387.856364;
        my_waypoints_list[109].y = 4071272.990870;  // 318

	my_waypoints_list[110].x = 315385.796675;
        my_waypoints_list[110].y = 4071270.158242;

	my_waypoints_list[111].x = 315386.074938;
        my_waypoints_list[111].y = 4071265.526325;

	my_waypoints_list[112].x = 315388.943717;
        my_waypoints_list[112].y = 4071259.214869;

	my_waypoints_list[113].x = 315398.464068;
        my_waypoints_list[113].y = 4071242.136842;

	my_waypoints_list[114].x = 315410.259674;
        my_waypoints_list[114].y = 4071220.260126;

	my_waypoints_list[115].x = 315413.615715;
        my_waypoints_list[115].y = 4071219.314764;

	my_waypoints_list[116].x = 315418.367382;
        my_waypoints_list[116].y = 4071219.340466;

	my_waypoints_list[117].x = 315425.957324;
        my_waypoints_list[117].y = 4071223.557847; // 326 final_goal


        set_delivery_id = 52;
        wp_finish_id = 117;
}

void Hak_Ye_Gwan(void)
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
////////////////// 42 -> 33, 49 -> 40 ////////////////////////////

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
////////////////// 219 -> 41, 227 -> 49 ////////////////////////////
	
        my_waypoints_list[41].x = 315681.306880;
        my_waypoints_list[41].y = 4071285.235925;

	my_waypoints_list[42].x = 315679.661730;
        my_waypoints_list[42].y = 4071290.271499;

	my_waypoints_list[43].x = 315678.365380;
        my_waypoints_list[43].y = 4071294.049485;

	my_waypoints_list[44].x = 315677.087318;
        my_waypoints_list[44].y = 4071298.702296;

	my_waypoints_list[45].x = 315675.689506;
        my_waypoints_list[45].y = 4071303.607670;

	my_waypoints_list[46].x = 315675.254918;
        my_waypoints_list[46].y = 4071306.742494;

	my_waypoints_list[47].x = 315673.174756;
        my_waypoints_list[47].y = 4071308.911481;

	my_waypoints_list[48].x = 315665.280858;
        my_waypoints_list[48].y = 4071314.077680;

	my_waypoints_list[49].x = 315650.873862;
        my_waypoints_list[49].y = 4071336.634114; // 227 guest
	
////////////////// 361 -> 50, 364 -> 53 //////////////////////////// back_init_place

	my_waypoints_list[50].x = 315638.445568;
        my_waypoints_list[50].y = 4071358.148951;  // 361

	my_waypoints_list[51].x = 315640.143393;
        my_waypoints_list[51].y = 4071361.614288;

	my_waypoints_list[52].x = 315654.682718;
        my_waypoints_list[52].y = 4071369.312246;

	my_waypoints_list[53].x = 315658.411067;
        my_waypoints_list[53].y = 4071368.234070;  // 364
////////////////// 339 -> 54, 356 -> 71 //////////////////////////// 
	my_waypoints_list[54].x = 315662.209528;
        my_waypoints_list[54].y = 4071364.528807;  // 339

	my_waypoints_list[55].x = 315665.262938;
        my_waypoints_list[55].y = 4071355.087759;

	my_waypoints_list[56].x = 315668.540173;
        my_waypoints_list[56].y = 4071344.391735;

	my_waypoints_list[57].x = 315674.542919;
        my_waypoints_list[57].y = 4071326.512051;

	my_waypoints_list[58].x = 315674.865594;
        my_waypoints_list[58].y = 4071324.004715;

	my_waypoints_list[59].x = 315673.177794;
        my_waypoints_list[59].y = 4071315.037867;

	my_waypoints_list[60].x = 315672.700981;
        my_waypoints_list[60].y = 4071310.171681;

	my_waypoints_list[61].x = 315672.734519;
        my_waypoints_list[61].y = 4071305.794944;  // 346

	my_waypoints_list[62].x = 315673.403381;
        my_waypoints_list[62].y = 4071301.905045;

	my_waypoints_list[63].x = 315674.428881;
        my_waypoints_list[63].y = 4071297.132484;

	my_waypoints_list[64].x = 315675.459606;
        my_waypoints_list[64].y = 4071292.609873;

	my_waypoints_list[65].x = 315677.500581;
        my_waypoints_list[65].y = 4071288.566261;  // 350

	my_waypoints_list[66].x = 315676.911857;
        my_waypoints_list[66].y = 4071284.327562;

	my_waypoints_list[67].x = 315674.958858;
        my_waypoints_list[67].y = 4071280.617501;

	my_waypoints_list[68].x = 315661.156324;
        my_waypoints_list[68].y = 4071272.278988;

	my_waypoints_list[69].x = 315646.997153;
        my_waypoints_list[69].y = 4071264.823137;

	my_waypoints_list[70].x = 315637.141993;
        my_waypoints_list[70].y = 4071259.902935; // 355

	my_waypoints_list[71].x = 315631.398368;
        my_waypoints_list[71].y = 4071260.273059; // 356
	
////////////////// 280 -> 72, 326 -> 118 //////////////////////////// 

	my_waypoints_list[72].x = 315629.302530;
        my_waypoints_list[72].y = 4071261.692196;  // 280

	my_waypoints_list[73].x = 315618.923040;
        my_waypoints_list[73].y = 4071279.538344;

	my_waypoints_list[74].x = 315610.488289;
        my_waypoints_list[74].y = 4071294.718218;

	my_waypoints_list[75].x = 315606.398501;
        my_waypoints_list[75].y = 4071302.430518;  // 283

	my_waypoints_list[76].x = 315603.865463;
        my_waypoints_list[76].y = 4071306.859506;

	my_waypoints_list[77].x = 315601.439113;
        my_waypoints_list[77].y = 4071310.411056;

	my_waypoints_list[78].x = 315598.520700;
        my_waypoints_list[78].y = 4071314.347981;

	my_waypoints_list[79].x = 315595.206463;
        my_waypoints_list[79].y = 4071317.292944;

	my_waypoints_list[80].x = 315591.012175;
        my_waypoints_list[80].y = 4071320.006245;

	my_waypoints_list[81].x = 315587.421862;
        my_waypoints_list[81].y = 4071321.706683;

	my_waypoints_list[82].x = 315582.946274;
        my_waypoints_list[82].y = 4071322.925508;

	my_waypoints_list[83].x = 315579.957323;
        my_waypoints_list[83].y = 4071323.488108;

	my_waypoints_list[84].x = 315574.336060;
        my_waypoints_list[84].y = 4071323.730646;

	my_waypoints_list[85].x = 315569.693696;
        my_waypoints_list[85].y = 4071322.952484;

	my_waypoints_list[86].x = 315564.413395;
        my_waypoints_list[86].y = 4071321.562509;

	my_waypoints_list[87].x = 315558.739881;
        my_waypoints_list[87].y = 4071319.305546;

	my_waypoints_list[88].x = 315548.642603;
        my_waypoints_list[88].y = 4071314.765495;

	my_waypoints_list[89].x = 315539.433213;
        my_waypoints_list[89].y = 4071310.832032;

	my_waypoints_list[90].x = 315530.596135;
        my_waypoints_list[90].y = 4071306.765755;

	my_waypoints_list[91].x = 315525.084170;
        my_waypoints_list[91].y = 4071306.255830;

	my_waypoints_list[92].x = 315519.478580;
        my_waypoints_list[92].y = 4071307.248218;
	
	my_waypoints_list[93].x = 315516.000179;
        my_waypoints_list[93].y = 4071308.321168;

	my_waypoints_list[94].x = 315511.298152;
        my_waypoints_list[94].y = 4071310.669994;

	my_waypoints_list[95].x = 315507.247125;
        my_waypoints_list[95].y = 4071314.255509;

	my_waypoints_list[96].x = 315500.303809;
        my_waypoints_list[96].y = 4071323.027701;

	my_waypoints_list[97].x = 315497.273483;
        my_waypoints_list[97].y = 4071327.592116;

	my_waypoints_list[98].x = 315493.961856;
        my_waypoints_list[98].y = 4071330.662055;

	my_waypoints_list[99].x = 315489.238929;
        my_waypoints_list[99].y = 4071332.011082;

	my_waypoints_list[100].x = 315483.240126;
        my_waypoints_list[100].y = 4071332.136482;

	my_waypoints_list[101].x = 315479.110723;
        my_waypoints_list[101].y = 4071331.972745;

	my_waypoints_list[102].x = 315474.215795;
        my_waypoints_list[102].y = 4071331.074833;

	my_waypoints_list[103].x = 315464.506501;
        my_waypoints_list[103].y = 4071327.151818;

	my_waypoints_list[104].x = 315458.030884;
        my_waypoints_list[104].y = 4071322.411028;

	my_waypoints_list[105].x = 315451.055366;
        my_waypoints_list[105].y = 4071317.680688;

	my_waypoints_list[106].x = 315432.141476;
        my_waypoints_list[106].y = 4071304.072740;

	my_waypoints_list[107].x = 315414.120699;
        my_waypoints_list[107].y = 4071291.321328;

	my_waypoints_list[108].x = 315401.679813;
        my_waypoints_list[108].y = 4071282.329194;

	my_waypoints_list[109].x = 315392.798318;
        my_waypoints_list[109].y = 4071276.138337;

	my_waypoints_list[110].x = 315387.856364;
        my_waypoints_list[110].y = 4071272.990870;  // 318

	my_waypoints_list[111].x = 315385.796675;
        my_waypoints_list[111].y = 4071270.158242;

	my_waypoints_list[112].x = 315386.074938;
        my_waypoints_list[112].y = 4071265.526325;

	my_waypoints_list[113].x = 315388.943717;
        my_waypoints_list[113].y = 4071259.214869;

	my_waypoints_list[114].x = 315398.464068;
        my_waypoints_list[114].y = 4071242.136842;

	my_waypoints_list[115].x = 315410.259674;
        my_waypoints_list[115].y = 4071220.260126;

	my_waypoints_list[116].x = 315413.615715;
        my_waypoints_list[116].y = 4071219.314764;

	my_waypoints_list[117].x = 315418.367382;
        my_waypoints_list[117].y = 4071219.340466;

	my_waypoints_list[118].x = 315425.957324;
        my_waypoints_list[118].y = 4071223.557847; // 326 final_goal
	

        set_delivery_id = 49;
        wp_finish_id = 118;
}

void Regional_Innovation_Gwan(void)
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
////////////////// 232 -> 11, 244 -> 23 ////////////////////////////
	
        my_waypoints_list[11].x = 315503.775650;
        my_waypoints_list[11].y = 4071339.584071;

	my_waypoints_list[12].x = 315509.988254;
        my_waypoints_list[12].y = 4071343.705230;

	my_waypoints_list[13].x = 315515.815482;
        my_waypoints_list[13].y = 4071347.334286;

	my_waypoints_list[14].x = 315526.808487;
        my_waypoints_list[14].y = 4071352.855848;

	my_waypoints_list[15].x = 315537.322493;
        my_waypoints_list[15].y = 4071359.387661;

	my_waypoints_list[16].x = 315540.254821;
        my_waypoints_list[16].y = 4071368.078438;

	my_waypoints_list[17].x = 315545.100963;
        my_waypoints_list[17].y = 4071378.604653;

	my_waypoints_list[18].x = 315556.614768;
        my_waypoints_list[18].y = 4071385.115564;

	my_waypoints_list[19].x = 315567.836823;
        my_waypoints_list[19].y = 4071389.632098;
	
	my_waypoints_list[20].x = 315575.405863;
        my_waypoints_list[20].y = 4071392.849671;

	my_waypoints_list[21].x = 315586.182880;
        my_waypoints_list[21].y = 4071400.001131;

	my_waypoints_list[22].x = 315606.459318;
        my_waypoints_list[22].y = 4071412.924890;

	my_waypoints_list[23].x = 315610.819487;
        my_waypoints_list[23].y = 4071412.239134;  // 244 guest
	
////////////////// 245 -> 24, 251 -> 30 //////////////////////////// back_init_place
	my_waypoints_list[24].x = 315624.991728;
        my_waypoints_list[24].y = 4071420.319852;

	my_waypoints_list[25].x = 315628.626453;
        my_waypoints_list[25].y = 4071420.743988;

	my_waypoints_list[26].x = 315632.094402;
        my_waypoints_list[26].y = 4071419.171136;

	my_waypoints_list[27].x = 315640.271357;
        my_waypoints_list[27].y = 4071403.621559;

	my_waypoints_list[28].x = 315650.510186;
        my_waypoints_list[28].y = 4071385.028170;

	my_waypoints_list[29].x = 315654.454522;
        my_waypoints_list[29].y = 4071382.320094;

	my_waypoints_list[30].x = 315657.573245;
        my_waypoints_list[30].y = 4071376.003419;  // 251
	
////////////////// 338 -> 31, 356 -> 49 //////////////////////////// 
	my_waypoints_list[31].x = 315659.838043;
        my_waypoints_list[31].y = 4071370.704832;  // 338

	my_waypoints_list[32].x = 315662.209528;
        my_waypoints_list[32].y = 4071364.528807;  // 339

	my_waypoints_list[33].x = 315665.262938;
        my_waypoints_list[33].y = 4071355.087759;

	my_waypoints_list[34].x = 315668.540173;
        my_waypoints_list[34].y = 4071344.391735;

	my_waypoints_list[35].x = 315674.542919;
        my_waypoints_list[35].y = 4071326.512051;

	my_waypoints_list[36].x = 315674.865594;
        my_waypoints_list[36].y = 4071324.004715;

	my_waypoints_list[37].x = 315673.177794;
        my_waypoints_list[37].y = 4071315.037867;

	my_waypoints_list[38].x = 315672.700981;
        my_waypoints_list[38].y = 4071310.171681;

	my_waypoints_list[39].x = 315672.734519;
        my_waypoints_list[39].y = 4071305.794944;  // 346

	my_waypoints_list[40].x = 315673.403381;
        my_waypoints_list[40].y = 4071301.905045;

	my_waypoints_list[41].x = 315674.428881;
        my_waypoints_list[41].y = 4071297.132484;

	my_waypoints_list[42].x = 315675.459606;
        my_waypoints_list[42].y = 4071292.609873;

	my_waypoints_list[43].x = 315677.500581;
        my_waypoints_list[43].y = 4071288.566261;  // 350

	my_waypoints_list[44].x = 315676.911857;
        my_waypoints_list[44].y = 4071284.327562;

	my_waypoints_list[45].x = 315674.958858;
        my_waypoints_list[45].y = 4071280.617501;

	my_waypoints_list[46].x = 315661.156324;
        my_waypoints_list[46].y = 4071272.278988;

	my_waypoints_list[47].x = 315646.997153;
        my_waypoints_list[47].y = 4071264.823137;

	my_waypoints_list[48].x = 315637.141993;
        my_waypoints_list[48].y = 4071259.902935; // 355

	my_waypoints_list[49].x = 315631.398368;
        my_waypoints_list[49].y = 4071260.273059; // 356
	
////////////////// 280 -> 50, 326 -> 96 //////////////////////////// 

	my_waypoints_list[50].x = 315629.302530;
        my_waypoints_list[50].y = 4071261.692196;  // 280

	my_waypoints_list[51].x = 315618.923040;
        my_waypoints_list[51].y = 4071279.538344;

	my_waypoints_list[52].x = 315610.488289;
        my_waypoints_list[52].y = 4071294.718218;

	my_waypoints_list[53].x = 315606.398501;
        my_waypoints_list[53].y = 4071302.430518;  // 283

	my_waypoints_list[54].x = 315603.865463;
        my_waypoints_list[54].y = 4071306.859506;

	my_waypoints_list[55].x = 315601.439113;
        my_waypoints_list[55].y = 4071310.411056;

	my_waypoints_list[56].x = 315598.520700;
        my_waypoints_list[56].y = 4071314.347981;

	my_waypoints_list[57].x = 315595.206463;
        my_waypoints_list[57].y = 4071317.292944;

	my_waypoints_list[58].x = 315591.012175;
        my_waypoints_list[58].y = 4071320.006245;

	my_waypoints_list[59].x = 315587.421862;
        my_waypoints_list[59].y = 4071321.706683;

	my_waypoints_list[60].x = 315582.946274;
        my_waypoints_list[60].y = 4071322.925508;

	my_waypoints_list[61].x = 315579.957323;
        my_waypoints_list[61].y = 4071323.488108;

	my_waypoints_list[62].x = 315574.336060;
        my_waypoints_list[62].y = 4071323.730646;

	my_waypoints_list[63].x = 315569.693696;
        my_waypoints_list[63].y = 4071322.952484;

	my_waypoints_list[64].x = 315564.413395;
        my_waypoints_list[64].y = 4071321.562509;

	my_waypoints_list[65].x = 315558.739881;
        my_waypoints_list[65].y = 4071319.305546;

	my_waypoints_list[66].x = 315548.642603;
        my_waypoints_list[66].y = 4071314.765495;

	my_waypoints_list[67].x = 315539.433213;
        my_waypoints_list[67].y = 4071310.832032;

	my_waypoints_list[68].x = 315530.596135;
        my_waypoints_list[68].y = 4071306.765755;

	my_waypoints_list[69].x = 315525.084170;
        my_waypoints_list[69].y = 4071306.255830;

	my_waypoints_list[70].x = 315519.478580;
        my_waypoints_list[70].y = 4071307.248218;
	
	my_waypoints_list[71].x = 315516.000179;
        my_waypoints_list[71].y = 4071308.321168;

	my_waypoints_list[72].x = 315511.298152;
        my_waypoints_list[72].y = 4071310.669994;

	my_waypoints_list[73].x = 315507.247125;
        my_waypoints_list[73].y = 4071314.255509;

	my_waypoints_list[74].x = 315500.303809;
        my_waypoints_list[74].y = 4071323.027701;

	my_waypoints_list[75].x = 315497.273483;
        my_waypoints_list[75].y = 4071327.592116;

	my_waypoints_list[76].x = 315493.961856;
        my_waypoints_list[76].y = 4071330.662055;

	my_waypoints_list[77].x = 315489.238929;
        my_waypoints_list[77].y = 4071332.011082;

	my_waypoints_list[78].x = 315483.240126;
        my_waypoints_list[78].y = 4071332.136482;

	my_waypoints_list[79].x = 315479.110723;
        my_waypoints_list[79].y = 4071331.972745;

	my_waypoints_list[80].x = 315474.215795;
        my_waypoints_list[80].y = 4071331.074833;

	my_waypoints_list[81].x = 315464.506501;
        my_waypoints_list[81].y = 4071327.151818;

	my_waypoints_list[82].x = 315458.030884;
        my_waypoints_list[82].y = 4071322.411028;

	my_waypoints_list[83].x = 315451.055366;
        my_waypoints_list[83].y = 4071317.680688;

	my_waypoints_list[84].x = 315432.141476;
        my_waypoints_list[84].y = 4071304.072740;

	my_waypoints_list[85].x = 315414.120699;
        my_waypoints_list[85].y = 4071291.321328;

	my_waypoints_list[86].x = 315401.679813;
        my_waypoints_list[86].y = 4071282.329194;

	my_waypoints_list[87].x = 315392.798318;
        my_waypoints_list[87].y = 4071276.138337;

	my_waypoints_list[88].x = 315387.856364;
        my_waypoints_list[88].y = 4071272.990870;  // 318

	my_waypoints_list[89].x = 315385.796675;
        my_waypoints_list[89].y = 4071270.158242;

	my_waypoints_list[90].x = 315386.074938;
        my_waypoints_list[90].y = 4071265.526325;

	my_waypoints_list[91].x = 315388.943717;
        my_waypoints_list[91].y = 4071259.214869;

	my_waypoints_list[92].x = 315398.464068;
        my_waypoints_list[92].y = 4071242.136842;

	my_waypoints_list[93].x = 315410.259674;
        my_waypoints_list[93].y = 4071220.260126;

	my_waypoints_list[94].x = 315413.615715;
        my_waypoints_list[94].y = 4071219.314764;

	my_waypoints_list[95].x = 315418.367382;
        my_waypoints_list[95].y = 4071219.340466;

	my_waypoints_list[96].x = 315425.957324;
        my_waypoints_list[96].y = 4071223.557847; // 326 final_goal
	
        set_delivery_id = 23;
        wp_finish_id = 96;
}


void BRIX_Gwan(void)
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
////////////////// 232 -> 11, 244 -> 23 ////////////////////////////
	
        my_waypoints_list[11].x = 315503.775650;
        my_waypoints_list[11].y = 4071339.584071;

	my_waypoints_list[12].x = 315509.988254;
        my_waypoints_list[12].y = 4071343.705230;

	my_waypoints_list[13].x = 315515.815482;
        my_waypoints_list[13].y = 4071347.334286;

	my_waypoints_list[14].x = 315526.808487;
        my_waypoints_list[14].y = 4071352.855848;

	my_waypoints_list[15].x = 315537.322493;
        my_waypoints_list[15].y = 4071359.387661;

	my_waypoints_list[16].x = 315540.254821;
        my_waypoints_list[16].y = 4071368.078438;

	my_waypoints_list[17].x = 315545.100963;
        my_waypoints_list[17].y = 4071378.604653;

	my_waypoints_list[18].x = 315556.614768;
        my_waypoints_list[18].y = 4071385.115564;

	my_waypoints_list[19].x = 315567.836823;
        my_waypoints_list[19].y = 4071389.632098;
	
	my_waypoints_list[20].x = 315575.405863;
        my_waypoints_list[20].y = 4071392.849671;

	my_waypoints_list[21].x = 315586.182880;
        my_waypoints_list[21].y = 4071400.001131;

	my_waypoints_list[22].x = 315606.459318;
        my_waypoints_list[22].y = 4071412.924890;

	my_waypoints_list[23].x = 315610.819487;
        my_waypoints_list[23].y = 4071412.239134;  // 244
////////////////// 245 -> 24, 251 -> 30 ////////////////////////////
	my_waypoints_list[24].x = 315624.991728;
        my_waypoints_list[24].y = 4071420.319852;

	my_waypoints_list[25].x = 315628.626453;
        my_waypoints_list[25].y = 4071420.743988;

	my_waypoints_list[26].x = 315632.094402;
        my_waypoints_list[26].y = 4071419.171136;

	my_waypoints_list[27].x = 315640.271357;
        my_waypoints_list[27].y = 4071403.621559;

	my_waypoints_list[28].x = 315650.510186;
        my_waypoints_list[28].y = 4071385.028170;

	my_waypoints_list[29].x = 315654.454522;
        my_waypoints_list[29].y = 4071382.320094;

	my_waypoints_list[30].x = 315657.573245;
        my_waypoints_list[30].y = 4071376.003419;  // 251
////////////////// 252 -> 31, 253 -> 32 ////////////////////////////

	my_waypoints_list[31].x = 315656.635719;
        my_waypoints_list[31].y = 4071373.022307;

	my_waypoints_list[32].x = 315636.901881;
        my_waypoints_list[32].y = 4071362.057139;  // 253 guest
	
////////////////// 357 -> 33, 360 -> 36 //////////////////////////// back_init_place
	my_waypoints_list[33].x = 315637.427480;
        my_waypoints_list[33].y = 4071357.295027;

	my_waypoints_list[34].x = 315647.520438;
        my_waypoints_list[34].y = 4071337.704452;

	my_waypoints_list[35].x = 315664.335496;
        my_waypoints_list[35].y = 4071310.721643;

	my_waypoints_list[36].x = 315671.037119;
        my_waypoints_list[36].y = 4071308.331019;  // 360
////////////////// 346 -> 37, 356 -> 47 ////////////////////////////
	my_waypoints_list[37].x = 315672.734519;
        my_waypoints_list[37].y = 4071305.794944;  // 346

	my_waypoints_list[38].x = 315673.403381;
        my_waypoints_list[38].y = 4071301.905045;

	my_waypoints_list[39].x = 315674.428881;
        my_waypoints_list[39].y = 4071297.132484;

	my_waypoints_list[40].x = 315675.459606;
        my_waypoints_list[40].y = 4071292.609873;

	my_waypoints_list[41].x = 315677.500581;
        my_waypoints_list[41].y = 4071288.566261;  // 350

	my_waypoints_list[42].x = 315676.911857;
        my_waypoints_list[42].y = 4071284.327562;

	my_waypoints_list[43].x = 315674.958858;
        my_waypoints_list[43].y = 4071280.617501;

	my_waypoints_list[44].x = 315661.156324;
        my_waypoints_list[44].y = 4071272.278988;

	my_waypoints_list[45].x = 315646.997153;
        my_waypoints_list[45].y = 4071264.823137;

	my_waypoints_list[46].x = 315637.141993;
        my_waypoints_list[46].y = 4071259.902935; // 355

	my_waypoints_list[47].x = 315631.398368;
        my_waypoints_list[47].y = 4071260.273059; // 356
	
////////////////// 280 -> 48, 326 -> 94 //////////////////////////// 

	my_waypoints_list[48].x = 315629.302530;
        my_waypoints_list[48].y = 4071261.692196;  // 280

	my_waypoints_list[49].x = 315618.923040;
        my_waypoints_list[49].y = 4071279.538344;

	my_waypoints_list[50].x = 315610.488289;
        my_waypoints_list[50].y = 4071294.718218;

	my_waypoints_list[51].x = 315606.398501;
        my_waypoints_list[51].y = 4071302.430518;  // 283

	my_waypoints_list[52].x = 315603.865463;
        my_waypoints_list[52].y = 4071306.859506;

	my_waypoints_list[53].x = 315601.439113;
        my_waypoints_list[53].y = 4071310.411056;

	my_waypoints_list[54].x = 315598.520700;
        my_waypoints_list[54].y = 4071314.347981;

	my_waypoints_list[55].x = 315595.206463;
        my_waypoints_list[55].y = 4071317.292944;

	my_waypoints_list[56].x = 315591.012175;
        my_waypoints_list[56].y = 4071320.006245;

	my_waypoints_list[57].x = 315587.421862;
        my_waypoints_list[57].y = 4071321.706683;

	my_waypoints_list[58].x = 315582.946274;
        my_waypoints_list[58].y = 4071322.925508;

	my_waypoints_list[59].x = 315579.957323;
        my_waypoints_list[59].y = 4071323.488108;

	my_waypoints_list[60].x = 315574.336060;
        my_waypoints_list[60].y = 4071323.730646;

	my_waypoints_list[61].x = 315569.693696;
        my_waypoints_list[61].y = 4071322.952484;

	my_waypoints_list[62].x = 315564.413395;
        my_waypoints_list[62].y = 4071321.562509;

	my_waypoints_list[63].x = 315558.739881;
        my_waypoints_list[63].y = 4071319.305546;

	my_waypoints_list[64].x = 315548.642603;
        my_waypoints_list[64].y = 4071314.765495;

	my_waypoints_list[65].x = 315539.433213;
        my_waypoints_list[65].y = 4071310.832032;

	my_waypoints_list[66].x = 315530.596135;
        my_waypoints_list[66].y = 4071306.765755;

	my_waypoints_list[67].x = 315525.084170;
        my_waypoints_list[67].y = 4071306.255830;

	my_waypoints_list[68].x = 315519.478580;
        my_waypoints_list[68].y = 4071307.248218;
	
	my_waypoints_list[69].x = 315516.000179;
        my_waypoints_list[69].y = 4071308.321168;

	my_waypoints_list[70].x = 315511.298152;
        my_waypoints_list[70].y = 4071310.669994;

	my_waypoints_list[71].x = 315507.247125;
        my_waypoints_list[71].y = 4071314.255509;

	my_waypoints_list[72].x = 315500.303809;
        my_waypoints_list[72].y = 4071323.027701;

	my_waypoints_list[73].x = 315497.273483;
        my_waypoints_list[73].y = 4071327.592116;

	my_waypoints_list[74].x = 315493.961856;
        my_waypoints_list[74].y = 4071330.662055;

	my_waypoints_list[75].x = 315489.238929;
        my_waypoints_list[75].y = 4071332.011082;

	my_waypoints_list[76].x = 315483.240126;
        my_waypoints_list[76].y = 4071332.136482;

	my_waypoints_list[77].x = 315479.110723;
        my_waypoints_list[77].y = 4071331.972745;

	my_waypoints_list[78].x = 315474.215795;
        my_waypoints_list[78].y = 4071331.074833;

	my_waypoints_list[79].x = 315464.506501;
        my_waypoints_list[79].y = 4071327.151818;

	my_waypoints_list[80].x = 315458.030884;
        my_waypoints_list[80].y = 4071322.411028;

	my_waypoints_list[81].x = 315451.055366;
        my_waypoints_list[81].y = 4071317.680688;

	my_waypoints_list[82].x = 315432.141476;
        my_waypoints_list[82].y = 4071304.072740;

	my_waypoints_list[83].x = 315414.120699;
        my_waypoints_list[83].y = 4071291.321328;

	my_waypoints_list[84].x = 315401.679813;
        my_waypoints_list[84].y = 4071282.329194;

	my_waypoints_list[85].x = 315392.798318;
        my_waypoints_list[85].y = 4071276.138337;

	my_waypoints_list[86].x = 315387.856364;
        my_waypoints_list[86].y = 4071272.990870;  // 318

	my_waypoints_list[87].x = 315385.796675;
        my_waypoints_list[87].y = 4071270.158242;

	my_waypoints_list[88].x = 315386.074938;
        my_waypoints_list[88].y = 4071265.526325;

	my_waypoints_list[89].x = 315388.943717;
        my_waypoints_list[89].y = 4071259.214869;

	my_waypoints_list[90].x = 315398.464068;
        my_waypoints_list[90].y = 4071242.136842;

	my_waypoints_list[91].x = 315410.259674;
        my_waypoints_list[91].y = 4071220.260126;

	my_waypoints_list[92].x = 315413.615715;
        my_waypoints_list[92].y = 4071219.314764;

	my_waypoints_list[93].x = 315418.367382;
        my_waypoints_list[93].y = 4071219.340466;

	my_waypoints_list[94].x = 315425.957324;
        my_waypoints_list[94].y = 4071223.557847; // 326 final_goal

        set_delivery_id = 32;
        wp_finish_id = 94;
}



void San_Hak_Hyeop_Ryeok_Gwan(void)
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
////////////////// 228 -> 11, 231 -> 14 ////////////////////////////
        my_waypoints_list[11].x = 315500.882510;
        my_waypoints_list[11].y = 4071332.767918;

	my_waypoints_list[12].x = 315504.387037;
        my_waypoints_list[12].y = 4071332.944718;

	my_waypoints_list[13].x = 315500.882510;
        my_waypoints_list[13].y = 4071332.767918;

	my_waypoints_list[14].x = 315508.807764;
        my_waypoints_list[14].y = 4071329.101416;  // 231 guest
	
////////////////// 379 -> 15, 399 -> 35 //////////////////////////// back_init_place
	my_waypoints_list[15].x = 315512.905390;
        my_waypoints_list[15].y = 4071321.764037; // 379

	my_waypoints_list[16].x = 315515.485454;
        my_waypoints_list[16].y = 4071319.584598;

	my_waypoints_list[17].x = 315518.333755;
        my_waypoints_list[17].y = 4071318.274760;

	my_waypoints_list[18].x = 315521.562206;
        my_waypoints_list[18].y = 4071317.207035;

	my_waypoints_list[19].x = 315525.061508;
        my_waypoints_list[19].y = 4071317.133884;

	my_waypoints_list[20].x = 315528.930509;
        my_waypoints_list[20].y = 4071316.802947;

	my_waypoints_list[21].x = 315534.077999;
        my_waypoints_list[21].y = 4071317.820609;

	my_waypoints_list[22].x = 315535.463174;
        my_waypoints_list[22].y = 4071318.291772; // 386 u_turn_start

	my_waypoints_list[23].x = 315530.477235;
        my_waypoints_list[23].y = 4071319.021147; // 387 u_turn_end

	my_waypoints_list[24].x = 315526.571658;
        my_waypoints_list[24].y = 4071317.602435;

	my_waypoints_list[25].x = 315521.197731;
        my_waypoints_list[25].y = 4071317.714772;

	my_waypoints_list[26].x = 315516.099879;
        my_waypoints_list[26].y = 4071319.071636;

	my_waypoints_list[27].x = 315512.413327;
        my_waypoints_list[27].y = 4071322.149412;

	my_waypoints_list[28].x = 315509.403901;
        my_waypoints_list[28].y = 4071327.713627;

	my_waypoints_list[29].x = 315504.497575;
        my_waypoints_list[29].y = 4071332.693495;

	my_waypoints_list[30].x = 315500.275923;
        my_waypoints_list[30].y = 4071333.655806;

	my_waypoints_list[31].x = 315490.871442;
        my_waypoints_list[31].y = 4071332.352044;

	my_waypoints_list[32].x = 315485.119977;
        my_waypoints_list[32].y = 4071332.347245;

	my_waypoints_list[33].x = 315479.863186;
        my_waypoints_list[33].y = 4071332.082045;

	my_waypoints_list[34].x = 315473.843482;
        my_waypoints_list[34].y = 4071331.207645;

	my_waypoints_list[35].x = 315469.315642;
        my_waypoints_list[35].y = 4071329.926970;  // 399

////////////////// 122 -> 36 //////////////////////////// 
	
	my_waypoints_list[36].x = 315432.274289; 
        my_waypoints_list[36].y = 4071304.445053;  // 122
	
////////////////// 375 -> 37, 378 -> 40 //////////////////////////// back_init_place
	my_waypoints_list[37].x = 315420.721292; 
        my_waypoints_list[37].y = 4071296.059508; // 375

	my_waypoints_list[38].x = 315411.201859; 
        my_waypoints_list[38].y = 4071289.256839;

	my_waypoints_list[39].x = 315401.679813; 
        my_waypoints_list[39].y = 4071282.329194;

	my_waypoints_list[40].x = 315393.938769; 
        my_waypoints_list[40].y = 4071276.864675; // 378

////////////////// 318 -> 41, 326 -> 49 //////////////////////////// 
	my_waypoints_list[41].x = 315387.856364;
        my_waypoints_list[41].y = 4071272.990870;  // 318

	my_waypoints_list[42].x = 315385.796675;
        my_waypoints_list[42].y = 4071270.158242;

	my_waypoints_list[43].x = 315386.074938;
        my_waypoints_list[43].y = 4071265.526325;

	my_waypoints_list[44].x = 315388.943717;
        my_waypoints_list[44].y = 4071259.214869;

	my_waypoints_list[45].x = 315398.464068;
        my_waypoints_list[45].y = 4071242.136842;

	my_waypoints_list[46].x = 315410.259674;
        my_waypoints_list[46].y = 4071220.260126;

	my_waypoints_list[47].x = 315413.615715;
        my_waypoints_list[47].y = 4071219.314764;

	my_waypoints_list[48].x = 315418.367382;
        my_waypoints_list[48].y = 4071219.340466;

	my_waypoints_list[49].x = 315425.957324;
        my_waypoints_list[49].y = 4071223.557847; // 326 final_goal
	
        set_delivery_id = 14;
        wp_finish_id = 49;

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
        my_waypoints_list[15].y = 4071304.445053;  // 122 guest
	
////////////////// 375 -> 16, 378 -> 19 //////////////////////////// back_init_place
	my_waypoints_list[16].x = 315420.721292; 
        my_waypoints_list[16].y = 4071296.059508;

	my_waypoints_list[17].x = 315411.201859; 
        my_waypoints_list[17].y = 4071289.256839;

	my_waypoints_list[18].x = 315401.679813; 
        my_waypoints_list[18].y = 4071282.329194;

	my_waypoints_list[19].x = 315393.938769; 
        my_waypoints_list[19].y = 4071276.864675;

////////////////// 318 -> 20, 326 -> 28 //////////////////////////// 
	my_waypoints_list[20].x = 315387.856364;
        my_waypoints_list[20].y = 4071272.990870;  // 318

	my_waypoints_list[21].x = 315385.796675;
        my_waypoints_list[21].y = 4071270.158242;

	my_waypoints_list[22].x = 315386.074938;
        my_waypoints_list[22].y = 4071265.526325;

	my_waypoints_list[23].x = 315388.943717;
        my_waypoints_list[23].y = 4071259.214869;

	my_waypoints_list[24].x = 315398.464068;
        my_waypoints_list[24].y = 4071242.136842;

	my_waypoints_list[25].x = 315410.259674;
        my_waypoints_list[25].y = 4071220.260126;

	my_waypoints_list[26].x = 315413.615715;
        my_waypoints_list[26].y = 4071219.314764;

	my_waypoints_list[27].x = 315418.367382;
        my_waypoints_list[27].y = 4071219.340466;

	my_waypoints_list[28].x = 315425.957324;
        my_waypoints_list[28].y = 4071223.557847; // 326 final_goal
	
        set_delivery_id = 15;
        wp_finish_id = 28;
}

void Library(void)
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
        my_waypoints_list[35].y = 4071265.162982;  // 142 guest

        set_delivery_id = 35;
        wp_finish_id = 35;
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
        my_waypoints_list[37].y = 4071223.810442;  // 75 guest

	
	/*
        my_waypoints_list[38].x = 315647.462741; 
        my_waypoints_list[38].y = 4071221.303105;

        my_waypoints_list[39].x = 315646.171192; 
        my_waypoints_list[39].y = 4071219.329630;

        my_waypoints_list[40].x = 315642.757680; 
        my_waypoints_list[40].y = 4071217.525542;  // 78
	*/
	
        set_delivery_id = 37;
        wp_finish_id = 37;


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
        my_waypoints_list[35].y = 4071265.162982;  // 142

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
        my_waypoints_list[79].y = 4071276.714510; // 193 guest

        set_delivery_id = 79;
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
        my_waypoints_list[66].y = 4071202.114007;  // 173 guest

        set_delivery_id = 66;
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
        my_waypoints_list[49].y = 4071205.198516;  // 156 guest

        set_delivery_id = 49;
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
////////////////// 92 -> 51, 114 -> 73 ////////////////////////////
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
        my_waypoints_list[73].y = 4071066.614383;  // 114 guest

////////////////// 254 -> 74, 326 -> 146 //////////////////////////// back_init_place

	my_waypoints_list[74].x = 315586.138713;
        my_waypoints_list[74].y = 4071068.923599;

	my_waypoints_list[75].x = 315589.109796;
        my_waypoints_list[75].y = 4071073.487590;

	my_waypoints_list[76].x = 315596.520314;
        my_waypoints_list[76].y = 4071081.084523;

	my_waypoints_list[77].x = 315601.730498;
        my_waypoints_list[77].y = 4071085.101591;

	my_waypoints_list[78].x = 315608.700781;
        my_waypoints_list[78].y = 4071089.581986;

	my_waypoints_list[79].x = 315620.290334;
        my_waypoints_list[79].y = 4071099.717184;

	my_waypoints_list[80].x = 315636.936780;
        my_waypoints_list[80].y = 4071118.498748;

	my_waypoints_list[81].x = 315647.701143;
        my_waypoints_list[81].y = 4071131.026755;

	my_waypoints_list[82].x = 315654.080513;
        my_waypoints_list[82].y = 4071137.144884;

	my_waypoints_list[83].x = 315660.191645;
        my_waypoints_list[83].y = 4071142.393413;

	my_waypoints_list[84].x = 315666.154291;
        my_waypoints_list[84].y = 4071146.519779;

	my_waypoints_list[85].x = 315671.249949;
        my_waypoints_list[85].y = 4071151.039356;

	my_waypoints_list[86].x = 315679.407704;
        my_waypoints_list[86].y = 4071158.495635;

	my_waypoints_list[87].x = 315682.295612;
        my_waypoints_list[87].y = 4071165.061835;

	my_waypoints_list[88].x = 315682.699695;
        my_waypoints_list[88].y = 4071172.430133;

	my_waypoints_list[89].x = 315682.171480;
        my_waypoints_list[89].y = 4071177.067269;

	my_waypoints_list[90].x = 315679.659340;
        my_waypoints_list[90].y = 4071182.496054;
	
	my_waypoints_list[91].x = 315676.615951;
        my_waypoints_list[91].y = 4071186.435588;  // 271

	my_waypoints_list[92].x = 315672.195225;
        my_waypoints_list[92].y = 4071190.278885;

	my_waypoints_list[93].x = 315667.183586;
        my_waypoints_list[93].y = 4071195.759919;  // 273

	my_waypoints_list[94].x = 315662.705808;
        my_waypoints_list[94].y = 4071202.855177;

	my_waypoints_list[95].x = 315657.996368;
        my_waypoints_list[95].y = 4071210.830486;

	my_waypoints_list[96].x = 315652.935515;
        my_waypoints_list[96].y = 4071219.938408;

	my_waypoints_list[97].x = 315650.287951;
        my_waypoints_list[97].y = 4071224.869906;  // 277

	my_waypoints_list[98].x = 315638.697896;
        my_waypoints_list[98].y = 4071244.616800;

	my_waypoints_list[99].x = 315631.960544;
        my_waypoints_list[99].y = 4071257.260597;  

	my_waypoints_list[100].x = 315629.302530;
        my_waypoints_list[100].y = 4071261.692196;  // 280

	my_waypoints_list[101].x = 315618.923040;
        my_waypoints_list[101].y = 4071279.538344;

	my_waypoints_list[102].x = 315610.488289;
        my_waypoints_list[102].y = 4071294.718218;

	my_waypoints_list[103].x = 315606.398501;
        my_waypoints_list[103].y = 4071302.430518;  // 283

	my_waypoints_list[104].x = 315603.865463;
        my_waypoints_list[104].y = 4071306.859506;

	my_waypoints_list[105].x = 315601.439113;
        my_waypoints_list[105].y = 4071310.411056;

	my_waypoints_list[106].x = 315598.520700;
        my_waypoints_list[106].y = 4071314.347981;

	my_waypoints_list[107].x = 315595.206463;
        my_waypoints_list[107].y = 4071317.292944;

	my_waypoints_list[108].x = 315591.012175;
        my_waypoints_list[108].y = 4071320.006245;

	my_waypoints_list[109].x = 315587.421862;
        my_waypoints_list[109].y = 4071321.706683;

	my_waypoints_list[110].x = 315582.946274;
        my_waypoints_list[110].y = 4071322.925508;

	my_waypoints_list[111].x = 315579.957323;
        my_waypoints_list[111].y = 4071323.488108;

	my_waypoints_list[112].x = 315574.336060;
        my_waypoints_list[112].y = 4071323.730646;

	my_waypoints_list[113].x = 315569.693696;
        my_waypoints_list[113].y = 4071322.952484;

	my_waypoints_list[114].x = 315564.413395;
        my_waypoints_list[114].y = 4071321.562509;

	my_waypoints_list[115].x = 315558.739881;
        my_waypoints_list[115].y = 4071319.305546;

	my_waypoints_list[116].x = 315548.642603;
        my_waypoints_list[116].y = 4071314.765495;

	my_waypoints_list[117].x = 315539.433213;
        my_waypoints_list[117].y = 4071310.832032;

	my_waypoints_list[118].x = 315530.596135;
        my_waypoints_list[118].y = 4071306.765755;

	my_waypoints_list[119].x = 315525.084170;
        my_waypoints_list[119].y = 4071306.255830;

	my_waypoints_list[120].x = 315519.478580;
        my_waypoints_list[120].y = 4071307.248218;
	
	my_waypoints_list[121].x = 315516.000179;
        my_waypoints_list[121].y = 4071308.321168;

	my_waypoints_list[122].x = 315511.298152;
        my_waypoints_list[122].y = 4071310.669994;

	my_waypoints_list[123].x = 315507.247125;
        my_waypoints_list[123].y = 4071314.255509;

	my_waypoints_list[124].x = 315500.303809;
        my_waypoints_list[124].y = 4071323.027701;

	my_waypoints_list[125].x = 315497.273483;
        my_waypoints_list[125].y = 4071327.592116;

	my_waypoints_list[126].x = 315493.961856;
        my_waypoints_list[126].y = 4071330.662055;

	my_waypoints_list[127].x = 315489.238929;
        my_waypoints_list[127].y = 4071332.011082;

	my_waypoints_list[128].x = 315483.240126;
        my_waypoints_list[128].y = 4071332.136482;

	my_waypoints_list[129].x = 315479.110723;
        my_waypoints_list[129].y = 4071331.972745;

	my_waypoints_list[130].x = 315474.215795;
        my_waypoints_list[130].y = 4071331.074833;

	my_waypoints_list[131].x = 315464.506501;
        my_waypoints_list[131].y = 4071327.151818;

	my_waypoints_list[132].x = 315458.030884;
        my_waypoints_list[132].y = 4071322.411028;

	my_waypoints_list[133].x = 315451.055366;
        my_waypoints_list[133].y = 4071317.680688;

	my_waypoints_list[134].x = 315432.141476;
        my_waypoints_list[134].y = 4071304.072740;

	my_waypoints_list[135].x = 315414.120699;
        my_waypoints_list[135].y = 4071291.321328;

	my_waypoints_list[136].x = 315401.679813;
        my_waypoints_list[136].y = 4071282.329194;

	my_waypoints_list[137].x = 315392.798318;
        my_waypoints_list[137].y = 4071276.138337;

	my_waypoints_list[138].x = 315387.856364;
        my_waypoints_list[138].y = 4071272.990870;  // 318

	my_waypoints_list[139].x = 315385.796675;
        my_waypoints_list[139].y = 4071270.158242;

	my_waypoints_list[140].x = 315386.074938;
        my_waypoints_list[140].y = 4071265.526325;

	my_waypoints_list[141].x = 315388.943717;
        my_waypoints_list[141].y = 4071259.214869;

	my_waypoints_list[142].x = 315398.464068;
        my_waypoints_list[142].y = 4071242.136842;

	my_waypoints_list[143].x = 315410.259674;
        my_waypoints_list[143].y = 4071220.260126;

	my_waypoints_list[144].x = 315413.615715;
        my_waypoints_list[144].y = 4071219.314764;

	my_waypoints_list[145].x = 315418.367382;
        my_waypoints_list[145].y = 4071219.340466;

	my_waypoints_list[146].x = 315425.957324;
        my_waypoints_list[146].y = 4071223.557847; // 326 final_goal

	
        set_delivery_id = 73;
        wp_finish_id = 146;
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
        my_waypoints_list[50].y = 4071144.942548;  // 91 guest
	
////////////////// 92 -> 51, 114 -> 73 //////////////////////////// back_init_place
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
        my_waypoints_list[73].y = 4071066.614383;  // 114 

////////////////// 254 -> 74, 326 -> 146 //////////////////////////// 

	my_waypoints_list[74].x = 315586.138713;
        my_waypoints_list[74].y = 4071068.923599;

	my_waypoints_list[75].x = 315589.109796;
        my_waypoints_list[75].y = 4071073.487590;

	my_waypoints_list[76].x = 315596.520314;
        my_waypoints_list[76].y = 4071081.084523;

	my_waypoints_list[77].x = 315601.730498;
        my_waypoints_list[77].y = 4071085.101591;

	my_waypoints_list[78].x = 315608.700781;
        my_waypoints_list[78].y = 4071089.581986;

	my_waypoints_list[79].x = 315620.290334;
        my_waypoints_list[79].y = 4071099.717184;

	my_waypoints_list[80].x = 315636.936780;
        my_waypoints_list[80].y = 4071118.498748;

	my_waypoints_list[81].x = 315647.701143;
        my_waypoints_list[81].y = 4071131.026755;

	my_waypoints_list[82].x = 315654.080513;
        my_waypoints_list[82].y = 4071137.144884;

	my_waypoints_list[83].x = 315660.191645;
        my_waypoints_list[83].y = 4071142.393413;

	my_waypoints_list[84].x = 315666.154291;
        my_waypoints_list[84].y = 4071146.519779;

	my_waypoints_list[85].x = 315671.249949;
        my_waypoints_list[85].y = 4071151.039356;

	my_waypoints_list[86].x = 315679.407704;
        my_waypoints_list[86].y = 4071158.495635;

	my_waypoints_list[87].x = 315682.295612;
        my_waypoints_list[87].y = 4071165.061835;

	my_waypoints_list[88].x = 315682.699695;
        my_waypoints_list[88].y = 4071172.430133;

	my_waypoints_list[89].x = 315682.171480;
        my_waypoints_list[89].y = 4071177.067269;

	my_waypoints_list[90].x = 315679.659340;
        my_waypoints_list[90].y = 4071182.496054;
	
	my_waypoints_list[91].x = 315676.615951;
        my_waypoints_list[91].y = 4071186.435588;  // 271

	my_waypoints_list[92].x = 315672.195225;
        my_waypoints_list[92].y = 4071190.278885;

	my_waypoints_list[93].x = 315667.183586;
        my_waypoints_list[93].y = 4071195.759919;  // 273

	my_waypoints_list[94].x = 315662.705808;
        my_waypoints_list[94].y = 4071202.855177;

	my_waypoints_list[95].x = 315657.996368;
        my_waypoints_list[95].y = 4071210.830486;

	my_waypoints_list[96].x = 315652.935515;
        my_waypoints_list[96].y = 4071219.938408;

	my_waypoints_list[97].x = 315650.287951;
        my_waypoints_list[97].y = 4071224.869906;  // 277

	my_waypoints_list[98].x = 315638.697896;
        my_waypoints_list[98].y = 4071244.616800;

	my_waypoints_list[99].x = 315631.960544;
        my_waypoints_list[99].y = 4071257.260597;  

	my_waypoints_list[100].x = 315629.302530;
        my_waypoints_list[100].y = 4071261.692196;  // 280

	my_waypoints_list[101].x = 315618.923040;
        my_waypoints_list[101].y = 4071279.538344;

	my_waypoints_list[102].x = 315610.488289;
        my_waypoints_list[102].y = 4071294.718218;

	my_waypoints_list[103].x = 315606.398501;
        my_waypoints_list[103].y = 4071302.430518;  // 283

	my_waypoints_list[104].x = 315603.865463;
        my_waypoints_list[104].y = 4071306.859506;

	my_waypoints_list[105].x = 315601.439113;
        my_waypoints_list[105].y = 4071310.411056;

	my_waypoints_list[106].x = 315598.520700;
        my_waypoints_list[106].y = 4071314.347981;

	my_waypoints_list[107].x = 315595.206463;
        my_waypoints_list[107].y = 4071317.292944;

	my_waypoints_list[108].x = 315591.012175;
        my_waypoints_list[108].y = 4071320.006245;

	my_waypoints_list[109].x = 315587.421862;
        my_waypoints_list[109].y = 4071321.706683;

	my_waypoints_list[110].x = 315582.946274;
        my_waypoints_list[110].y = 4071322.925508;

	my_waypoints_list[111].x = 315579.957323;
        my_waypoints_list[111].y = 4071323.488108;

	my_waypoints_list[112].x = 315574.336060;
        my_waypoints_list[112].y = 4071323.730646;

	my_waypoints_list[113].x = 315569.693696;
        my_waypoints_list[113].y = 4071322.952484;

	my_waypoints_list[114].x = 315564.413395;
        my_waypoints_list[114].y = 4071321.562509;

	my_waypoints_list[115].x = 315558.739881;
        my_waypoints_list[115].y = 4071319.305546;

	my_waypoints_list[116].x = 315548.642603;
        my_waypoints_list[116].y = 4071314.765495;

	my_waypoints_list[117].x = 315539.433213;
        my_waypoints_list[117].y = 4071310.832032;

	my_waypoints_list[118].x = 315530.596135;
        my_waypoints_list[118].y = 4071306.765755;

	my_waypoints_list[119].x = 315525.084170;
        my_waypoints_list[119].y = 4071306.255830;

	my_waypoints_list[120].x = 315519.478580;
        my_waypoints_list[120].y = 4071307.248218;
	
	my_waypoints_list[121].x = 315516.000179;
        my_waypoints_list[121].y = 4071308.321168;

	my_waypoints_list[122].x = 315511.298152;
        my_waypoints_list[122].y = 4071310.669994;

	my_waypoints_list[123].x = 315507.247125;
        my_waypoints_list[123].y = 4071314.255509;

	my_waypoints_list[124].x = 315500.303809;
        my_waypoints_list[124].y = 4071323.027701;

	my_waypoints_list[125].x = 315497.273483;
        my_waypoints_list[125].y = 4071327.592116;

	my_waypoints_list[126].x = 315493.961856;
        my_waypoints_list[126].y = 4071330.662055;

	my_waypoints_list[127].x = 315489.238929;
        my_waypoints_list[127].y = 4071332.011082;

	my_waypoints_list[128].x = 315483.240126;
        my_waypoints_list[128].y = 4071332.136482;

	my_waypoints_list[129].x = 315479.110723;
        my_waypoints_list[129].y = 4071331.972745;

	my_waypoints_list[130].x = 315474.215795;
        my_waypoints_list[130].y = 4071331.074833;

	my_waypoints_list[131].x = 315464.506501;
        my_waypoints_list[131].y = 4071327.151818;

	my_waypoints_list[132].x = 315458.030884;
        my_waypoints_list[132].y = 4071322.411028;

	my_waypoints_list[133].x = 315451.055366;
        my_waypoints_list[133].y = 4071317.680688;

	my_waypoints_list[134].x = 315432.141476;
        my_waypoints_list[134].y = 4071304.072740;

	my_waypoints_list[135].x = 315414.120699;
        my_waypoints_list[135].y = 4071291.321328;

	my_waypoints_list[136].x = 315401.679813;
        my_waypoints_list[136].y = 4071282.329194;

	my_waypoints_list[137].x = 315392.798318;
        my_waypoints_list[137].y = 4071276.138337;

	my_waypoints_list[138].x = 315387.856364;
        my_waypoints_list[138].y = 4071272.990870;  // 318

	my_waypoints_list[139].x = 315385.796675;
        my_waypoints_list[139].y = 4071270.158242;

	my_waypoints_list[140].x = 315386.074938;
        my_waypoints_list[140].y = 4071265.526325;

	my_waypoints_list[141].x = 315388.943717;
        my_waypoints_list[141].y = 4071259.214869;

	my_waypoints_list[142].x = 315398.464068;
        my_waypoints_list[142].y = 4071242.136842;

	my_waypoints_list[143].x = 315410.259674;
        my_waypoints_list[143].y = 4071220.260126;

	my_waypoints_list[144].x = 315413.615715;
        my_waypoints_list[144].y = 4071219.314764;

	my_waypoints_list[145].x = 315418.367382;
        my_waypoints_list[145].y = 4071219.340466;

	my_waypoints_list[146].x = 315425.957324;
        my_waypoints_list[146].y = 4071223.557847; // 326 final_goal


        set_delivery_id = 50;
        wp_finish_id = 146;

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
    if(msg->data == "Unitophia" || (msg->data == "Multi_Media_Gwan" || msg->data == "Hak_Ye_Gwan" || msg->data == "BRIX_Gwan" || msg->data == "San_Hak_Hyeop_Ryeok_Gwan" || msg->data == "Gong_Hak_Gwan" || msg->data == "Library" || msg->data == "Antire_preneur_Gwan" || msg->data == "Naturel_Science_Gwan" || msg->data == "Humanities_Social_Science_Gwan" || msg->data == "Main_University" || msg->data == "Global_Village" || msg->data == "Hyang_333") ){


        if(start_command == false) {
            sub_destination = true;
            target_string = msg->data;
        }

    }



}

void detect_name_Callback(const std_msgs::String::ConstPtr& msg)
{
    detect_name = msg->data;

}

void depth_Callback(const std_msgs::Float64::ConstPtr& msg)
{
    depth = msg->data;

}

void wp_go_id_Callback(const std_msgs::Int16::ConstPtr& msg)
{
    wp_go_id = msg->data;

}

void start_joy_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    start_joy = msg->data;

    sub_joy_current_time = ros::Time::now().toSec();
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
    ros::Subscriber wp_go_id_sub  = n.subscribe("/wp_go_id",10,&wp_go_id_Callback);
    ros::Subscriber start_joy_sub  = n.subscribe("/start_joy",10,&start_joy_Callback);

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
    else if( target_string == "Unitophia" ){ Unitophia();  destination_data = "Unitophia"; }
    else if( target_string == "Multi_Media_Gwan" ){ Multi_Media_Gwan();  destination_data = "Multi_Media_Gwan"; }
    else if( target_string == "Hak_Ye_Gwan" ){ Hak_Ye_Gwan();  destination_data = "Hak_Ye_Gwan"; }
    else if( target_string == "BRIX_Gwan" ){ BRIX_Gwan();  destination_data = "BRIX_Gwan"; }
    else if( target_string == "San_Hak_Hyeop_Ryeok_Gwan" ){ San_Hak_Hyeop_Ryeok_Gwan();  destination_data = "San_Hak_Hyeop_Ryeok_Gwan"; }
    else if( target_string == "Gong_Hak_Gwan" ){ Gong_Hak_Gwan();  destination_data = "Gong_Hak_Gwan"; }
    else if( target_string == "Library" ){ Library();  destination_data = "Library"; }
    else if( target_string == "Antire_preneur_Gwan" ){ Antire_preneur_Gwan();  destination_data = "Antire_preneur_Gwan"; }
    else if( target_string == "Naturel_Science_Gwan" ){ Naturel_Science_Gwan();  destination_data = "Naturel_Science_Gwan"; }
    else if( target_string == "Humanities_Social_Science_Gwan" ){ Humanities_Social_Science_Gwan();  destination_data = "Humanities_Social_Science_Gwan"; }
    else if( target_string == "Main_University" ){ Main_University();  destination_data = "Main_University"; }
    else if( target_string == "Global_Village" ){ Global_Village();  destination_data = "Global_Village"; }
    else if( target_string == "Hyang_333" ){ Hyang_333();   destination_data = "Hyang_333"; }
    else{ Multi_Media_Gwan();  destination_data = "Unitophia"; }

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
                    received_product = false;
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
                ROS_INFO("WP Mission Completed");
                
                if(target_string != destination_data){

                    wp_go_id = 0;

                    if(target_string == "init"){ init_waypoint(); } ////////////////////////////////////////////////////////////////////////////////
                    else if( target_string == "Unitophia" ){ Unitophia();  destination_data = "Unitophia"; }
                    else if( target_string == "Multi_Media_Gwan" ){ Multi_Media_Gwan();  destination_data = "Multi_Media_Gwan"; }
                    else if( target_string == "Hak_Ye_Gwan" ){ Hak_Ye_Gwan();  destination_data = "Hak_Ye_Gwan"; }
                    else if( target_string == "BRIX_Gwan" ){ BRIX_Gwan();  destination_data = "BRIX_Gwan"; }
                    else if( target_string == "San_Hak_Hyeop_Ryeok_Gwan" ){ San_Hak_Hyeop_Ryeok_Gwan();  destination_data = "San_Hak_Hyeop_Ryeok_Gwan"; }
                    else if( target_string == "Gong_Hak_Gwan" ){ Gong_Hak_Gwan();  destination_data = "Gong_Hak_Gwan"; }
                    else if( target_string == "Library" ){ Library();  destination_data = "Library"; }
                    else if( target_string == "Antire_preneur_Gwan" ){ Antire_preneur_Gwan();  destination_data = "Antire_preneur_Gwan"; }
                    else if( target_string == "Naturel_Science_Gwan" ){ Naturel_Science_Gwan();  destination_data = "Naturel_Science_Gwan"; }
                    else if( target_string == "Humanities_Social_Science_Gwan" ){ Humanities_Social_Science_Gwan();  destination_data = "Humanities_Social_Science_Gwan"; }
                    else if( target_string == "Main_University" ){ Main_University();  destination_data = "Main_University"; }
                    else if( target_string == "Global_Village" ){ Global_Village();  destination_data = "Global_Village"; }
                    else if( target_string == "Hyang_333" ){ Hyang_333();   destination_data = "Hyang_333"; }
                    else{ Multi_Media_Gwan();  destination_data = "Unitophia"; }

                }

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
        ROS_INFO("current_Destination : %s \n", destination_data.c_str() );

        if( count>=2 && (start_joy == false) ) {

            SteerAngle_pub.publish(s_angle);
            car_speed_pub.publish(c_speed);
        }


        if(ros::Time::now().toSec() - sub_joy_current_time > 1){ start_joy = false; }


        avoid_function_start = false; //////////

        loop_rate.sleep();
    ros::spinOnce();
    ++count;

  }
    return 0;
}
