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
bool Received_product = false; ////////////
bool lidar_object_detect = false; ////////////
bool avoid_function_start = false; ////////////
double avoid_heading_angle_left = 0; //////////////
double avoid_heading_angle_right = 0; //////////////
int wp_go_id = 0;
int wp_finish_id = 0;
double roll,pitch,yaw;
bool start_command = false; ////////////emergency_stop
bool emergency_stop = false; ////////////
double depth; /////////////




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
        my_waypoints_list[6].y = 4071215.08;

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
        no_waypoints = 3;
        wp_finish_id = 6;
}

void Unitophia(void)
{	
        my_waypoints_list[0].x = 315430.02;
        my_waypoints_list[0].y = 4071223.10;

        my_waypoints_list[1].x = 315433.70;
        my_waypoints_list[1].y = 4071225.60;

        my_waypoints_list[2].x = 315437.41;
        my_waypoints_list[2].y = 4071227.86;

        my_waypoints_list[3].x = 315441.48;
        my_waypoints_list[3].y = 4071229.93;

        my_waypoints_list[4].x = 315445.73;
        my_waypoints_list[4].y = 4071232.85

        my_waypoints_list[5].x = 315449.23;
        my_waypoints_list[5].y = 4071235.14;

        my_waypoints_list[6].x = 315453.18;
        my_waypoints_list[6].y = 4071238.04;

        my_waypoints_list[7].x = 315457.60;
        my_waypoints_list[7].y = 4071240.55;

        my_waypoints_list[8].x = 315462.17;
        my_waypoints_list[8].y = 4071242.75;

        my_waypoints_list[9].x = 315466.18;
        my_waypoints_list[9].y = 4071245.43;

        my_waypoints_list[5].x = 315449.23;
        my_waypoints_list[5].y = 4071235.14;

        my_waypoints_list[5].x = 315449.23;
        my_waypoints_list[5].y = 4071235.14;

        //set_delivery_id = 7;
        //no_waypoints = 3;
       // wp_finish_id = 6;
}

void Hak_Ye_Gwan(void)
{

}

void BRIX_Gwan(void)
{

}

void San_Hak_Hyeop_Ryeok_Gwan(void)
{

}

void Gong_Hak_Gwan(void)
{

}

void Library(void)
{

}

void Antire_preneur_Gwan(void)
{

}

void Han_maru(void)
{

}

void Medical_Science_Gwan(void)
{

}

void Naturel_Science_Gwan(void)
{

}

void Humanities_Social_Science_Gwan(void)
{

}

void Main_University(void)
{

}

void Media_Laps_Gwan(void)
{

}

void Global_Village(void)
{

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
    Received_product = msg->data;
}

void Receive_Product_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    Received_product = msg->data;
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

void destination_Callback(const std_msgs::String::ConstPtr& msg)
{
    destination_data = msg->data;
    
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
    ros::Subscriber destination_sub  = n.subscribe("/destination",10,&destination_Callback);
    ros::Subscriber detect_name_sub  = n.subscribe("/detect_name",10,&detect_name_Callback);
    ros::Subscriber depth_sub  = n.subscribe("/depth",10,&depth_Callback);

    ros::Publisher SteerAngle_pub            = n.advertise<std_msgs::Int16>("Car_Control_cmd/SteerAngle_Int16", 10);
    ros::Publisher car_speed_pub             = n.advertise<std_msgs::Int16>("Car_Control_cmd/Speed_Int16", 10);
    ros::Publisher target_id_pub             = n.advertise<std_msgs::Int16>("target_id",2);
    ros::Publisher target_pos_pub            = n.advertise<geometry_msgs::Pose2D>("/pose_goal", 10);
  
    ros::Rate loop_rate(5);  // 10 

    long count = 0;
    double pos_error_x = 0.0;
    double pos_error_y = 0.0;

    double waypoint_distance = 0.0;
    double waypoint_gap_distance = 0.0;

    geometry_msgs::Pose2D pose_goal;  
    
    if(destination_data.c_str() == "init"){ init_waypoint(); } ////////////////////////////////////////////////////////////////////////////////
    else if( destination_data.c_str() == "unitophia" ) { Unitophia(); }
    else { init_waypoint(); }
    
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
           
                    if(wp_go_id == set_delivery_id) {
	                if(Received_product == false) { 
                            c_speed.data = 0; 
                            s_angle.data = 0; 
                        }
                        else if(Received_product == true) {
                            c_speed.data = 100;
                            s_angle.data = car_angle;
                            wp_go_id++;
                        }             
                    }
                    else if(wp_go_id != set_delivery_id) { wp_go_id++; } 
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

                if(emergency_stop == true || ((detect_name.c_str() == "person") && (depth < 2.3)) ){
                    c_speed.data = 0;
                    s_angle.data = 0;
                }

	        SteerAngle_pub.publish(s_angle);
	        car_speed_pub.publish(c_speed);
                ros::Duration(0.5).sleep();      
            }
	    
	    if(wp_go_id >= wp_finish_id) {
                         start_command = false;
			 c_speed.data = 0;
                         s_angle.data = 0;
			 wp_go_id = wp_finish_id;
			 ROS_INFO("WP Mission Completed");	
	    }

	}	
	
	// publish topics
	target_id_pub.publish(ros_waypoint_id);
	ROS_INFO("steering_angle : %d Speed : %d \n",s_angle.data ,c_speed.data);

	if(count>=2) {
            if(emergency_stop == true){
                c_speed.data = 0;
                s_angle.data = 0;
            }
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
