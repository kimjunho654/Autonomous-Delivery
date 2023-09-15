#include <ros/ros.h>
#include <std_msgs/Int16.h>

ros::Publisher SteerAngle_pub;

std_msgs::Int16 steer_angle_msg;

int steer_angle_current = 0;
int steer_angle_prev = 0;
int alpha = 0;
int sum = 0;
bool msg_positive;

void SteerAngle_Callback(const std_msgs::Int16::ConstPtr& msg)
{

    steer_angle_current = msg->data;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "angle_segementation");
    ros::NodeHandle nh;


    SteerAngle_pub = nh.advertise<std_msgs::Int16>("heading_angle", 1);

    ros::Subscriber SteerAngle_sub = nh.subscribe("/Car_Control_cmd/SteerAngle_Int16", 10, SteerAngle_Callback);

    ros::Rate loop_rate(5);  // 10 

    while (ros::ok()) {	
        if( steer_angle_current == (sum + 1) || steer_angle_current == (sum - 1) ) { alpha = 1; } 
        else if(steer_angle_current == (sum + 2) || steer_angle_current == (sum - 2) ) { alpha = 2; } 
        else if(steer_angle_current > (sum + 3) || steer_angle_current < (sum - 3) ) { alpha = 3; } 


        if(steer_angle_current - sum > 0) { msg_positive = true; }
        if(steer_angle_current - sum < 0) { msg_positive = false; }

        if(msg_positive == true){
            if(steer_angle_current != sum){
                sum += alpha;
                steer_angle_msg.data = sum;
                SteerAngle_pub.publish(steer_angle_msg);
            }
            else if(steer_angle_current == sum){

                steer_angle_msg.data = sum;
                SteerAngle_pub.publish(steer_angle_msg);
            }

        }
        else if(msg_positive == false){
            if(steer_angle_current != sum){
                sum -= alpha;
                steer_angle_msg.data = sum;
                SteerAngle_pub.publish(steer_angle_msg);
            }
            else if(steer_angle_current == sum){

                steer_angle_msg.data = sum;
                SteerAngle_pub.publish(steer_angle_msg);
            }

        }





	loop_rate.sleep();
        ros::spinOnce();
    }


    return 0;


}
