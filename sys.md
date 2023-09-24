# 1. 실외주행 알고리즘 (waypoints_navigation_node.cpp)

![image](https://github.com/kimjunho654/henes_develop/assets/105560901/70ebd034-203c-45b7-aac0-285fc1b48c68)

    실외주행에서 로봇의 현재위치는 GPS 로 받고있으며, 로봇이 센서가 구동된 초기 상태에서 좌, 우로 얼마나 조향했는지 판별하는 것은 IMU로 부터 센싱하고 있다.

    주행 목적지가 정해지면, 주행로봇이 어떠한 경로로 이동할지 미리 경유점을 설정하였다.
    0번 경유점을 가장 먼저 지나야 하므로 로봇은 0번 경유점을 현재의 wp_go_id 를 0으로 설정한다. 로봇이 해당 경유점에 도착하면 다음 경유점인 1번을 현재의 wp_go_id = 1로 설정한다.

    우선 가야할 경유점이 wp_go_id=0일때 로봇의 현재 위치에서 [0]번 경유점으로 얼마나 조향을 틀어야 하는지는 상대좌표 계산을 통해 이루어진다.

  ![image](https://github.com/kimjunho654/henes_develop/assets/105560901/a2d0797c-4b66-4f69-9cb9-ca51ca2cea38)
![image](https://github.com/kimjunho654/henes_develop/assets/105560901/f6a921d7-3a22-4ff9-bc48-07faf483f42c)

    상대좌표 계산은 다음과 같이 이루어 진다.

    *목표위치 : 미리 정의된 경유점 좌표,  현재위치 : GPS UTM 변환 값
    1. 상대 좌표(= 목표위치 - 현재위치)를 계산한다. 이는 x, y 값에 대해 계산된다.

    2. 위 좌표에는 로봇이 현재 어느 방향을 바라보고 있는지에 대해 전혀 고려되지 않는다.
       로봇의 회전을 IMU로 부터 센싱하여, 회전행렬을 적용하여 로봇의 방향도 고려한 상대좌표를 계산한다.

    3. 방향도 고려한 상대좌표 x, y를 통해 조향각도를 계산한다.

# gps로 부터 현재위치 얻기

<pre><code>void utm_Callback(const geometry_msgs::PoseStamped::ConstPtr& msgs)
{
        utm_fix.header = msgs->header;
        utm_fix.pose.position.x     =  msgs->pose.position.x ;
        utm_fix.pose.position.y     =  msgs->pose.position.y ;

        my_pose.x     =  utm_fix.pose.position.x;      //UTM 좌표의 경우 X,Y 좌표가 90도 회전되어 있음
        my_pose.y     =  utm_fix.pose.position.y;      //UTM 좌표의 경우 X,Y 좌표가 90도 회전되어 있음
}

ros::Subscriber utm_sub                  = n.subscribe("/utm",1,&utm_Callback);
</code></pre>

    이때 my_pose.x는 GPS로 /ublox_gps/fix를 센싱하고 이를 /utm으로 변환한 값을 사용하며 로봇의 현재위치를 의미한다. 
    또한 센싱값을 칼만필터를 적용하여 /ublox_gps/fix --> /filtered_gps_topic --> /utm 이런식으로 변환하여 사용하였다.

# IMU 값 얻기
<pre><code>void imu_Callback(const sensor_msgs::Imu::ConstPtr& msg)
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

ros::Subscriber imu_sub                  = n.subscribe("/imu/data",10,&imu_Callback);
</code></pre>

    IMU 데이터로부터 x, y, z축 회전을 나타내는 값을 얻어내고 이 중에서 차량의 조향에 관련한 회전인 yaw값을 얻어온다.
    상대좌표 계산에 있어서 로봇의 회전을 고려해야 하는데 yaw를 my_pose.theta 값으로 설정한다.

    이때 yaw 값에서 offset을 설정하는 이유는 위 그림처럼 상대좌표 계산 과정 설명에 있어서 로봇이 0도 인, 오른쪽을 바라보고 있다고 가정하고 설명했는데
    사실 로봇은 처음 시작상태에서 다른 방향을 바라보고 시작할 수도 있다. 이에 의해서 방향의 초기상태인 offset 값을 개발자가 스스로 알아내서 반영해 주어야 한다. float imu_offset = -53;

# 경유점 지정하기
<pre><code>void init_waypoint(void)
{
        my_waypoints_list[0].x = 315719.89;
        my_waypoints_list[0].y = 4071242.41;

        my_waypoints_list[1].x = 315723.16;
        my_waypoints_list[1].y = 4071239.60;

        my_waypoints_list[2].x = 315727.92;
        my_waypoints_list[2].y = 4071232.56;
        .
        .
        .
        my_waypoints_list[12].x = 315721.12;
        my_waypoints_list[12].y = 4071244.21;

        set_delivery_id = 6;
        wp_finish_id = 12;
}
</code></pre>
    여기서 목적지마다 각각 설정된 set_delivery_id 값에 도착했을 경우 로봇은 정지하여, 손님이 배달 물품을 수령하는 과정을 수행한다.
    이때 wp_go_id는 증가하지 않다가 손님이 배달을 완료했다는 topic을 받으면 wp_go_id는 증가하며 다음 경유지로 주행을 시작한다.

    wp_finish_id 에 도착하면 배달과정이 모두 완료되어 정지한다. 이때 노드가 종료되는 것이 아니라 새로운 목적지를 sub 받고, 주행 시작을 sub 받으면 다시 새롭게 실외주행 알고리즘을 수행한다.

# 상대좌표 계산하기
<pre><code>void base_link_tf_utm(void)
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
</code></pre>
    my_target_pose_goal.x는 다음 경유점의 좌표이며 목적지에 따라 각각의 함수에서 미리 정의된 경유점 좌표로 설정된다. 그것이 my_waypoints_list[wp_go_id].x; 이며 여기서 wp_go_id 인덱스는 0부터 시작하여, 각 경유점에 도착하면 1씩 증가하여 다음 경유점으로 주행하도록 한다.

    목표 경유점 위치와 현재위치를 통해 방향이 고려되지 않은 tf_base_map_x, y 를 계산한다.

    그리고 my_pose.theta 을 통해 방향을 고려한 상대좌표인 waypoint_pos_base_link_x, y를 계산한다.

    현재 위치에서 다음 경유점을 가기 위해 로봇이 조향해야 하는 각도인 waypoint_angle을 계산한다. 이를 degree 값으로 바꾸어 car_angle 에 저장한다.

# 목적지 설정
<pre><code>string target_string;
bool sub_destination = false;

void destination_Callback(const std_msgs::String::ConstPtr& msg)
{
    if(msg->data == "Unitophia" || (msg->data == "Multi_Media_Gwan" || msg->data == "Hak_Ye_Gwan" || msg->data == "BRIX_Gwan" || msg->data == "San_Hak_Hyeop_Ryeok_Gwan" || msg->data == "Gong_Hak_Gwan" || msg->data == "Library" || msg->data == "Antire_preneur_Gwan" || msg->data == "Naturel_Science_Gwan" || msg->data == "Humanities_Social_Science_Gwan" || msg->data == "Main_University" || msg->data == "Global_Village" || msg->data == "Hyang_333" || msg->data == "test") ){

        if(start_command == false) {
            sub_destination = true;
            target_string = msg->data;
        }
    }
}

ros::Subscriber destination_sub  = n.subscribe<std_msgs::String>("/destination",10,&destination_Callback);
</code></pre>
    목적지를 sub받아 target_string 에 저장한다. 이때 한번 sub 받은 목적지가 주행 도중에 변경되는 것을 막기 위해 sub_destination을 통해 방지한다.

# 주행 도중 목적지 변경을 방지
<pre><code>    while (ros::ok() && (sub_destination == false) ){

        ROS_INFO("Wati sub Destination");

        ros::spinOnce();
    }
</code></pre>

# 경로지정
<pre><code>    if(target_string == "init"){ init_waypoint(); } ////////////////////////////////////////////////////////////////////////////////
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
    else if( target_string == "test" ) { test_multi_hakyeah();  destination_data = "test_multi_hakyeah"; }
</code></pre>
    sub 받은 목적지에 따라 각각 다른 경로 설정 함수가 실행된다.

# 출발 명령 sub 후, 주행 시작
<pre><code>void start_command_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    start_command = msg->data;
}

ros::Subscriber start_command_sub      = n.subscribe("/start_command",10,&start_command_Callback);
</code></pre>
<pre><code>    while (ros::ok()) {

        if(emergency_stop == false){
            if(start_command == true){
                c_speed.data = 230;
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
</code></pre>
    출발 명령이 true 일 경우, 주행 명령값 = 230을 pub한다. 이는 속도 or pwm 값을 230으로 준다는 의미는 아니다.

    이 주행 명령값은 speed_pid_control 노드에서 sub 받은 후, 실제 차량 속도제어를 하는데 사용되는 값이며 단순히 230 이면 전진, 0이면 정지 하라는 명령으로 이해하면 된다.

# 경유점, 손님에게 도달했는지 판별
<pre><code>            if( (pos_error_x <= WayPoint_X_Tor) && (pos_error_y <= WayPoint_Y_Tor )) {
                printf("----------------------------\n");
                printf("Arrvied at My WayPoint[%3d] !\n",wp_go_id);
                printf("----------------------------\n");

                if(wp_go_id != set_delivery_id) { wp_go_id++; }

                if( (wp_go_id == set_delivery_id) && (received_product == false) ) {
                    c_speed.data = 0;
                    s_angle.data = 0;
            }
                else if( (wp_go_id == set_delivery_id) && (received_product == true) ) {
                    c_speed.data = 230;
                    s_angle.data = car_angle;
                    wp_go_id++;
                    received_product = false;
                }

            }
</code></pre>
    pos_error_x 와 pos_error_y 값이 일정 값 이하일 경우, 즉 현재의 위치가 각 경유점의 위치에 일정 범위 안에 도착했을경우 현재 목표 위치에 도착했음을 판단한다.

    wp_go_id 와 set_delivery_id를 비교하여 서로 다르다면 로봇이 손님에게 도착하지 않았다는 뜻이므로 wp_go_id를 증가시키고 다음 경유점을 향해 이동한다.

    만약 wp_go_id 와 set_delivery_id이 같다면, 로봇이 손님에게 도착했으므로 정지한다. 이는 상품이 수령되지 않았다면 계속 정지한다. received_product = false
    손님이 상품을 수령했을 경우(received_product = true) 로봇은 wp_go_id를 증가시켜 주행을 시작하여 다시 시작위치로 왕복해서 돌아간다.

# 최종 목적지 도착 판별 / 대기 후 다음 배달 시작
<pre><code>            if(wp_go_id > wp_finish_id) {
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
                    else if( target_string == "test" ) { test_multi_hakyeah();  destination_data = "test_multi_hakyeah"; }
                }
            }
        }
</code></pre>
    wp_go_id > wp_finish_id 이라면 최종 목적지에 도착하여 배달이 완료되었음을 판별한다. 그리고 로봇을 정지시킨다.

    이때 기존 목적지와 다른 새로운 목적지를 sub 받으면 wp_go_id = 0으로 초기화 하고, 새로운 경로를 설정한다. 이후 start_command = true를 sub 받으면 다시 주행이 시작된다.

# 회피 알고리즘
<pre><code>void avoid_function_start_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    if( (avoid_function_start == false) && msg->data) { avoid_function_start = msg->data; }
}

void avoid_heading_angle_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    avoid_heading_angle_right = msg->data[0];
    avoid_heading_angle_left = msg->data[1];

}

ros::Subscriber avoid_function_start_sub = n.subscribe("/avoid_function_start",10,&avoid_function_start_Callback);
ros::Subscriber avoid_heading_angle_sub  = n.subscribe("/avoid_heading_angle",10,&avoid_heading_angle_Callback);
</code></pre>
<pre><code>            // avoid function
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
            }
</code></pre>
    장애물이 감지됨을 판별하는 topic을 sub받아 if문을 통해 회피 알고리즘을 수행한다.
    if문의 내용은 다음과 같다.
    * wp_go_id < wp_finish_id 주행하는 도중에 회피가 수행되야 한다.
    * avoid_function_start == true 장애물이 감지되었을때 회피해야 한다.
    * wp_go_id != set_delivery_id 손님 배송위치에 있을때 회피가 이루어지면 안된다.
    * start_command == true 주행이 시작되고 나서 회피가 수행되야 한다.

    회피 알고리즘이 수행되면 로봇 속도를 60으로 감소시킨다.
    avoid_heading_angle_left 은 장애물의 가장 왼쪽 각도(위치)를 의미한다. 로봇은 보통 우차선을 통해 주행하므로 일반적으로 장애물의 왼쪽으로 회피가 이루어저야 한다.

    if( (avoid_heading_angle_left > 5) && (avoid_heading_angle_right > 5)) 하지만 만약 장애물의 로봇의 왼쪽에 있는 경우, 로봇은 오른쪽으로 회피해야 한다.

# 전방 사람 감지 시, 우선 정지
<pre><code>void detect_name_Callback(const std_msgs::String::ConstPtr& msg)
{
    detect_name = msg->data;

}

void depth_Callback(const std_msgs::Float64::ConstPtr& msg)
{
    depth = msg->data;

}

ros::Subscriber detect_name_sub  = n.subscribe("/detect_name",10,&detect_name_Callback);
ros::Subscriber depth_sub  = n.subscribe("/depth",10,&depth_Callback);
</code></pre>
<pre><code>    if(emergency_stop == true || ((detect_name == "person") && (depth < 2.3)) ){ 
            c_speed.data = 0;
            s_angle.data = 0;
        }
</code></pre>
    사물인식 프로그램 구동 중, 사람이 감지되었고 그 거리가 2.3m 이하 일 경우 로봇을 정지시킨다.
    또한 emergency_stop 이 활성화 된 경우에도 로봇을 정지시킨다.

# 조이스틱으로 조종시 속도, 조향 명령 중지
<pre><code>void start_joy_Callback(const std_msgs::Bool::ConstPtr& msg)
{
    start_joy = msg->data;

    sub_joy_current_time = ros::Time::now().toSec();
}

ros::Subscriber start_joy_sub  = n.subscribe("/start_joy",10,&start_joy_Callback);
</code></pre>
<pre><code>        if( count>=2 && (start_joy == false) ) {

            SteerAngle_pub.publish(s_angle);
            car_speed_pub.publish(c_speed);
        }

        if(ros::Time::now().toSec() - sub_joy_current_time > 1){ start_joy = false; }
</code></pre>
    조이스틱으로 조종하고 있지 않다면 기본적으로 속도, 조향 명령을 pub한다.

    조이스틱으로 조종 중이라면 명령 pub를 중단한다. (주행을 위한 계산을 계속 이루어지며, 값의 pub만 되지 않는다.)

# 2. 속도 제어 및 smooth한 가,감속을 위한 PI제어 (test_pid.cpp)
