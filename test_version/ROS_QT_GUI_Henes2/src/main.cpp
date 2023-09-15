#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testui2");
    ros::NodeHandle nh;

    QApplication a(argc, argv);
    MainWindow w;

    ros::Subscriber steeringAngleSub = nh.subscribe("/steering_angle", 1, &MainWindow::steeringAngleCallback, &w);
    ros::Subscriber odomSub = nh.subscribe("/odom", 1, &MainWindow::odomCallback, &w);
    ros::Subscriber WaySteerControlSub = nh.subscribe("/Car_Control_cmd/SteerAngle_Int16",10, &MainWindow::WaySteerControlCallback, &w);
    ros::Subscriber PIDcarspeedSub = nh.subscribe("PID_car_speed",10, &MainWindow::PIDcarspeedCallback, &w);
    ros::Subscriber PIDerrorSub = nh.subscribe("PID_error",10, &MainWindow::PIDerrorCallback, &w);
    ros::Subscriber encoderarduinoSub = nh.subscribe("encoder_arduino", 10, &MainWindow::encoderarduinoCallback, &w);
    ros::Subscriber imudataSub = nh.subscribe("imu/data",1, &MainWindow::imudataCallback, &w);

    ros::Subscriber cameraImageSub = nh.subscribe("/camera/color/image_raw", 1, &MainWindow::imageCallback, &w);
    //ros::Subscriber cameraImageSub2 = nh.subscribe("2번 카메라 토픽명", 1, &MainWindow::imageCallback2, &w);

    ros::Subscriber gps_sub = nh.subscribe("/ublox_gps/fix", 10, &MainWindow::gpsCallback, &w);  //
    ros::Subscriber pose_sub = nh.subscribe("/utm", 10, &MainWindow::poseCallback, &w);    //

    ros::Subscriber chatter_sub = nh.subscribe("chatter", 10, &MainWindow::chatterCallback, &w);

    w.show();

    QTimer rosTimer;
    rosTimer.setInterval(10);

    QObject::connect(&rosTimer, &QTimer::timeout, [](){
        ros::spinOnce();
    });
    rosTimer.start();

    return a.exec();
}
