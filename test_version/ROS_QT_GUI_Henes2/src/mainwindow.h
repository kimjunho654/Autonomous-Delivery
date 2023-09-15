#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "ui_mainwindow.h"
#include <QMainWindow>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <QTimer>

#include <sensor_msgs/NavSatFix.h>  //
#include <geometry_msgs/PoseStamped.h>  //

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT


public:
    explicit MainWindow(QWidget *parent = nullptr);
    void steeringAngleCallback(const std_msgs::Float32::ConstPtr& msg);
    void encoderarduinoCallback(const std_msgs::Float32::ConstPtr& msg);
    void WaySteerControlCallback(const std_msgs::Int16::ConstPtr& msg);
    void PIDcarspeedCallback(const std_msgs::Int16::ConstPtr& msg);
    void PIDerrorCallback(const std_msgs::Float64::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
//    void imageCallback2(const sensor_msgs::Image::ConstPtr& msg);
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);  //
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);  //
    void imudataCallback(const sensor_msgs::Imu::ConstPtr& msg);

    void chatterCallback(const std_msgs::String::ConstPtr& msg);

    ~MainWindow();

private slots:

    void updateUI();

private:
    Ui::MainWindow *ui;
    ros::NodeHandle nh;
    ros::Subscriber steeringAngleSub;
    ros::Subscriber encoderarduinoSub;
    ros::Subscriber odomSub;
    ros::Subscriber depthSub;
    ros::Subscriber WaySteerControlSub;
    ros::Subscriber PIDcarspeedSub;
    ros::Subscriber PIDerrorSub;
    ros::Subscriber imudataSub;
    ros::Subscriber chatter_sub;
    float steeringAngle;
    float encoderarduino;
    float WaySteerControl;
    float PIDcarspeed;
    float PIDerror;
    float odomValue;
    float imudata;
    QImage depthImage;
    double linearVelocityValue;
    double angularVelocityValue;

    std::string target_string;

    ros::Subscriber cameraImageSub;
    //ros::Subscriber cameraImageSub2;

    ros::Subscriber gps_sub;
    double utmLatitude;
    //double utmLongitude;
    int utmLongitudefront;
    double utmLongitudeback;

    ros::Subscriber pose_sub;
    double gpsLatitude;
    double gpsLongitude;

    QTimer *timer;
};

#endif // MAINWINDOW_H
