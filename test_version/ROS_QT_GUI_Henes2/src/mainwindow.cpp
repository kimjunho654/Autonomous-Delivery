#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <cv_bridge/cv_bridge.h>
#include <QPixmap>
#include <QImage>
#include <QThread>
#include <QTimer>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    steeringAngleSub = nh.subscribe("/steering_angle", 1, &MainWindow::steeringAngleCallback, this);
    odomSub = nh.subscribe("/odom", 1, &MainWindow::odomCallback, this);
    WaySteerControlSub = nh.subscribe("/Car_Control_cmd/SteerAngle_Int16",10, &MainWindow::WaySteerControlCallback, this);
    PIDcarspeedSub = nh.subscribe("PID_car_speed",10, &MainWindow::PIDcarspeedCallback, this);
    PIDerrorSub = nh.subscribe("PID_error",10, &MainWindow::PIDerrorCallback, this);
    encoderarduinoSub = nh.subscribe("encoder_arduino", 10, &MainWindow::encoderarduinoCallback, this);
    imudataSub = nh.subscribe("imu/data",1, &MainWindow::imudataCallback, this);

    gps_sub = nh.subscribe("/ublox_gps/fix", 10, &MainWindow::gpsCallback, this);
    pose_sub = nh.subscribe("/utm", 10, &MainWindow::poseCallback, this);

    cameraImageSub = nh.subscribe("/camera/color/image_raw", 1, &MainWindow::imageCallback, this);
    //cameraImageSub2 = nh.subscribe("2번 카메라 토픽명", 1, &MainWindow::imageCallback2, this);

    chatter_sub = nh.subscribe("chatter", 10, &MainWindow::chatterCallback, this);

    linearVelocityValue = 0.0;
    angularVelocityValue = 0.0;

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateUI()));
    timer->start(100); // UI 업데이트 주기
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::PIDerrorCallback(const std_msgs::Float64::ConstPtr& msg)
{
    PIDerror = msg->data;
}

void MainWindow::PIDcarspeedCallback(const std_msgs::Int16::ConstPtr& msg)
{
    PIDcarspeed = msg->data;
}

void MainWindow::WaySteerControlCallback(const std_msgs::Int16::ConstPtr& msg)
{
    WaySteerControl = msg->data;
}

void MainWindow::steeringAngleCallback(const std_msgs::Float32::ConstPtr& msg)
{
    steeringAngle = msg->data;
}

void MainWindow::encoderarduinoCallback(const std_msgs::Float32::ConstPtr& msg)
{
    encoderarduino = msg->data;
}
void MainWindow::imudataCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imudata = msg->orientation.w;
}

void MainWindow::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double linear_velocity = msg->twist.twist.linear.x;
    double angular_velocity = msg->twist.twist.angular.z;

    linearVelocityValue = linear_velocity;
    angularVelocityValue = angular_velocity;
}

void MainWindow::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    utmLatitude = msg->pose.position.x;
    //utmLongitude = msg->pose.position.y;
    utmLongitudefront = static_cast<int>(msg->pose.position.y);
    utmLongitudeback = msg->pose.position.y - utmLongitudefront;
}

void MainWindow::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    gpsLatitude = msg->latitude;
    gpsLongitude = msg->longitude;
}

void MainWindow::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);        //이미지 메시지를 OpenCV 이미지로 변환
        QImage qimage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step, QImage::Format_RGB888);       //OpenCV 이미지 -> QImage 생성 // 이미지 데이터, 너비, 높이, 행당 바이트 수) 및 이미지 포맷 설정

        //리얼센스 집에오면 테스트해보기
        //QImage qimage(cv_ptr->image.data, static_cast<int>(cv_ptr->image.cols), static_cast<int>(cv_ptr->image.rows), static_cast<int>(cv_ptr->image.step), QImage::Format_RGB888);

        ui->cameraImageView->setPixmap(QPixmap::fromImage(qimage));
        ui->cameraImageView->setScaledContents(true);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

//void MainWindow::imageCallback2(const sensor_msgs::Image::ConstPtr& msg)
//{
//    try
//    {
//        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//        QImage qimage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.step, QImage::Format_RGB888);
//
//        ui->cameraImageView2->setPixmap(QPixmap::fromImage(qimage));
//        ui->cameraImageView2->setScaledContents(true);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//    }
//}

void MainWindow::chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    QString message = QString::fromStdString(msg->data);
    ui->chattertextBrowser->append("Received String: " + message);
    target_string = msg->data;
}

void MainWindow::updateUI()
{
    ros::spinOnce();
    ui->steeringAngleLabel->setText(QString::number(steeringAngle));
    ui->WaySteerControlLabel->setText(QString::number(WaySteerControl));
    ui->PIDcarspeedLabel->setText(QString::number(PIDcarspeed));
    ui->PIDerrorLabel->setText(QString::number(PIDerror));
    ui->encoderarduinoLabel->setText(QString::number(encoderarduino));
    ui->linearVelocityLabel->setText(QString::number(linearVelocityValue));
    ui->angularVelocityLabel->setText(QString::number(angularVelocityValue));
    ui->imudataLabel->setText(QString::number(imudata));

    ui->utmLatitudeLabel->setText(QString::number(utmLatitude));
    //ui->utmLongitudeLabel->setText(QString::number(utmLongitude));

    ui->utmLongitudefrontLabel->setText(QString::number(utmLongitudefront));
    ui->utmLongitudebackLabel->setText(QString::number(utmLongitudeback));
}
