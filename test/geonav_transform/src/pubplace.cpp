#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <cmath>

class GPSCircularRangeChecker {
public:
    GPSCircularRangeChecker(double center_lat, double center_lon, double radius, const std::string& place) 
        : nh("~"), center_latitude(center_lat), center_longitude(center_lon), radius_meters(radius), place_name(place) {

        gps_sub = nh.subscribe("/ublox_gps/fix", 10, &GPSCircularRangeChecker::gpsCallback, this);
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        double latitude = msg->latitude;
        double longitude = msg->longitude;

        // 중심점과 현재 위치 사이의 빗변 길이 계산
        double distance = calculateHaversineDistance(center_latitude, center_longitude, latitude, longitude);

        // 원형 범위 내부에 있는지 판별
        if (distance <= 133) {
            std::locale::global(std::locale("ko_KR.utf8"));
            ROS_INFO("Current GPS position : %s", place_name.c_str());
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber gps_sub;

    double center_latitude, center_longitude, radius_meters;
    std::string place_name;

    double calculateHaversineDistance(double lat1, double lon1, double lat2, double lon2) {

        double dlat = lat2 - lat1;
        double dlon = lon2 - lon1;
        double distance = sqrt(dlat*dlat + dlon*dlon);
        return distance;
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_circular_range_checker");

    // 생성자 파라미터 설정

    GPSCircularRangeChecker checker(36.768975, 126.934916, 0.000417, "멀티미디어관"); // Multi_media

    ros::spin();

    return 0;
}
