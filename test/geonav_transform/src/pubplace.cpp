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
        if (distance <= radius_meters) {
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
    GPSCircularRangeChecker checker2(36.7696255, 126.934242, 0.0004315, "학예관");
    GPSCircularRangeChecker checker3(36.770382, 126.934106, 0.000311, "지역혁신관 / BRIX관");
    GPSCircularRangeChecker checker4(36.770001, 126.934255, 0.000462, "산학협력관");
    GPSCircularRangeChecker checker5(36.769252, 126.933446, 0.000597, "유니토피아관");
    GPSCircularRangeChecker checker6(36.768847, 126.934145, 0.00023, "앙뜨레프레너관");
    GPSCircularRangeChecker checker7(36.767996, 126.934792, 0.000439, "향설생활관 3");
    GPSCircularRangeChecker checker8(36.767145, 126.933844, 0.000635, "글로벌빌리지");
    GPSCircularRangeChecker checker9(36.768018, 126.933539, 0.0006, "향설생활관 2");
    GPSCircularRangeChecker checker10(36.768542, 126.92242, 0.0005603, "향설생활관 1");
    GPSCircularRangeChecker checker11(36.769235, 126.932182, 0.0005906, "공학관");
    GPSCircularRangeChecker checker12(36.769967, 126.931443, 0.0003307, "학생회관");
    GPSCircularRangeChecker checker13(36.769488, 126.930109, 0.0003669, "자연과학관");
    GPSCircularRangeChecker checker14(36.768391, 126.92764, 0.000562, "인문과학관");
    GPSCircularRangeChecker checker15(36.770497, 126.932299, 0.000117, "LINC 사업단");
    GPSCircularRangeChecker checker16(36.770714, 126.933066, 0.00018, "교육과학관 / 신한은행");
    GPSCircularRangeChecker checker17(36.771132, 126.932034, 0.000339, "의료과학관");
    GPSCircularRangeChecker checker18(36.772077, 126.931797, 0.0003277, "미디어랩스관");
    GPSCircularRangeChecker checker19(36.771102, 126.933565, 0.0002094, "한마루 / 교직원식당");
    GPSCircularRangeChecker checker20(36.771357, 126.934144, 0.000254, "학성사 3");
    GPSCircularRangeChecker checker21(36.77212, 126.933563, 0.000333, "학성사 1 / 2");
    GPSCircularRangeChecker checker22(36.773178, 126.933539, 0.0002, "후문");
    GPSCircularRangeChecker checker23(36.769253, 126.927688, 0.0003426, "정문");





    ros::spin();

    return 0;
}
