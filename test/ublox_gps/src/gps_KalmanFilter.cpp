#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

class KalmanFilter {
public:
    KalmanFilter(double initial_state,  double measurement_noise, double initial_covariance, double process_noise) :
        state_(initial_state),
        covariance_(initial_covariance),
        process_noise_(process_noise),
        measurement_noise_(measurement_noise) {}

    double filter(double measurement, double position_covariance) {
        
        // Prediction
        double predicted_state = state_;
        double predicted_covariance = covariance_ + process_noise_;

        measurement_noise_ = position_covariance;

        // Calculate Kalman Gain with weighted position_covariance
        double kalman_gain = predicted_covariance / (predicted_covariance + measurement_noise_);

        // Update
        state_ = predicted_state + kalman_gain * (measurement - predicted_state);
        covariance_ = (1 - kalman_gain) * predicted_covariance;

        return state_;
    }

private:
    double state_;
    double covariance_;
    double process_noise_;
    double measurement_noise_;
};

class GPSToKalmanNode {
public:
    GPSToKalmanNode() : nh_("~"), kalman_filter_lat_(36.76905957, 1.0, 1.0, 0), kalman_filter_lon_(126.93517886, 1.0, 1.0, 0) {
        sub_ = nh_.subscribe("/ublox_gps/fix", 1, &GPSToKalmanNode::gpsCallback, this);
        pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/filtered_gps_topic", 1);
        prev_latitude_ = 0.0;  // Initialize previous values
        prev_longitude_ = 0.0;
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // Apply Kalman filter to GPS latitude and longitude, considering position_covariance
        double filtered_latitude = kalman_filter_lat_.filter(msg->latitude, msg->position_covariance[0]);
        double filtered_longitude = kalman_filter_lon_.filter(msg->longitude, msg->position_covariance[4]);


        // Update previous values
        //prev_latitude_ = filtered_latitude;
        //prev_longitude_ = filtered_longitude;

        // Create filtered GPS message
        sensor_msgs::NavSatFix filtered_gps;
        filtered_gps.header = msg->header;
        filtered_gps.latitude = filtered_latitude;
        filtered_gps.longitude = filtered_longitude;
        filtered_gps.position_covariance = msg->position_covariance;
        filtered_gps.position_covariance_type = msg->position_covariance_type;

        // Publish the filtered GPS message
        pub_.publish(filtered_gps);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    KalmanFilter kalman_filter_lat_;
    KalmanFilter kalman_filter_lon_;
    double prev_latitude_;
    double prev_longitude_;
    const double threshold_covariance_ = 1.0;  // Set your threshold value here
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_to_kalman_node");
    GPSToKalmanNode node;
    ros::spin();
    return 0;
}

