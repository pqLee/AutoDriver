/**
 * LLA, ECEF, ENU 坐标系转换
 */
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/QuaternionStamped.h>
#include <Eigen/Core>

#define DEG_TO_RAD 0.01745329252
#define EARTH_MAJOR 6378137.0            ///< WGS84 MAJOR AXIS
#define EARTH_MINOR 6356752.31424518    ///< WGS84 MINOR AXIS

/**
 * 这里点坐标，比如（1，1，1），即为平面系，将所有的GPS LLA坐标转换到平面系中进行后续的计算。这里坐标系转换流程，LLA->ECEF->ENU，ENU即为我们的平面系，东北天
 * http://xchu.net/2020/01/10/37gnss-localizer/
 */
double imu_roll = 0.0, imu_pitch = 0.0, imu_yaw = 0.0;
//1. lla的起点
Eigen::Vector3d lla_origin_;
//2. enu下的坐标
Eigen::Vector3d gps_pos_;

static double wrapToPm(double a_num, const double a_max) {
    if (a_num >= a_max) {
        a_num -= 2.0 * a_max;
    }
    return a_num;
}

static double wrapToPmPi(const double a_angle_rad) {
    return wrapToPm(a_angle_rad, M_PI);
}

static double calcDiffForRadian(const double lhs_rad, const double rhs_rad) {
    double diff_rad = lhs_rad - rhs_rad;
    if (diff_rad >= M_PI)
        diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
        diff_rad = diff_rad + 2 * M_PI;
    return diff_rad;
}

Eigen::Vector3d gpsTools::GpsMsg2Eigen(const sensor_msgs::NavSatFix &gps_msgs) {
	Eigen::Vector3d
			lla(gps_msgs.latitude, gps_msgs.longitude, gps_msgs.altitude);
	return lla;
}

Eigen::Vector3d gpsTools::LLA2ECEF(const Eigen::Vector3d &lla) {
	Eigen::Vector3d ecef;
	double lat = deg2rad(lla.x());
	double lon = deg2rad(lla.y());
	double alt = lla.z();
	double earth_r = pow(EARTH_MAJOR, 2)
					 / sqrt(pow(EARTH_MAJOR * cos(lat), 2) + pow(EARTH_MINOR * sin(lat), 2));
	ecef.x() = (earth_r + alt) * cos(lat) * cos(lon);
	ecef.y() = (earth_r + alt) * cos(lat) * sin(lon);
	ecef.z() = (pow(EARTH_MINOR / EARTH_MAJOR, 2) * earth_r + alt) * sin(lat);
	
	return ecef;
}

Eigen::Vector3d gpsTools::ECEF2ENU(const Eigen::Vector3d &ecef) {
	double lat = deg2rad(lla_origin_.x());
	double lon = deg2rad(lla_origin_.y());
	
	Eigen::Vector3d t = -LLA2ECEF(lla_origin_);
	Eigen::Matrix3d r;
	r << -sin(lon), cos(lon), 0,
			-cos(lon) * sin(lat), -sin(lat) * sin(lon), cos(lat),
			cos(lon) * cos(lat), sin(lon) * cos(lat), sin(lat);
	
	Eigen::Vector3d enu;
	enu = ecef + t;
	enu = r * enu;
	return enu;
}

//这俩函数自己稍微改一下
static inline double deg2rad(const double &deg) {
		return deg * DEG_TO_RAD;
};
static inline double rad2deg(const double &rad) {
    return rad / DEG_TO_RAD;
}
