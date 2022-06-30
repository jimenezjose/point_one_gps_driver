#ifndef FUSION_ENGINE_UTILS_HPP
#define FUSION_ENGINE_UTILS_HPP

#include "sensor_msgs/msg/imu.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "point_one/fusion_engine/messages/ros.h"

namespace fusion_engine {

class Utils {
public:
  /**
   * Helper method to translate FusionEngine GPSFixMessage to ROS standard GPSFix.
   * @param contents Culprit gps data to be translated.
   * @return ROS standard message - GPSFix.
   */
  static gps_msgs::msg::GPSFix toGPSFix(const point_one::fusion_engine::messages::ros::GPSFixMessage & contents) {
    gps_msgs::msg::GPSFix gps_fix;
    gps_fix.latitude  = contents.latitude_deg;
    gps_fix.longitude = contents.longitude_deg;
    gps_fix.altitude  = contents.altitude_m;
    gps_fix.track     = contents.track_deg;
    gps_fix.speed     = contents.speed_mps;
    gps_fix.climb     = contents.climb_mps;
    gps_fix.pitch     = contents.pitch_deg;
    gps_fix.roll      = contents.roll_deg;
    gps_fix.dip       = contents.dip_deg;
    gps_fix.time      = contents.p1_time.seconds + (contents.p1_time.fraction_ns * 1e-9); // time since power-on
    gps_fix.gdop      = contents.gdop;
    gps_fix.hdop      = contents.hdop;
    gps_fix.vdop      = contents.vdop;
    gps_fix.tdop      = contents.tdop;
    gps_fix.err       = contents.err_3d_m;
    gps_fix.err_horz  = contents.err_horiz_m;
    gps_fix.err_vert  = contents.err_vert_m;
    gps_fix.err_speed = contents.err_speed_mps;
    gps_fix.err_climb = contents.err_climb_mps;
    gps_fix.err_time  = contents.err_time_sec;
    gps_fix.err_pitch = contents.err_pitch_deg;
    gps_fix.err_roll  = contents.err_roll_deg;
    gps_fix.err_dip   = contents.err_dip_deg;
    std::copy(std::begin(contents.position_covariance_m2), std::end(contents.position_covariance_m2), std::begin(gps_fix.position_covariance));
    gps_fix.position_covariance_type = contents.position_covariance_type;
    // TODO (github/jimenezjose): Feature suggestion to add gps_fix.status.status for complete conversion.
    return gps_fix;
  }

  /**
   * Helper method to translate FusionEngine GPSFixMessage to ROS standard NavSatFix.
   * @param contents Culprit gps data to be translated.
   * @return ROS standard message - NavSatFix.
   */
  static sensor_msgs::msg::NavSatFix toNavSatFix(const point_one::fusion_engine::messages::ros::GPSFixMessage & contents) {
    sensor_msgs::msg::NavSatFix nav_sat_fix;
    // fix.header = gps_fix.header;
    // fix.status.status = gps_fix.status.status;
    // fix.status.service = 0;
    // if (gps_fix.status.position_source & gps_msgs::msg::GPSStatus::SOURCE_GPS)
    // {
    //   fix.status.service = fix.status.service | sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    // }
    // if (gps_fix.status.orientation_source & gps_msgs::msg::GPSStatus::SOURCE_MAGNETIC)
    // {
    //   fix.status.service = fix.status.service | sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
    // }
    nav_sat_fix.latitude  = contents.latitude_deg;
    nav_sat_fix.longitude = contents.longitude_deg;
    nav_sat_fix.altitude  = contents.altitude_m;
    std::copy(std::begin(contents.position_covariance_m2), std::end(contents.position_covariance_m2), std::begin(nav_sat_fix.position_covariance));
    nav_sat_fix.position_covariance_type = contents.position_covariance_type;
    return nav_sat_fix;
  }

  /**
   * Helper method to translate FusionEngine IMUMessage to ROS standard Imu.
   * @param contents Culprit gps data to be translated.
   * @return ROS standard message - Imu.
   */
  static sensor_msgs::msg::Imu toImu(const point_one::fusion_engine::messages::ros::IMUMessage & contents) {
    sensor_msgs::msg::Imu imu;
    imu.orientation.x = contents.orientation[0];
    imu.orientation.y = contents.orientation[1];
    imu.orientation.z = contents.orientation[2];
    imu.orientation.w = contents.orientation[3];
    std::copy(std::begin(contents.orientation_covariance), std::end(contents.orientation_covariance), std::begin(imu.orientation_covariance));
    imu.angular_velocity.x = contents.angular_velocity_rps[0];
    imu.angular_velocity.y = contents.angular_velocity_rps[1];
    imu.angular_velocity.z = contents.angular_velocity_rps[2];
    std::copy(std::begin(contents.angular_velocity_rps), std::end(contents.angular_velocity_rps), std::begin(imu.angular_velocity_covariance));
    imu.linear_acceleration.x = contents.acceleration_mps2[0];
    imu.linear_acceleration.y = contents.acceleration_mps2[1];
    imu.linear_acceleration.z = contents.acceleration_mps2[2];
    std::copy(std::begin(contents.acceleration_covariance), std::end(contents.acceleration_covariance), std::begin(imu.linear_acceleration_covariance));
    return imu;
  }

  /**
   * Helper method to translate FusionEngine PoseMessage to ROS standard PoseStamped.
   * @param contents Culprit pose data to be translated.
   * @return ROS standard message - PoseStamped.
   */
  static geometry_msgs::msg::PoseStamped toPose(const point_one::fusion_engine::messages::ros::PoseMessage & contents) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = contents.position_rel_m[0];
    pose_stamped.pose.position.y = contents.position_rel_m[1];
    pose_stamped.pose.position.z = contents.position_rel_m[2];
    pose_stamped.pose.orientation.x = contents.orientation[0];
    pose_stamped.pose.orientation.y = contents.orientation[1];
    pose_stamped.pose.orientation.z = contents.orientation[2];
    pose_stamped.pose.orientation.w = contents.orientation[3];
    return pose_stamped;
  }
};

} // namespace fusion_engine

#endif