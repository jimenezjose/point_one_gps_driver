#ifndef FUSION_ENGINE_MESSAGE_EVENT_HPP
#define FUSION_ENGINE_MESSAGE_EVENT_HPP

#include "gps_msgs/msg/gps_fix.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "point_one_gps_driver/fusion_engine/message_type.hpp"


namespace fusion_engine {

/**
 * Data class that wraps message data in a generic object.
 */
class MessageEvent {
public:
  gps_msgs::msg::GPSFix gps_fix;
  sensor_msgs::msg::NavSatFix nav_sat_fix;
  sensor_msgs::msg::Imu imu;
  geometry_msgs::msg::PoseStamped pose;
  MessageType message_type;

  MessageEvent(gps_msgs::msg::GPSFix gps_fix_) 
      : gps_fix(gps_fix_), message_type(MessageType::GPS_FIX) {}

  MessageEvent(sensor_msgs::msg::NavSatFix nav_sat_fix_) 
      : nav_sat_fix(nav_sat_fix_), message_type(MessageType::NAV_SAT_FIX) {}

  MessageEvent(sensor_msgs::msg::Imu imu_)     
      : imu(imu_), message_type(MessageType::IMU) {}

  MessageEvent(geometry_msgs::msg::PoseStamped pose_)
      : pose(pose_), message_type(MessageType::POSE) {}

};

} // namespace fusion_engine

#endif