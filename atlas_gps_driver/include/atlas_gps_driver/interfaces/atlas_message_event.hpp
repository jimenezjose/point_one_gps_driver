#ifndef ATLAS_MESSAGE_EVENT_HPP
#define ATLAS_MESSAGE_EVENT_HPP

#include "gps_msgs/msg/gps_fix.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "atlas_gps_driver/common/atlas_message_type.hpp"

/**
 * Data class that wraps message data in a generic object.
 */
class AtlasMessageEvent {
public:
  gps_msgs::msg::GPSFix gps_fix;
  sensor_msgs::msg::NavSatFix nav_sat_fix;
  sensor_msgs::msg::Imu imu;
  geometry_msgs::msg::PoseStamped pose;
  AtlasMessageType message_type;

  AtlasMessageEvent(gps_msgs::msg::GPSFix gps_fix_) 
      : gps_fix(gps_fix_), message_type(AtlasMessageType::GPS_FIX) {}

  AtlasMessageEvent(sensor_msgs::msg::NavSatFix nav_sat_fix_) 
      : nav_sat_fix(nav_sat_fix_), message_type(AtlasMessageType::NAV_SAT_FIX) {}

  AtlasMessageEvent(sensor_msgs::msg::Imu imu_)     
      : imu(imu_), message_type(AtlasMessageType::IMU) {}

  AtlasMessageEvent(geometry_msgs::msg::PoseStamped pose_)
      : pose(pose_), message_type(AtlasMessageType::POSE) {}

};

#endif