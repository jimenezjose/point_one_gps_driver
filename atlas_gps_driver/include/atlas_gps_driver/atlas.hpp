#ifndef POINT_ONE_NAV_ATLAS_HPP
#define POINT_ONE_NAV_ATLAS_HPP

#include <vector>
#include <cstdio>

#include "point_one/fusion_engine/parsers/fusion_engine_framer.h"
#include "point_one/fusion_engine/messages/core.h"
#include "point_one/fusion_engine/messages/ros.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "atlas_gps_driver/interfaces/atlas_message_listener.hpp"
#include "atlas_gps_driver/interfaces/atlas_message_event.hpp"
#include "atlas_gps_driver/interfaces/atlas_byte_frame_listener.hpp"
#include "atlas_gps_driver/interfaces/atlas_byte_frame_event.hpp"
#include "atlas_gps_driver/atlas_receiver.hpp"
#include "atlas_gps_driver/common/utils.hpp"

namespace point_one {
  namespace fusion_engine {
    void messageReceived(const messages::MessageHeader&, const void*);
  }
}

/*
 * Reads bit stream from the Point One Nav Atlas and notifies all event
 * listeners attached to this singelton object once a complete message has
 * been received.
 */
class Atlas : public AtlasByteFrameListener {

public:
  /**
   * Singleton object. Only one message parser is necessary.
   */
  static Atlas & getInstance() {
    static Atlas instance; // static method fields are instatiated once
    return instance;
  }

  /**
   * Initialize needed to set a ros envoronment for logging output.
   * @param node Link to ROS environment.
   * @return Nothing.
   */
  void initialize(rclcpp::Node * node, std::string connection_type, std::string tcp_ip, int tcp_port, int udp_port) {
    recv.initialize(node, connection_type, tcp_ip, tcp_port, udp_port);
    this->node_ = node;
  }

  /**
   * Adds an event listener to be notified for every gps message received.
   * @param listener object to be notified for gps message received.
   * @return Nothing.
   */
  void addAtlasMessageListener(AtlasMessageListener & listener) {
    listenerList.push_back(&listener);
  }

  /**
   * Callback function for every new parsed message received from Atlas.
   * @param header Metadata on payload.
   * @param payload_in Message received.
   * @return Nothing.
   */
  void messageReceived(const point_one::fusion_engine::messages::MessageHeader & header, const void * payload_in) {
    auto payload = static_cast<const uint8_t*>(payload_in);

    if(header.message_type == point_one::fusion_engine::messages::MessageType::ROS_GPS_FIX) {
      auto & contents = *reinterpret_cast<const point_one::fusion_engine::messages::ros::GPSFixMessage*>(payload);  
      AtlasMessageEvent evt( AtlasUtils::toGPSFix(contents) );
      fireAtlasMessageEvent(evt);
      // Also report standard {@link NavSatFix} translation of {@link GPSFix}.
      AtlasMessageEvent navSatFixEvt( AtlasUtils::toNavSatFix(contents) );
      fireAtlasMessageEvent(navSatFixEvt);
    } 
    else if(header.message_type == point_one::fusion_engine::messages::MessageType::ROS_IMU) {
      auto & contents = *reinterpret_cast<const point_one::fusion_engine::messages::ros::IMUMessage*>(payload);
      AtlasMessageEvent evt( AtlasUtils::toImu(contents) );
      fireAtlasMessageEvent(evt);
    }
    else if(header.message_type == point_one::fusion_engine::messages::MessageType::ROS_POSE) {
      auto & contents = *reinterpret_cast<const point_one::fusion_engine::messages::ros::PoseMessage*>(payload);
      AtlasMessageEvent evt( AtlasUtils::toPose(contents) );
      fireAtlasMessageEvent(evt);
    }
  }

  /**
   * Callback function for every new byte frame received from Atlas.
   * @note Inherited from AtlasByteFrameListener interface.
   * @param evt Wrapper that holds the byte frame data recieved.
   * @return Nothing.
   */
  void receivedAtlasByteFrame(AtlasByteFrameEvent & evt) {
    framer.OnData(evt.frame, evt.bytes_read);
  }

  /**
   * Main service to receive gps data from Atlas.
   * @return Nothing.
   */
  void service() {
    auto connection_type = recv.get_connection_type();
    RCLCPP_INFO(node_->get_logger(), "Using connection_type %s", connection_type.c_str());
    if (connection_type == "tcp") {
      recv.tcp_service();
    }
    else if (connection_type == "udp") {
      recv.udp_service();
    }
    else {
      RCLCPP_INFO(node_->get_logger(), "Invalid connection type %s", connection_type.c_str());
    }
  }

private:
  point_one::fusion_engine::parsers::FusionEngineFramer framer;
  std::vector<AtlasMessageListener *> listenerList;
  AtlasReceiver & recv;
  rclcpp::Node * node_;

  /* only one instance will exist - singleton object. */
  Atlas() : framer(1024), recv(AtlasReceiver::getInstance()) {
    recv.addByteFrameListener(*this);
    framer.SetMessageCallback(point_one::fusion_engine::messageReceived);
  }

  /**
   * Notifies all AtlasMessageListeners of a newly recieved gps message.
   * @param evt data sent to listeners.
   * @return Nothing.
   */
  void fireAtlasMessageEvent(AtlasMessageEvent evt) {
    for(AtlasMessageListener * listener : listenerList) {
      listener->receivedAtlasMessage(evt);
    }
  }
};

// annoying
namespace point_one {
  namespace fusion_engine {
    void messageReceived(const messages::MessageHeader& header, const void* payload_in) {
      Atlas::getInstance().messageReceived(header, payload_in);
    }
  }
}

#endif