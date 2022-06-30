#ifndef FUSION_ENGINE_POINT_ONE_GPS_HPP
#define FUSION_ENGINE_POINT_ONE_GPS_HPP

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

#include "point_one_gps_driver/fusion_engine/core.h"

#include "point_one_gps_driver/fusion_engine_client.hpp"

namespace point_one {
  namespace fusion_engine {
    void messageReceived(const messages::MessageHeader&, const void*);
  }
}

/*
 * Reads bit stream from the Point One Nav GPS and notifies all event
 * listeners attached to this singelton object once a complete message has
 * been received.
 */
class PointOneGps : public fusion_engine::ByteFrameListener {
public:

  /**
   * Singleton object. Only one FusionEngine message parser is necessary.
   */
  static PointOneGps & getInstance() {
    static PointOneGps instance; // static method fields are instatiated once
    return instance;
  }

  /** Illegal operations for singleton object. */
  PointOneGps(PointOneGps const&) = delete;
  void operator=(PointOneGps const&) = delete;

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
  void addMessageListener(fusion_engine::MessageListener & listener) {
    listenerList.push_back(&listener);
  }

  /**
   * Removes byte frame listener.
   * @param listener Byte frame listener to remove.
   * @return Removal success state.
   */
  bool removeMessageListener(fusion_engine::MessageListener * listener) {
    auto culprit = std::find(listenerList.begin(), listenerList.end(), listener);
    if(culprit != listenerList.end()) {
      // TODO (github/jimenezjose): explore if return listenerList.erase is possible.
      listenerList.erase(culprit);
      return true;
    }
    return false;
  }

  /**
   * Callback function for every new parsed message received from FusionEngine.
   * @param header Metadata on payload.
   * @param payload_in Message received.
   * @return Nothing.
   */
  void messageReceived(const point_one::fusion_engine::messages::MessageHeader & header, const void * payload_in) {
    auto payload = static_cast<const uint8_t*>(payload_in);

    if(header.message_type == point_one::fusion_engine::messages::MessageType::ROS_GPS_FIX) {
      auto & contents = *reinterpret_cast<const point_one::fusion_engine::messages::ros::GPSFixMessage*>(payload);  
      fusion_engine::MessageEvent evt( fusion_engine::Utils::toGPSFix(contents) );
      fireFusionEngineMessageEvent(evt);
      // Also report standard {@link NavSatFix} translation of {@link GPSFix}.
      fusion_engine::MessageEvent navSatFixEvt( fusion_engine::Utils::toNavSatFix(contents) );
      fireFusionEngineMessageEvent(navSatFixEvt);
    } 
    else if(header.message_type == point_one::fusion_engine::messages::MessageType::ROS_IMU) {
      auto & contents = *reinterpret_cast<const point_one::fusion_engine::messages::ros::IMUMessage*>(payload);
      fusion_engine::MessageEvent evt( fusion_engine::Utils::toImu(contents) );
      fireFusionEngineMessageEvent(evt);
    }
    else if(header.message_type == point_one::fusion_engine::messages::MessageType::ROS_POSE) {
      auto & contents = *reinterpret_cast<const point_one::fusion_engine::messages::ros::PoseMessage*>(payload);
      fusion_engine::MessageEvent evt( fusion_engine::Utils::toPose(contents) );
      fireFusionEngineMessageEvent(evt);
    }
  }

  /**
   * Callback function for every new byte frame received from FusionEngine.
   * @note Inherited from fusion_engine::ByteFrameListener interface.
   * @param evt Wrapper that holds the byte frame data recieved.
   * @return Nothing.
   */
  void receivedFusionEngineByteFrame(fusion_engine::ByteFrameEvent & evt) {
    framer.OnData(evt.frame, evt.bytes_read);
  }

  /**
   * Main service to receive gps data from FusionEngine.
   * @return Nothing.
   */
  void service() {
    auto connection_type = recv.get_connection_type();
    RCLCPP_INFO(node_->get_logger(), "Using connection_type %s", connection_type.c_str());
    if(connection_type == "tcp") {
      recv.tcp_service();
    }
    else if(connection_type == "udp") {
      recv.udp_service();
    }
    else {
      RCLCPP_INFO(node_->get_logger(), "Invalid connection type %s", connection_type.c_str());
    }
  }

private:
  point_one::fusion_engine::parsers::FusionEngineFramer framer;
  std::vector<fusion_engine::MessageListener *> listenerList;
  FusionEngineClient & recv;
  rclcpp::Node * node_;

  /* Only one instance will exist - singleton object. */
  PointOneGps() : framer(1024), recv(FusionEngineClient::getInstance()) {
    recv.addByteFrameListener(*this);
    framer.SetMessageCallback(point_one::fusion_engine::messageReceived);
  }

  /**
   * Notifies all fusion_engine::MessageListeners of a newly recieved gps message.
   * @param evt data sent to listeners.
   * @return Nothing.
   */
  void fireFusionEngineMessageEvent(fusion_engine::MessageEvent evt) {
    for(fusion_engine::MessageListener * listener : listenerList) {
      listener->receivedFusionEngineMessage(evt);
    }
  }
};

// annoying
namespace point_one {
  namespace fusion_engine {
    void messageReceived(const messages::MessageHeader& header, const void* payload_in) {
      PointOneGps::getInstance().messageReceived(header, payload_in);
    }
  }
}

#endif