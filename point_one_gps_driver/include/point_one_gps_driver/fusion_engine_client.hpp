#ifndef FUSION_ENGINE_CLIENT_HPP
#define FUSION_ENGINE_CLIENT_HPP

#include <cerrno>
#include <cmath> 
#include <csignal> 
#include <cstdio> 
#include <cstring> 
#include <string> 
#include <netdb.h> 
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h> 
#include <vector>
#include <arpa/inet.h>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "point_one_gps_driver/fusion_engine/core.h"

/*
 * Reads bit stream from the Point One Nav FusionEngine and notifies all event 
 * listeners attached to this singleton object once a raw FusionEngine 
 * data packet is receieved.
 */
class FusionEngineClient {
public:

  /**
   * Singleton object. Only one message parser is necessary.
   * @return static reference to single FusionEngineClient instance.
   */
  static FusionEngineClient & getInstance() {
    static FusionEngineClient instance; // static method field instatiated once
    return instance;
  }

  /* Illegal singleton copy operations */
  FusionEngineClient(FusionEngineClient const &) = delete;
  void operator=(FusionEngineClient const &) = delete;

  /**
   * Initialize needed to set a ros envoronment for logging output.
   * @param node link to ros environment.
   * @return Nothing.
   */
  void initialize(rclcpp::Node * node, std::string connection_type, std::string tcp_ip, int tcp_port, int udp_port) {
    node_ = node;
    connection_type_ = connection_type;
    tcp_ip_ = tcp_ip;
    tcp_port_ = tcp_port;
    udp_port_ = udp_port;
  }

  /**
   * Adds an event listener to be notified for every byte frames received.
   * @param listener Listener to be notified for byte frames received.
   * @return Nothing.
   */
  void addByteFrameListener(fusion_engine::ByteFrameListener & listener) {
    listenerList.push_back(&listener);
  }

  /**
   * Removes byte frame listener.
   * @param listener Byte frame listener to remove.
   * @return Removal success state.
   */
  bool removeByteFrameListener(fusion_engine::ByteFrameListener * listener) {
    auto culprit = std::find(listenerList.begin(), listenerList.end(), listener);
    if(culprit != listenerList.end()) {
      listenerList.erase(culprit);
      return true;
    }
    return false;
  }

  /**
   * Read udp input stream from the Point One Nav until this
   * system has shut down or this node is killed.
   * @return Nothing.
   */
  void udp_service() {
    uint8_t buffer[1024];
    size_t total_bytes_read = 0;
    struct sockaddr_storage their_addr; // address of recieved packet
    socklen_t addr_len = sizeof(their_addr);
    char their_ip[INET6_ADDRSTRLEN];

    open_udp_socket(); 

    try {
      while(rclcpp::ok()) {
        // continuously read input stream until system is shutdown / rclcpp::ok().
        ssize_t bytes_read = recvfrom(sock_, buffer, sizeof(buffer), 0, (struct sockaddr *)&their_addr, &addr_len);
        if(bytes_read < 0) {
          RCLCPP_INFO(node_->get_logger(), "Error reading from socket: %s (%d)", std::strerror(errno), errno);
          break;
        }   
        else if (bytes_read == 0) {
          RCLCPP_INFO(node_->get_logger(), "Socket closed remotely.");
          break;
        }
        inet_ntop(their_addr.ss_family, get_in_addr((struct sockaddr *)&their_addr), their_ip, sizeof(their_ip));
        total_bytes_read += bytes_read;
        fusion_engine::ByteFrameEvent evt(buffer, bytes_read);
        fireFusionEngineByteFrameEvent(evt);
      }
      close(sock_);
      RCLCPP_INFO(node_->get_logger(), "Finished. %zu bytes read.", total_bytes_read);
    }
    catch(std::exception const & ex) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Decoder exception: " << ex.what());
    }
  }

  /**
   * Read tcp input stream from the Point One Nav until this
   * system has shut down or this node is killed.
   * @return Nothing.
   */
  void tcp_service() {
    uint8_t buffer[1024];
    size_t total_bytes_read = 0;

    open_tcp_socket();

    try {
      while(rclcpp::ok()) {
        ssize_t bytes_read = recv(sock_, buffer, sizeof(buffer), 0);
        if (bytes_read < 0) {
          RCLCPP_INFO(node_->get_logger(), "Error reading from socket: %s (%d)", std::strerror(errno), errno);
          break;
        }
        else if (bytes_read == 0) {
          RCLCPP_INFO(node_->get_logger(), "Socket closed remotely.");
          break;
        }
        total_bytes_read += bytes_read;
        fusion_engine::ByteFrameEvent evt(buffer, bytes_read);
        fireFusionEngineByteFrameEvent(evt);
      }
    } catch(std::exception const & ex) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Decoder exception: " << ex.what());
    }
  }

  /**
   * Getter for type of network transport layer protocol (udp/tcp).
   * @return String identification of connection type protocol.
   */
  std::string get_connection_type() {
    return connection_type_;
  }

private:
  int udp_port_ = 0;
  int tcp_port_ = 0;
  std::string connection_type_ = "";
  std::string tcp_ip_ = "";
  int sock_ = 0;
  std::vector<fusion_engine::ByteFrameListener *> listenerList;
  rclcpp::Node * node_;

  /* private constructor for singleton design */
  FusionEngineClient() {}

  /**
   * Notifies all fusion_engine::ByteFrameListeners of a newly recieved byte frame.
   * @param evt Raw byte frame recieved along with its byte size.
   * @return Nothing.
   */
  void fireFusionEngineByteFrameEvent(fusion_engine::ByteFrameEvent evt) {
    for(fusion_engine::ByteFrameListener * listener : listenerList) {
      listener->receivedFusionEngineByteFrame(evt);
    }
  }

  /**
   * Creates and binds to a UDP network socket.
   * @return status code 0 on success, non-zero otherwise.
   */
  int open_udp_socket() {
    // create UDP socket.
    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(sock_ < 0) {
      RCLCPP_INFO(node_->get_logger(), "Error creating socket");
      return 2;
    }   
    // bind socket to port.
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(udp_port_);
    addr.sin_addr.s_addr = INADDR_ANY; // any local address
    int ret = bind(sock_, (struct sockaddr *) &addr, sizeof(addr));
    if(ret < 0) {
      close(sock_);
      RCLCPP_INFO(node_->get_logger(), "Error binding");
      return 3;
    }
    RCLCPP_INFO(node_->get_logger(), "Opened port %d", udp_port_);
    return 0;
  }

  /**
   * Creates and connects to a tcp network socket.
   * @return status code 0 on success, non-zero otherwise.
   */
  int open_tcp_socket() {
    // Connect the socket.
    sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock_ < 0) {
      RCLCPP_INFO(node_->get_logger(), "Error creating socket");
      return 2;
    }
    const char* hostname = tcp_ip_.c_str();
    hostent* host_info = gethostbyname(hostname);
    if (host_info == NULL) {
      RCLCPP_INFO(node_->get_logger(), "Error: IP address lookup failed for hostname '%s'.\n", hostname);
      return 1;
    }
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(tcp_port_);
    memcpy(&addr.sin_addr, host_info->h_addr_list[0], host_info->h_length);
    int ret = connect(sock_, (sockaddr*)&addr, sizeof(addr));
    if (ret < 0) {
      close(sock_);
      RCLCPP_INFO(node_->get_logger(), "Error connecting to target device: %s (%d)\n", std::strerror(errno),
            errno);
      return 3;
    }
    RCLCPP_INFO(node_->get_logger(), "Opened port %d at ip %s", tcp_port_, hostname);
    return 0;
  }

  /**
   * IPv4 and IPv5 address resolution.
   * @param sa socket address.
   * @return Resolved ip address.
   */
  void * get_in_addr(struct sockaddr * sa) {
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in *)sa)->sin_addr);
    }
    return &(((struct sockaddr_in6 *)sa)->sin6_addr);
  }
};

#endif