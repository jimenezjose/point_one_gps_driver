#ifndef ATLAS_MESSAGE_LISTENER_HPP
#define ATLAS_MESSAGE_LISTENER_HPP

#include "atlas_gps_driver/interfaces/atlas_message_event.hpp"

/**
 * Abstract interfaces for clients listening for asynchronous data from Atlas.
 */
class AtlasMessageListener {
public:
  /**
   * Triggers when atlas receives a complete message.
   * @param evt Event that wraps the message data received.
   */
  virtual void receivedAtlasMessage(AtlasMessageEvent & evt) = 0;
};

#endif