#ifndef FUSION_ENGINE_MESSAGE_LISTENER_HPP
#define FUSION_ENGINE_MESSAGE_LISTENER_HPP

#include "point_one_gps_driver/fusion_engine/message_event.hpp"

namespace fusion_engine {
/**
 * Abstract interface for listening to asynchronous data from a Point One Nav FusionEngine.
 */
class MessageListener {
public:
  /**
   * Triggers when FusionEngine receives a complete message.
   * @param evt Event that wraps the message data received.
   */
  virtual void receivedFusionEngineMessage(MessageEvent & evt) = 0;
};

} // namespace fusion_engine

#endif