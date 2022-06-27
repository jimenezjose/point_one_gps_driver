#ifndef ATLAS_BYTE_FRAME_LISTENER_HPP
#define ATLAS_BYTE_FRAME_LISTENER_HPP

#include "atlas_gps_driver/interfaces/atlas_byte_frame_event.hpp"

/**
 * Abstract interface for clients listening to byte frames.
 */
class AtlasByteFrameListener {
public: 
  /**
   * Triggers when atlas internal port receives a byte frame.
   * @param evt Event that wraps the message data received.
   */
  virtual void receivedAtlasByteFrame(AtlasByteFrameEvent & evt) = 0;
};

#endif