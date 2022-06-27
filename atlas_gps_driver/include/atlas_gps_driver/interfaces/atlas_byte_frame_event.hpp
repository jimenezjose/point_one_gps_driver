#ifndef ATLAS_BYTE_FRAME_EVENT_HPP
#define ATLAS_BYTE_FRAME_EVENT_HPP

#include <cstring>
#include <arpa/inet.h>

/**
 * Data class that wraps a received byte frame into a generic object.
 */
class AtlasByteFrameEvent {
public:
  uint8_t * frame;
  size_t bytes_read;

  AtlasByteFrameEvent(uint8_t * frame, size_t bytes_read) 
      : frame{frame}, bytes_read{bytes_read} {}
  
};

#endif