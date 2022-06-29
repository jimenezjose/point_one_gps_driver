#ifndef FUSION_ENGINE_CONNECTION_TYPE_HPP
#define FUSION_ENGINE_CONNECTION_TYPE_HPP

namespace fusion_engine {

/** Type of trasnport layer protocol being used in network connection. */
enum class ConnectionType { 
  UDP = 0, 
  TCP = 1, 
  INVALID = 2, 
};

// TODO (github/jimenezjose): Feature a human readable string conversion.

} // namespace fusion_engine

#endif