#ifndef SDL_TURTLE_MOVEMENT_HPP
#define SDL_TURTLE_MOVEMENT_HPP

#include <cstdint>

enum class Movement : uint8_t {
  NOOP = 0,
  FORWARD,
  TURN_RIGHT,
  BACKWARD,
  TURN_LEFT
};

#endif // SDL_TURTLE_MOVEMENT_HPP
