#ifndef SDL_TURTLE_BASEACTOR_HPP
#define SDL_TURTLE_BASEACTOR_HPP

#include "Config.hpp"

#include <SDL_render.h>

#include <ros/ros.h>

#include <cstdint>

// forward decl
class Application;

class BaseActor {
protected:
  // parent (owner) of this actor
  Application *World;

  // X position.
  uint32_t X;

  // Y position.
  uint32_t Y;

  // size per cell
  const uint32_t CellSize;

public:
  /**
   * Initialize this actor.
   */
  virtual void initialize(ros::NodeHandle &Handle) = 0;

  /**
   * Render this actor.
   */
  virtual void render(SDL_Renderer *Renderer) const = 0;

  /**
   * Get `X` coordinate.
   */
  uint32_t getX() const { return X; }

  /**
   * Get `Y` coordinate.
   */
  uint32_t getY() const { return Y; }

  /**
   * Default constructor.
   */
  BaseActor() = default;

  /**
   * Constructor taking position arguments.
   */
  BaseActor(Application *World, const uint32_t X, const uint32_t Y)
      : World(World), X(X), Y(Y), CellSize(Config::get().getCellSize()) {}

  /**
   * Default move constructor.
   */
  BaseActor(BaseActor &&Other) = default;

  /**
   * Default move assignment.
   */
  BaseActor &operator=(BaseActor &&Other) = default;

  /**
   * Deleted copy constructor.
   */
  BaseActor(const BaseActor &Other) = delete;

  /**
   * Deleted copy assignment.
   */
  BaseActor &operator=(const BaseActor &Other) = delete;

  /**
   * Virtual default destructor.
   */
  virtual ~BaseActor() = default;
};

#endif // SDL_TURTLE_BASEACTOR_HPP
