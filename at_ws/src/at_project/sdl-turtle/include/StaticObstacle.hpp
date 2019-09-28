#ifndef SDL_TURTLE_STATICOBSTACLE_HPP
#define SDL_TURTLE_STATICOBSTACLE_HPP

#include "BaseActor.hpp"

class StaticObstacle : public BaseActor {
public:
  /**
   * Translates the current position and orientation into the map
   * and renders this actor.
   */
  void render(SDL_Renderer *Renderer) const override;

  /**
   * Initializes this actor.
   */
  void initialize(ros::NodeHandle &Handle) override;

  /**
   * Constructor taking position arguments.
   */
  StaticObstacle(Application *World, const uint32_t X, const uint32_t Y);

  /**
   * Default move constructor.
   */
  StaticObstacle(StaticObstacle &&Other) = default;

  /**
   * Default move assignment.
   */
  StaticObstacle &operator=(StaticObstacle &&Other) = default;

  /**
   * Deleted copy constructor.
   */
  StaticObstacle(const StaticObstacle &Other) = delete;

  /**
   * Deleted copy assignment.
   */
  StaticObstacle &operator=(const StaticObstacle &Other) = delete;

  /**
   * Default virtual destructor.
   */
  ~StaticObstacle() override = default;
};

#endif // SDL_TURTLE_STATICOBSTACLE_HPP
