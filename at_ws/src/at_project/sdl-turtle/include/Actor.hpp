#ifndef SDL_TURTLE_ACTOR_HPP
#define SDL_TURTLE_ACTOR_HPP

#include "BaseActor.hpp"

#include "Config.hpp"
#include "Orientation.hpp"
#include "SDLDeleter.hpp"

#include <turtle_msg/NextMoveSrv.h>
#include <turtle_msg/TurtlePosition.h>

#include <ros/ros.h>

#include <cstdint>

class Actor : public BaseActor {
protected:
  // this turtle's name
  std::string Name;

  // smart_ptr to the texture to render
  TexturePtr Texture;

  // orientation of the turtle
  Orientation Orient;

  // publish bumper information
  ros::Publisher BumperPub;

  // publish position and orientation
  ros::Publisher PosePub;

  // publish position inside the map
  ros::ServiceServer PositSrv;

  /**
   * Load the image at the specified `Path` and creates `Texture` from it.
   */
  void loadSprite(const std::string &Path, SDL_Renderer *Renderer);

  /**
   * ROS-endpoint for service /turtleX/getPosition.
   * Fills `res` with the current position and orientation.
   */
  bool getPosition(turtle_msg::TurtlePosition::Request &req,
                   turtle_msg::TurtlePosition::Response &res);

  /**
   * Moves one field up, if possible.
   * Calls `collisionResponse` if move is impossible.
   */
  void moveUp();

  /**
   * Moves one field down, if possible.
   * Calls `collisionResponse` if move is impossible.
   */
  void moveDown();

  /**
   * Moves one field left, if possible.
   * Calls `collisionResponse` if move is impossible.
   */
  void moveLeft();

  /**
   * Moves one field right, if possible.
   * Calls `collisionResponse` if move is impossible.
   */
  void moveRight();

  /**
   * Performs the movement to the specified `NewX`, `NewY` coordinates.
   * Publishes the new position via `PosePub`.
   */
  void performMove(const uint32_t NewX, const uint32_t NewY);

  /**
   * Queries the world to check whether a collision occurs at `NewX`, `NewY`.
   */
  bool checkCollision(const uint32_t NewX, const uint32_t NewY) const;

  /**
   * Called upon collision. `X` and `Y` are the coordinates of the current
   * position, NOT where the collision would have happened.
   */
  void collisionResponse(const uint32_t X, const uint32_t Y) const;

public:
  /**
   * Initializes this actor.
   * Advertises services and topics.
   */
  void initialize(ros::NodeHandle &Node) override;

  /**
   * Translates the current position and orientation into the map
   * and renders this actor.
   */
  void render(SDL_Renderer *Renderer) const override;

  /**
   * Turns 90° to the right.
   */
  void turnRight();

  /**
   * Turns 90° to the left.
   */
  void turnLeft();

  /**
   * Move one field forward, relative to the current orientation.
   */
  void moveForward();

  /**
   * Move one field backward, relative to the current orientation.
   */
  void moveBackward();

  /**
   * Performs the move specified by the passed `NextMove`.
   */
  void move(const turtle_msg::NextMoveSrv &NextMove);

  /**
   * Queries the /turtleX/next_move service for the next move.
   * On success, returns true and fills the `Move` parameter.
   * On error, returns false and does not change the `Move` parameter.
   */
  bool getNextMove(ros::NodeHandle &Handle, turtle_msg::NextMoveSrv &Move);

  /**
   * Gets the `Name` of this actor.
   */
  const std::string &getName() const & { return Name; }

  /**
   * Gets the `Name` of this actor.
   * (Overload for temporaries; might leak otherwise)
   */
  std::string getName() && { return Name; }

  /**
   * Constructor.
   */
  Actor(Application *World, std::string Name, const uint32_t X,
        const uint32_t Y, const Orientation orient);
  /**
   * Default move constructor.
   */
  Actor(Actor &&Other) = default;

  /**
   * Default move assignment.
   */
  Actor &operator=(Actor &&Other) = default;

  /**
   * Deleted copy constructor.
   */
  Actor(const Actor &Other) = delete;

  /**
   * Deleted copy assignment.
   */
  Actor &operator=(const Actor &Other) = delete;

  /**
   * Default (virtual) destructor.
   */
  ~Actor() override = default;
};

#endif // SDL_TURTLE_ACTOR_HPP
