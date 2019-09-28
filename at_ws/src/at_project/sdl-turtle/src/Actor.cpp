#include "Actor.hpp"

#include "Application.hpp"
#include "Movement.hpp"

#include <SDL_image.h>

#include <ca_msgs/Bumper.h>
#include <ros/package.h>
#include <ros/console.h>
#include <at_msgs/TurtlePosition.h>
#include <turtlesim/Pose.h>

Actor::Actor(Application *World, std::string Name, const uint32_t X,
             const uint32_t Y, const Orientation orient)
    : BaseActor(World, X, Y), Name(std::move(Name)), Orient(orient) {
  loadSprite(ros::package::getPath("sdl_turtle") + "/resources/turtle.png",
             World->getRenderer());

  ROS_INFO("Spawn agent %s at [%d, %d]", Name.c_str(), X, Y);
}

bool Actor::getPosition(at_msgs::TurtlePosition::Request &Req,
                        at_msgs::TurtlePosition::Response &Res) {
  Res.x = X;
  Res.y = Y;
  Res.theta = static_cast<uint8_t>(Orient);
  return true;
}

void Actor::initialize(ros::NodeHandle &Handle) {
  BumperPub = Handle.advertise<ca_msgs::Bumper>(Name + "/bumper", 1);
  PosePub = Handle.advertise<turtlesim::Pose>(Name + "/pose", 1);
  PositSrv =
      Handle.advertiseService(Name + "/getPosition", &Actor::getPosition, this);

  ROS_INFO("%s: Initialized Position, Bumper and Pose publisher", Name.c_str());
}

bool Actor::getNextMove(ros::NodeHandle &Handle,
                        at_msgs::NextMoveSrv &Move) {
  const std::string SrvName{ Name + "/next_move" };

  // ****
  // INFO: This still does not work correctly for a _persistent_ connection.
  // ****
  // Querying information about the /turtleX/next_move service on-demand
  auto MoveSrv = Handle.serviceClient<at_msgs::NextMoveSrv>(SrvName);
  if (MoveSrv.call(Move)) {
    return true;
  }

  ROS_INFO("Error in /%s/next_move", Name.c_str());
  return false;
}

void Actor::move(const at_msgs::NextMoveSrv &NextMove) {
  const auto Move = static_cast<Movement>(NextMove.response.move);
  switch (Move) {
  case Movement::FORWARD: {
    moveForward();
    break;
  }
  case Movement::TURN_RIGHT: {
    turnRight();
    break;
  }
  case Movement::BACKWARD: {
    moveBackward();
    break;
  }
  case Movement::TURN_LEFT: {
    turnLeft();
    break;
  }
  case Movement::NOOP:
  default: { break; }
  }
}

void Actor::loadSprite(const std::string &Path, SDL_Renderer *Renderer) {
  auto Surf = SurfacePtr{ IMG_Load(Path.c_str()) };
  if (!Surf) {
    ROS_INFO("Could not load sprite: %s", Path.c_str());
    return;
  }

  Texture = TexturePtr{ SDL_CreateTextureFromSurface(Renderer, Surf.get()) };

  if (!Texture) {
    ROS_INFO("Could not convert surface to texture!");
  }
}

void Actor::render(SDL_Renderer *Renderer) const {
  const auto HGap = World->HorizontalGap;
  const auto VGap = World->VerticalGap;
  const auto CSize = World->CellSize;

  const auto WidthOffset = (HGap - CSize) / 2;
  const auto HeightOffset = (VGap - CSize) / 2;

  SDL_Rect DstRect{ /*x=*/static_cast<int32_t>(X * HGap + WidthOffset),
                    /*y=*/static_cast<int32_t>(Y * VGap + HeightOffset),
                    /*w=*/static_cast<int32_t>(CSize),
                    /*h=*/static_cast<int32_t>(CSize) };

  SDL_RenderCopyEx(Renderer, Texture.get(), /*SrcRect=*/nullptr, &DstRect,
                   90 * static_cast<uint8_t>(Orient), /*Center=*/nullptr,
                   SDL_FLIP_HORIZONTAL);
}

// move to application
bool Actor::checkCollision(const uint32_t NewX, const uint32_t NewY) const {
  return World->isCollision(NewX, NewY);
}

void Actor::moveUp() {
  if (Y == 0 || checkCollision(X, Y - 1)) {
    collisionResponse(X, Y);
    return;
  }

  performMove(X, Y - 1);
}

void Actor::moveDown() {
  if (Y == World->NumRows - 1 || checkCollision(X, Y + 1)) {
    collisionResponse(X, Y);
    return;
  }

  performMove(X, Y + 1);
}

void Actor::moveLeft() {
  if (X == 0 || checkCollision(X - 1, Y)) {
    collisionResponse(X, Y);
    return;
  }

  performMove(X - 1, Y);
}

void Actor::moveRight() {
  if (X == World->NumCols - 1 || checkCollision(X + 1, Y)) {
    collisionResponse(X, Y);
    return;
  }

  performMove(X + 1, Y);
}

void Actor::performMove(const uint32_t NewX, const uint32_t NewY) {
  X = NewX;
  Y = NewY;

  turtlesim::Pose newPose;
  newPose.x = NewX;
  newPose.y = NewY;
  newPose.theta = static_cast<uint8_t>(Orient);

  PosePub.publish(newPose);
}

void Actor::turnRight() {
  const auto Cur = static_cast<uint8_t>(Orient);
  Orient = static_cast<Orientation>((Cur + 1) % 4);
  ROS_INFO("%s: Turning right, orientation is: %d", Name.c_str(), static_cast<uint8_t>(Orient));
}

void Actor::turnLeft() {
  const auto Cur = static_cast<uint8_t>(Orient);

  // careful with negative modulo
  // => add the modulus to it so we stay in the correct range
  // also careful with subtracting from `Cur` when it is 0 (unsigned wrap)
  const auto NextOrient = ((static_cast<int8_t>(Cur) - 1) + 4) % 4;
  Orient = static_cast<Orientation>(NextOrient);
  ROS_INFO("%s: Turning right, orientation is: %d", Name.c_str(), static_cast<uint8_t>(Orient));
}

void Actor::moveForward() {
  switch (Orient) {
  case Orientation::UP: {
    moveUp();
    break;
  }
  case Orientation::RIGHT: {
    moveRight();
    break;
  }
  case Orientation::DOWN: {
    moveDown();
    break;
  }
  case Orientation::LEFT: {
    moveLeft();
    break;
  }
  }
}

void Actor::moveBackward() {
  switch (Orient) {
  case Orientation::UP: {
    moveDown();
    break;
  }
  case Orientation::RIGHT: {
    moveLeft();
    break;
  }
  case Orientation::DOWN: {
    moveUp();
    break;
  }
  case Orientation::LEFT: {
    moveRight();
    break;
  }
  }
}

void Actor::collisionResponse(const uint32_t X, const uint32_t Y) const {
  ROS_INFO("%s at [%d, %d] collided!", Name.c_str(), X, Y);

  ca_msgs::Bumper BmpMsg;
  BmpMsg.is_left_pressed = BmpMsg.is_right_pressed = true;

  BumperPub.publish(BmpMsg);
}
