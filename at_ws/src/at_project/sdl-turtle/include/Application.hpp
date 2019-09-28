#ifndef SDL_TURTLE_APPLICATION_HPP
#define SDL_TURTLE_APPLICATION_HPP

#include "Actor.hpp"
#include "SDLDeleter.hpp"
#include "StaticObstacle.hpp"

#include <ros/ros.h>

#include <cstdint>
#include <memory>
#include <vector>

class Application {
  friend class Actor;
  friend class StaticObstacle;

  // map width
  uint32_t Width;

  // map height
  uint32_t Height;

  // number of rows
  uint32_t NumRows;

  // number of columns
  uint32_t NumCols;

  // ros node handle
  ros::NodeHandle Handle;

  // smart_ptr to sdl window
  WindowPtr Window;

  // smart_ptr to sdl renderer
  RendererPtr Renderer;

  // cell width
  uint32_t HorizontalGap;

  // cell height
  uint32_t VerticalGap;

  // cell size
  uint32_t CellSize;

  // all objects (dynamic + static)
  std::vector<std::unique_ptr<BaseActor>> Elements;

  /**
   * Renders the horizontal and vertical lines that make up the map.
   */
  void renderGrid() const;

  /**
   * Checks for a collision at the given `X` and `Y` coordinates.
   */
  bool isCollision(const uint32_t X, const uint32_t Y) const;

  /**
   * Parses the map file and creates the specified objects.
   */
  void parseFieldMap();

  /**
   * Parses the map file at the given `Path`.
   */
  void parseFieldMap(const std::string &Path);

  /**
   * Creates a unique_ptr of the given type `T` from the given argument pack.
   */
  template <typename T, typename... Args> void addElement(Args &&... args) {
    Elements.emplace_back(std::make_unique<T>(std::forward<Args>(args)...));
  }

public:
  /**
   * Initializes the sdl objects, parses the map and initializes parsed objects.
   */
  void initialize();

  /**
   * Performs one simulation step.
   * Return true, if another step should be simulated. Otherwise returns false.
   */
  bool run();

  /**
   * Returns a raw pointer to the renderer.
   */
  SDL_Renderer *getRenderer() const;

  /**
   * Initialize the objects - mandatory before running!
   */
  void initObjects();

  /**
   * Constructor.
   */
  Application(const uint32_t Width, const uint32_t Height)
      : Width(Width), Height(Height), Window(nullptr), Renderer(nullptr) {}

  /**
   * Default destructor.
   */
  ~Application() = default;
};

#endif // SDL_TURTLE_APPLICATION_HPP
