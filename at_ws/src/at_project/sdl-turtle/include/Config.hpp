#ifndef SDL_TURTLE_CONFIG_HPP
#define SDL_TURTLE_CONFIG_HPP

#include <cstdint>
#include <string>

class Config {
  /**
   * Width of the window.
   */
  constexpr static uint32_t WindowWidth{ 1000 };

  /**
   * Height of the window.
   */
  constexpr static uint32_t WindowHeight{ 1000 };

  /**
   * Size of a cell.
   */
  constexpr static uint32_t CellSize{ 30 };

  /**
   * Path to load the turtle image from.
   */
  const std::string SpritePath{ "./resources/turtle.png" };

public:
  /**
   * Thread-safe accessor to the instance.
   */
  static Config &get() {
    static Config Conf;
    return Conf;
  }

  /**
   * Get the window width.
   */
  uint32_t getWindowWidth() const;

  /**
   * Get the window height.
   */
  uint32_t getWindowHeight() const;

  /**
   * Get the cells' size.
   */
  uint32_t getCellSize() const;

  /**
   * Get the path to the image.
   */
  const std::string &getSpritePath() const;

  /**
   * Deleted copy constructor.
   */
  Config(Config const &) = delete;

  /**
   * Deleted copy assignment.
   */
  Config &operator=(Config const &) = delete;

  /**
   * Deleted move constructor.
   */
  Config(Config &&) = delete;

  /**
   * Deleted move assignment.
   */
  Config &operator=(Config &&) = delete;

protected:
  /**
   * Default constructor.
   */
  explicit Config() = default;
};

#endif // SDL_TURTLE_CONFIG_HPP
