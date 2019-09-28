#include "Application.hpp"
#include "Config.hpp"

#include <SDL.h>
#include <SDL_image.h>
#include <ros/ros.h>

int main(int Argc, char *Argv[]) {
  SDL_Init(SDL_INIT_VIDEO);
  IMG_Init(IMG_INIT_PNG);
  ros::init(Argc, Argv, "sdl_node");

  {
    const auto Width = Config::get().getWindowWidth();
    const auto Height = Config::get().getWindowHeight();

    Application App{ Width, Height };
    App.initialize();
    while (App.run())
      ;
  }

  ros::shutdown();
  IMG_Quit();
  SDL_Quit();

  return EXIT_SUCCESS;
}
