#include "Application.hpp"

#include "Config.hpp"

#include <turtle_msg/NextMoveSrv.h>

#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>

#include <ros/console.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>

namespace fs = boost::filesystem;
namespace {
const fs::path userFolder =
    fs::path("/home") / fs::path(std::getenv("USER")) /
    fs::path("pire_ws/src/pire-robosim/config/sdl_field.map");
const fs::path globalFolder = fs::path("/pire/config/sdl_field.map");
}

void Application::initialize() {
  CellSize = Config::get().getCellSize();

  Window = WindowPtr{ SDL_CreateWindow("sdl-turtle", SDL_WINDOWPOS_CENTERED,
                                       SDL_WINDOWPOS_CENTERED, Width, Height,
                                       /*Flags=*/0) };

  Renderer = RendererPtr{ SDL_CreateRenderer(Window.get(), /*Index=*/0,
                                             SDL_RENDERER_ACCELERATED) };
  // Set NumCols, NumRows, Elements according to field map
  parseFieldMap();

  auto NumBaseline = std::max(NumCols, NumRows);
  auto Gap = Width / NumBaseline;
  HorizontalGap = VerticalGap = Gap;

  // Ensure the window does not have superfluous Width and Height
  Width = HorizontalGap * NumCols;
  Height = VerticalGap * NumRows;
  SDL_SetWindowSize(Window.get(), Width, Height);
  // Update the cell size
  CellSize = static_cast<uint32_t>(3.0/5.0 * Gap);

  initObjects();
  SDL_Delay(20);
}

void Application::initObjects() {
  for (const auto &Elem : Elements) {
    Elem->initialize(Handle);
  }
}

void Application::parseFieldMap() {
  if (fs::exists(userFolder)) {
    parseFieldMap(userFolder.string());
  } else if (fs::exists(globalFolder)) {
    parseFieldMap(globalFolder.string());
  } else {
    ROS_FATAL("Config file does not exist. Expecting file in one of these locations: %s | %s", userFolder.c_str(), globalFolder.c_str());	  
    std::exit(EXIT_FAILURE);
  }
}

void Application::parseFieldMap(const std::string &Path) {
  using list_tokenizer = boost::tokenizer<boost::escaped_list_separator<char>>;

  std::ifstream fieldFile{ Path };

  // #<ID>-<Orientation>
  //  ID: 					number in interval [1, ...)
  //  Orientation:	number in interval [0, 3]
  //  							see Orientation enum in
  //  Actor.hpp
  //  for
  //  reference
  std::regex turtleRegex{ "#([1-9]+)-([0-3])" };
  constexpr int32_t WALL = 1;
  NumCols = 0;

  std::string line;
  auto row = 0u;
  while (std::getline(fieldFile, line)) {
    list_tokenizer tknz{ line };
    auto col = 0u;

    // Tokens:
    // 	0: ignored, free field
    // 	1: wall
    // 	.: as defined by regular expressions
    // 		- turtleRegex
    for (const auto &token : tknz) {
      if (token[0] == '#') {
        // parse turtle
        std::smatch groups;
        if (std::regex_match(token, groups, turtleRegex)) {
          const auto id = std::stoi(groups[1].str());
          const auto orient = std::stoi(groups[2].str());
          const auto turtleName = "turtle" + std::to_string(id);
          addElement<Actor>(this, turtleName, col, row,
                            static_cast<Orientation>(orient));
        } else {
          // error
	        ROS_FATAL("Could not parse turtle from token: %s", token.c_str());
          std::exit(EXIT_FAILURE);
        }
      } else if (std::stoi(token) == WALL) {
        addElement<StaticObstacle>(this, col, row);
      }

      ++col;
    }

    if (NumCols == 0) {
      NumCols = col;
    }

    ++row;
  }

  NumRows = row;

  // close file
  fieldFile.close();
}

bool Application::run() {
  // no need to run with 0 turtles
  if (Elements.empty()) {
    return false;
  }

  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    if (e.type == SDL_QUIT) {
      return false;
    } else if (e.type == SDL_KEYDOWN) {
      switch (e.key.keysym.sym) {
      case SDLK_ESCAPE: { // quit on escape
        return false;
      }
      default: { break; }
      }
    }
  }

  // we need to spin so that the callbacks get processed:
  // https://answers.ros.org/question/11887/significance-of-rosspinonce/
  ros::spinOnce();

  std::map<Actor *, turtle_msg::NextMoveSrv> NextMoves;
  for (const auto &Elem : Elements) {
    if (auto *Act = dynamic_cast<Actor *>(Elem.get())) {
      turtle_msg::NextMoveSrv Move;
      if (Act->getNextMove(Handle, Move)) {
        NextMoves.emplace(Act, Move);
      } else {
        ROS_WARN("%s did not respond to next_move request!", Act->getName().c_str());	      
      }
    }
  }

  if (NextMoves.empty()) {
    ROS_WARN("No unit responded to next_move request!");
  }    

  for (const auto &Pair : NextMoves) {
    Pair.first->move(Pair.second);
  }

  auto *Rend = Renderer.get();

  // clear with white
  SDL_SetRenderDrawColor(Rend, 255, 255, 255, SDL_ALPHA_OPAQUE);
  SDL_RenderClear(Rend);

  // rendering
  renderGrid();
  for (const auto &Elem : Elements) {
    Elem->render(Rend);
  }

  // present frame
  SDL_RenderPresent(Rend);

  ros::Duration(1).sleep();

  return true;
}

void Application::renderGrid() const {
  auto *Rend = Renderer.get();
  SDL_SetRenderDrawColor(Rend, 0, 0, 0, SDL_ALPHA_OPAQUE);

  // vertical
  for (auto X = 0u; X < Width; X += HorizontalGap) {
    SDL_RenderDrawLine(Rend, X, 0, X, Height);
  }

  // horizontal
  for (auto Y = 0u; Y < Height; Y += VerticalGap) {
    SDL_RenderDrawLine(Rend, 0, Y, Width, Y);
  }
}

bool Application::isCollision(const uint32_t X, const uint32_t Y) const {
  const auto Iter =
      std::find_if(Elements.begin(), Elements.end(),
                   [&X, &Y](const std::unique_ptr<BaseActor> &Elem) {
                     return Elem->getX() == X && Elem->getY() == Y;
                   });

  return Iter != Elements.end();
}

SDL_Renderer *Application::getRenderer() const { return Renderer.get(); }
