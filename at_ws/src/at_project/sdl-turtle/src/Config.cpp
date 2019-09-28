#include "Config.hpp"

uint32_t Config::getWindowWidth() const { return WindowWidth; }
uint32_t Config::getWindowHeight() const { return WindowHeight; }
uint32_t Config::getCellSize() const { return CellSize; }

const std::string &Config::getSpritePath() const { return SpritePath; }
