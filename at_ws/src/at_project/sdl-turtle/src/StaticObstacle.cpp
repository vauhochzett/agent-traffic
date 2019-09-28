#include "StaticObstacle.hpp"

#include "Application.hpp"

StaticObstacle::StaticObstacle(Application *World, const uint32_t X,
                               const uint32_t Y)
    : BaseActor(World, X, Y) {}

void StaticObstacle::initialize(ros::NodeHandle &Handle) {
  // nothing to do here yet
}

void StaticObstacle::render(SDL_Renderer *Renderer) const {
  const auto HGap = World->HorizontalGap;
  const auto VGap = World->VerticalGap;
  const auto DstRect = SDL_Rect{ /*x=*/static_cast<int32_t>(X * HGap),
                                 /*y=*/static_cast<int32_t>(Y * VGap),
                                 /*w=*/static_cast<int32_t>(HGap),
                                 /*h=*/static_cast<int32_t>(VGap) };

  SDL_SetRenderDrawColor(Renderer, 68, 68, 68, SDL_ALPHA_OPAQUE);
  SDL_RenderFillRect(Renderer, &DstRect);
}
