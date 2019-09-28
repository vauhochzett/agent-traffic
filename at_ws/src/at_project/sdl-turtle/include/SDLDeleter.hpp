#ifndef SDL_TURTLE_SDLDELETER_HPP
#define SDL_TURTLE_SDLDELETER_HPP

#include <SDL.h>

#include <memory>

struct SDLDeleter {
  /**
   * Deleter for a `SDL_Window` smart pointer.
   */
  void operator()(SDL_Window *Ptr) const {
    if (Ptr) {
      SDL_DestroyWindow(Ptr);
    }
  }

  /**
   * Deleter for a `SDL_Renderer` smart pointer.
   */
  void operator()(SDL_Renderer *Ptr) const {
    if (Ptr) {
      SDL_DestroyRenderer(Ptr);
    }
  }

  /**
   * Deleter for a `SDL_Texture` smart pointer.
   */
  void operator()(SDL_Texture *Ptr) const {
    if (Ptr) {
      SDL_DestroyTexture(Ptr);
    }
  }

  /**
   * Deleter for a `SDL_Surface` smart pointer.
   */
  void operator()(SDL_Surface *Ptr) const {
    if (Ptr) {
      SDL_FreeSurface(Ptr);
    }
  }
};

// type aliases for convenience
using WindowPtr = std::unique_ptr<SDL_Window, SDLDeleter>;
using RendererPtr = std::unique_ptr<SDL_Renderer, SDLDeleter>;
using TexturePtr = std::unique_ptr<SDL_Texture, SDLDeleter>;
using SurfacePtr = std::unique_ptr<SDL_Surface, SDLDeleter>;

#endif // SDL_TURTLE_SDLDELETER_HPP
