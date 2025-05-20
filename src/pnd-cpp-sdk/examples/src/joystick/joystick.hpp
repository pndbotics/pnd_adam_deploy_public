#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <iostream>
#include <thread>

#include "SDL2/SDL.h"
#include "input_common.hpp"

class Joystick : public InputCommon {
  std::thread js_thread;
  SDL_Joystick* sdl_joystick = nullptr;
  std::atomic_bool quit = false;

 public:
  Joystick() = default;
  ~Joystick() {
    SDL_JoystickClose(sdl_joystick);
    SDL_Quit();
    quit.store(true);
    if (js_thread.joinable()) js_thread.join();
  }
  int run() {
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
      // SPDERROR("SDL initialization failed: {}", SDL_GetError());
      return -1;
    }

    if (SDL_NumJoysticks() < 1) {
      // SPDERROR("No joysticks found.");
      SDL_Quit();
      return -1;
    }

    sdl_joystick = SDL_JoystickOpen(0);
    if (sdl_joystick == nullptr) {
      // SPDERROR("Failed to open joystick: {}", SDL_GetError());
      SDL_Quit();
      return -1;
    }

    js_thread = std::thread(&Joystick::js_thread_fun, this);
    return 0;
  };

 private:
  void js_thread_fun() {
    // INFO("joystick thread start.");
    SDL_Event event;
    int button_idx = 0;
    while (!quit) {
      while (SDL_PollEvent(&event) != 0) {
        if (event.type == SDL_QUIT) {
          quit = true;
        } else if (event.type == SDL_JOYBUTTONDOWN) {
          // DEBUG("Button {} pressed.", static_cast<int>(event.jbutton.button));
          button_idx = static_cast<int>(event.jbutton.button);
          if (button_idx == 11) {
            key_.store('w');
          } else if (button_idx == 12) {
            key_.store('s');
          } else if (button_idx == 13) {
            key_.store('a');
          } else if (button_idx == 14) {
            key_.store('d');
          } else if (button_idx == 2) {
            key_.store('q');
          } else if (button_idx == 1) {
            key_.store('e');
          } else if (button_idx == 0) {
            key_.store('j');
          } else if (button_idx == 3) {
            key_.store('u');
          }

        } else if (event.type == SDL_JOYBUTTONUP) {
          // DEBUG("Button {} released.", static_cast<int>(event.jbutton.button));
          key_.store(' ');
        } else if (event.type == SDL_JOYAXISMOTION) {
          // DEBUG("Axis {} moved. Value: {}", static_cast<int>(event.jaxis.axis),
          // event.jaxis.value);
          key_ = 'r';
          if (event.jaxis.axis == 1) {
            axis_front_rear_ = event.jaxis.value;
          } else if (event.jaxis.axis == 0) {
            axis_left_right_ = event.jaxis.value;
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // INFO("joystick thread end.");
  }
};

#endif