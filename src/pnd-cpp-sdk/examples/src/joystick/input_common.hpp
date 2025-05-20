#ifndef INPUT_COMMON_HPP
#define INPUT_COMMON_HPP

#include <iostream>

class InputCommon {
 public:
  std::atomic_char key_ = 0;
  std::atomic_int axis_front_rear_ = 0;
  std::atomic_int axis_left_right_ = 0;

 public:
  virtual ~InputCommon() = default;
  virtual int run() = 0;
};

#endif