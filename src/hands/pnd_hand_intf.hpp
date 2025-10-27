#pragma once

#include <vector>

class PndHandInterface {
 public:
  virtual ~PndHandInterface() = default;

  virtual void setPosition(const std::vector<int> &pos) = 0;
};

PndHandInterface *getPndHandInterface(const std::vector<int> &values = {});
