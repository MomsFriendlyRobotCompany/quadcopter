
#pragma once

class Trigger {
  bool value{false};

  public:
  Trigger() {}
  ~Trigger() {}

  operator bool() {
    if (value) {
      value = false;
      return true;
    }
    return false;
  }

  inline void set() { value = true; }
};