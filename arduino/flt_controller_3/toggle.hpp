#pragma once

class Toggle {
  public:
  Toggle(bool v=false): val(v) {}

  bool toggle() {
    val = !val;
    return val;
  }

  void set(bool v) {val = v;}

  explicit operator bool() const { return val; }

  protected:
  bool val;
};