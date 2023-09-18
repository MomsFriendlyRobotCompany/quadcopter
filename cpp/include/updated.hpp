
#pragma once

template<typename T>
class Updated {
  public:
  Updated(): updated(false) {}

  const T get() {
    updated = false;
    return value;
  }

  void set(const T& v) {
    value = v;
    updated = true;
  }

  explicit operator bool() const {
    return updated;
  }

  protected:
  T value;
  bool updated;
};