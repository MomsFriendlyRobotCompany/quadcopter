
#pragma once

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

bool is_ascii(const char c) {
  if (c >= 33 && c <= 126) return true;
  return false;
}

bool exec(const char* cmd, std::string &result) {
    std::array<char, 128> buffer{0};
    // std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        // throw std::runtime_error("popen() failed!");
        return false;
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        // result += buffer.data();

        for (int i=0; i < buffer.size(); ++i) {
          char c = buffer[i];
          if (c == '\0') break;
          if (is_ascii(c)) result += c;
        }
    }
    // return result;
    return true;
}