#include <iostream>
#include <serialcomm/serialcomm.hpp>
#include <gecko2.hpp>

using namespace std;

int main() {
  SerialPort ser;
  while (true) {
    cout << "bus" << endl;
    gecko::msleep(500);
  }
}