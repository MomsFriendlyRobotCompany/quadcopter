
#pragma once

#include <gcigps.hpp>
#include "uart.hpp"

// extern Serial Serial1;

gci::GPS gps;

void getGps(Serial* serial) {
  bool ok = false;
  while (serial->available()) {
    char c = (char)serial->read();
    // uart_read_blocking(UART_ID, &c, 1);
    // char cc = static_cast<char>(c);
    // if (c == 13) cc = '>';
    // if (c == 10) cc = '<';
    // printf("%c", cc);
    ok = gps.read(c);
    if (ok) break;
  }

  if (!ok) return;

  gga_t gga;
  gci::GpsID id = gps.get_id();
  if (id == gci::GpsID::GGA) {
    ok = gps.get_msg(gga);
    if (ok) printf("GGA lat: %f lon: %f\n", gga.lat, gga.lon);
    else printf("Bad parsing\n");
  }
}