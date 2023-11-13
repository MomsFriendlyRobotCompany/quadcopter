
/*
$PMTK001,314,3*36 - message types to send
$PMTK001,220,3*30 - update rate
$PGACK,33,0*6E - no antenna
*/

#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "tusb.h" // wait for USB
#include <stdio.h>

#include "gcigps.hpp"
#include "picolib/uart.hpp"

Serial Serial1;
gci::GPS gps;

bi_decl(bi_2pins_with_func(UART1_RX_PIN, UART1_TX_PIN, GPIO_FUNC_UART));

int main() {
  stdio_init_all();

  // wait for USB serial connection
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  uint baud = Serial1.init(9600, 1, UART1_TX_PIN, UART1_RX_PIN);

  if (Serial1.is_enabled()) printf("Serial1 is enabled\n");
  printf("/-- UART is set to %u baud --/\n", baud);

  Serial1.write(GCI_GGA, sizeof(GCI_GGA));
  Serial1.write(GCI_UPDATE_1HZ, sizeof(GCI_UPDATE_1HZ));
  Serial1.write(GCI_NOANTENNA, sizeof(GCI_NOANTENNA));
  Serial1.write(GCI_BAUD_115200, sizeof(GCI_BAUD_115200));

  sleep_ms(500);
  baud = Serial1.set_baud(115200);
  printf("/-- UART is reset to %u baud --/\n", baud);

  Serial1.flush();

  while (1) {
    bool ok = false;
    while (Serial1.available() > 0) {
      // printf(".");
      char c = static_cast<char>(Serial1.read());
      printf("%c", c);
      ok = gps.read(c);
      if (ok) break;
    }

    if (!ok) continue;

    gga_t gga;
    gci::GpsID id = gps.get_id();
    if (id == gci::GpsID::GGA) {
      ok = gps.get_msg(gga);
      if (ok) printf("GGA lat: %f lon: %f\n", gga.lat, gga.lon);
      else printf("*** Bad parsing ***\n");
      // printf("buffer: %hu\n", Serial1.available());
    }
  }

  return 0;
}