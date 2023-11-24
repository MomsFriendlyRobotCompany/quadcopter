
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "tusb.h" // wait for USB

uint32_t read_stdin(char *buff, uint32_t size, uint32_t timeout) {
  uint32_t cnt = 0;
  int c;
  while (cnt < size) {
    c = getchar_timeout_us(timeout);
    if (c == PICO_ERROR_TIMEOUT) break;
    buff[cnt++] = (char)c;
  }
  return cnt;
}


int main() {

  stdio_init_all();

  // wait for USB serial connection
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  printf("<<< start >>>\n");

  char msg[] = "hello\n";
  fwrite(msg,1,6,stdout);
  fwrite(msg,1,6,stdout);

  char buffer[32];

  // Wait for interrupts
  while (1) {
    uint32_t num = read_stdin(buffer, 31, 1);
    if (num > 0) {
      buffer[num++] = '\n';
      fwrite(buffer,1,num,stdout);
      // fflush(stdout);
    }
    sleep_ms(100);
  }

  return 0;
}