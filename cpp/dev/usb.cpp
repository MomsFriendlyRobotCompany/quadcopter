
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include "pico/stdlib.h"
#include "tusb.h" // wait for USB


class SerialUSB {
public:
  SerialUSB() {}
  ~SerialUSB() {}

  uint32_t read(uint8_t *buffer, uint32_t size, uint32_t timeout=10) {
    uint32_t cnt = 0;
    int b;
    while (cnt < size) {
      b = getchar_timeout_us(timeout);
      if (b == PICO_ERROR_TIMEOUT) break;
      buffer[cnt++] = (uint8_t)b;
    }
    return cnt;
  }

  int read(uint32_t timeout=10) {
    int b = getchar_timeout_us(timeout);
    if (b == PICO_ERROR_TIMEOUT) return -1;
    return (uint8_t)b;
  }

  void write(const uint8_t *buffer, size_t size) {
    while (size > 0) {
      size -= fwrite(buffer,1,size,stdout);
    }
    // char c = '\n';
    // fwrite(&c,1,1,stdout); // send '\n' to flush
  }

  void write(const uint8_t b) {
    fwrite(&b,1,1,stdout);
  }
};

// uint32_t read_stdin(char *buff, uint32_t size, uint32_t timeout) {
//   uint32_t cnt = 0;
//   int c;
//   while (cnt < size) {
//     c = getchar_timeout_us(timeout);
//     if (c == PICO_ERROR_TIMEOUT) break;
//     buff[cnt++] = (char)c;
//   }
//   return cnt;
// }


int main() {

  stdio_init_all();

  // wait for USB serial connection
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  printf("<<< start >>>\n");

  SerialUSB serial;

  uint8_t msg[] = "hello\n";
  serial.write(msg, 6);

  uint8_t buffer[32];

  // Wait for interrupts
  while (1) {
    uint32_t num = serial.read(buffer,31);
    if (num > 0) {
      buffer[num++] = '\n';
      serial.write(buffer, num);
    }
    sleep_ms(100);
  }

  return 0;
}