#include <cstring>
#include <cstdlib>
#include <cstdint>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "tusb.h" // wait for USB

#include "lwip/pbuf.h"
#include "lwip/udp.h"

#define UDP_PORT 12321
#define BEACON_MSG_LEN_MAX 127
#define BEACON_TARGET "10.0.1.20"//"127.0.0.1"//"255.255.255.255"
// #define BEACON_TARGET "127.0.0.1"//"255.255.255.255"
// #define BEACON_INTERVAL_MS 1000
// #define WIFI_SSID "test"
// #define WIFI_PASSWORD "test"

int main() {

  stdio_init_all();

  // if (cyw43_arch_init()) {
  if (cyw43_arch_init_with_country(CYW43_COUNTRY_USA)) {
    printf("failed to initialise\n");
    return 1;
  }

  cyw43_arch_enable_sta_mode();

  // wait for USB serial connection
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  printf("Connecting to Wi-Fi: %s %s\n", WIFI_SSID, WIFI_PASSWORD);
  if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
    printf("FAILED: unable to connect to %s\n", WIFI_SSID);
    return 1;
  } else {
    printf("SUCCESS: connected to %s\n", WIFI_SSID);
  }

  struct udp_pcb* pcb = udp_new();
  udp_bind(pcb, IP_ADDR_ANY, UDP_PORT);

  ip_addr_t addr;
  ipaddr_aton(BEACON_TARGET, &addr);
  // IP4_ADDR(&addr, 127, 0, 0, 1);
  udp_connect(pcb,&addr,UDP_PORT);

  uint32_t counter = 0;
  while(1) {
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, BEACON_MSG_LEN_MAX+1, PBUF_RAM);
    char *req = (char *)p->payload;
    memset(req, 0, BEACON_MSG_LEN_MAX+1);
    snprintf(req, BEACON_MSG_LEN_MAX, "%d\n\r", counter);
    err_t er = udp_sendto(pcb, p, &addr, UDP_PORT);
    pbuf_free(p);
    if (er != ERR_OK) {
      printf("Failed to send UDP packet! error=%d\n", er);
    } else {
      printf("Sent packet %d\n", counter);
      counter++;
    }

    sleep_ms(10);
  }

  cyw43_arch_deinit();
  return 0;
}