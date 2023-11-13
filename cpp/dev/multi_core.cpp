
#include <cstdint>
#include <cstdio>
#include "pico/stdlib.h"
// #include "pico/util/queue.h"
// #include "pico/mutex.h"
#include "pico/multicore.h"
#include "tusb.h" // wait for USB
#include "flags.hpp"
#include "picolib/picolib.hpp"

constexpr uint32_t FLAG_VALUE = 16;

// class Mutex {
//   mutex_t mutex;
//   uint32_t timeout_ms{0};

//   public:
//   Mutex(uint32_t msec=0): timeout_ms(msec) { mutex_init(&mutex); }
//   ~Mutex() {}

//   void set_timeout(uint32_t msec) { timeout_ms = msec; }

//   bool acquire() {
//     printf("aqc\n");
//     if (timeout_ms > 0) return mutex_enter_timeout_ms(&mutex, timeout_ms);
//     mutex_enter_blocking(&mutex);
//     return true;
//   }

//   void release() { mutex_exit(&mutex); }
//   // operator bool () { return mutex_is_initialized(&mutex); }
//   bool is_ready() { return mutex_is_initialized(&mutex); }
// };

struct SharedMemory_t {
  uint32_t value{0};
  BitFlag status;
};

SharedMemory_t sm;
Mutex sm_mutex;

void main_core_1() {
  while (1) {
    if (sm_mutex.acquire()) {
      sm.value += 1;
      sm.status += FLAG_VALUE; // set flag
      sm_mutex.release();
    }

    sleep_ms(2000); // this core is slow
  }
}

int main() {
  stdio_init_all();

  // wait for USB serial connection
  while (!tud_cdc_connected()) {
    sleep_ms(100);
  }

  multicore_launch_core1(main_core_1);

  while (1) {
    sm_mutex.acquire(); // block here until we get it
    // check if flag set, act of checking clears flag if set
    if (sm.status.is_set(FLAG_VALUE)) {
      printf(">> %u\n", sm.value);
    }
    sm_mutex.release();

    sleep_ms(100); // this core is fast
  }
}