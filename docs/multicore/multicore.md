
# Multicore

## Main

```c
#include "pico/multicore.h"

void core1_entry() {} // core 1

int main() { // core 0
    // Launch core 1
    multicore_launch_core1(core1_entry);
}
```

## Queues or FIFOs?

The C SDK docs says:

| **NOTE:** The inter-core FIFOs are a very precious resource and are frequently used for SDK functionality (e.g. during core 1 launch or by the lockout functions). Additionally they are often required for the exclusive use of an RTOS (e.g. FreeRTOS SMP). For these reasons it is suggested that you do not use the FIFO for your own purposes unless none of the above concerns apply; the majority of cases for transferring data between cores can be eqaully well handled by using a queue [1]

[1]: https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#multicore_fifo


Multi-core and IRQ safe queue implementation. Note that this queue stores values of a specified size, and pushed values are copied into the queue.

```c

typedef struct {
    int32_t (*func)(int32_t);
    int32_t data;
} queue_entry_t;

queue_t call_queue;
queue_t results_queue;


int32_t factorial(int32_t n) {
    int32_t f = 1;
    for (int i = 2; i <= n; i++) {
        f *= i;
    }
    return f;
}

int main() {
    queue_init(&call_queue, sizeof(queue_entry_t), 2);
    queue_init(&results_queue, sizeof(int32_t), 2);

    multicore_launch_core1(core1_entry);

    queue_entry_t entry = {factorial, 12};
    queue_add_blocking(&call_queue, &entry);

    int32_t res;
    queue_remove_blocking(&results_queue, &res);
    ...
}
```

## Spinlock

```c
// Counter spinlock
int spinlock_num_count;
spin_lock_t *spinlock_count;
```

Declare an int to store the number of the spinlock. The RP2040 provides 32 hardware spin locks, which can be used to manage mutually-exclusive access to shared software resources. Spin locks 0-15 are currently reserved for fixed uses by the SDK - i.e. if you use them other functionality may break or not function optimally. Later in the code, we claim a particular spinlock to use.

From the RP2040 datasheet:

| If both cores try to claim the same lock on the same clock cycle, core 0 succeeds. Generally software will acquire a lock by repeatedly polling the lock bit ("spinning" on the lock) until it is successfully claimed. This is inefficient if the lock is held for long periods, so generally the spinlocks should be used to protect the short critical sections of higher-level primitives such as mutexes, semaphores and queues. For debugging purposes, the current state of all 32 spinlocks can be observed via SPINLOCK_ST.

