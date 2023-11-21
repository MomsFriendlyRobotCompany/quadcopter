# Timer Callbacks

https://forums.raspberrypi.com/viewtopic.php?t=328648

| Only one alarm callback from a pool is executed at a time. If a second callback becomes due while a first one is still executing, the second callback is delayed until the first one completes.
|
| To have a second callback interrupt the first one before the first one completes, create a second alarm pool using a higher priority (lower numbered) hardware timer and assign the second callback to this higher priority alarm pool.
|
| **Note:** you can also create a pool on the other core if you want the callbacks to run concurrently.

```c
// for core 0
struct repeating_timer timer;

add_repeating_timer_us(
    -25,
    repeating_timer_callback,
    NULL,
    &timer);
```

## Create Alarm Pool

If doing multicore, create a new `alarm_pool_t` for core 1.

```c
bool repeating_timer_callback(struct repeating_timer *t) { ... }

int main() {
    // create an alarm pool
    alarm_pool_t *pool ;
    pool = alarm_pool_create(2, 16); // alarm_num, max_timers

    struct repeating_timer timer;
    alarm_pool_add_repeating_timer_us(
        pool,
        -25, // usec
        repeating_timer_callback,
        NULL, // user data passed to function
        &timer);
}
```