
#pragma once

#include <cstdio>
#include <cstdint>

template<uint16_t BUFFER_SIZE>
class Fifo {
  public:
  uint8_t buffer[BUFFER_SIZE];
  uint16_t head{0}; // write
  uint16_t tail{0}; // read
  uint16_t numElem{0};

  public:
  Fifo() {}

  inline const uint16_t size() const volatile { return numElem; }
  // inline const size_t available() const { return numElem; }
  inline bool isFull() volatile { return numElem >= BUFFER_SIZE; }
  inline bool isEmpty() volatile { return numElem == 0; }
  uint16_t nextPos(const size_t pos) volatile { return (pos + 1) % BUFFER_SIZE; }

  void clear() volatile {
    head = tail = numElem = 0;
    // memset(buffer, 0, BUFFER_SIZE);
  }

  void push(const uint8_t b) volatile {
    if (isFull()) {
      tail = nextPos(tail); // drop oldest
      numElem--;
    }
    buffer[head] = b;
    head = nextPos(head);
    numElem++;
  }

  uint8_t pop() volatile {
    if(isEmpty()) return 0;
    uint8_t ret = buffer[tail];
    tail = nextPos(tail);
    numElem--;
    return ret;
  }
};