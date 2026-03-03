#pragma once

// Minimal SoftwareSerial stub for PlatformIO native unit tests.
// The library only needs the type and a small method surface to compile.

#include "Arduino.h"

class SoftwareSerial
{
public:
  SoftwareSerial(int /*rx*/ = -1, int /*tx*/ = -1) {}

  void begin(unsigned long /*baud*/) {}

  int available() { return 0; }
  int read() { return -1; }
  size_t write(uint8_t /*c*/) { return 0; }
};
