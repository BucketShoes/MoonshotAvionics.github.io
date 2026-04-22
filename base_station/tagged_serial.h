// tagged_serial.h — Serial.print wrapper that prefixes boot-relative microseconds to each line.
// Usage: #include "tagged_serial.h" — replaces Serial globally with TaggedSerial.

#ifndef TAGGED_SERIAL_H
#define TAGGED_SERIAL_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <cstring>

extern unsigned long bootMicros;  // set in setup()

class TaggedSerial : public Print {
  HardwareSerial* _serial;
  bool _atLineStart;

 public:
  TaggedSerial(HardwareSerial* serial) : _serial(serial), _atLineStart(true) {}

  size_t write(uint8_t c) override {
    if (_atLineStart) {
      unsigned long now = micros();
      unsigned long elapsed = (now >= bootMicros) ? (now - bootMicros) : 0;
      char buf[16];
      snprintf(buf, sizeof(buf), "[%lu] ", elapsed);
      _serial->write((const uint8_t*)buf, strlen(buf));
      _atLineStart = false;
    }
    _serial->write(c);
    if (c == '\n') _atLineStart = true;
    return 1;
  }

  void begin(unsigned long baud) { _serial->begin(baud); }
  void flush() { _serial->flush(); }
  int available() { return _serial->available(); }
  int read() { return _serial->read(); }
};

extern TaggedSerial taggedSerial;

// Override Serial to use TaggedSerial globally
#define Serial taggedSerial

#endif  // TAGGED_SERIAL_H
