#pragma once
// Arduino / ESP32 platform glue for afec::Node.
#include <Arduino.h>
#include "afec_node.h"

namespace afec {
inline uint32_t arduinoMillis()          { return millis(); }
inline uint32_t arduinoRandom(uint32_t n) { return n ? (uint32_t)random((long)n) : 0; }
inline void     arduinoYield()           { yield(); }

inline Platform arduinoPlatform() {
    Platform p; p.millis = arduinoMillis; p.random = arduinoRandom; p.yield = arduinoYield; return p;
}
} // namespace afec
