#pragma once
#include <Arduino.h>

inline void logHelper(const __FlashStringHelper* msg, const char* func, const char* file, int line) {
    #ifdef USE_LOG
        USE_LOG.print(F("> "));
        USE_LOG.print(msg);
        USE_LOG.print(F(" in "));
        USE_LOG.print(func);
        USE_LOG.print(F("() ["));
        USE_LOG.print(file);
        USE_LOG.print(F(" : "));
        USE_LOG.print(line);
        USE_LOG.println(F("]"));
    #endif
}

#ifdef USE_LOG
  #define LOG(x) logHelper(F(x), __FUNCTION__, __FILE__, __LINE__)
#else
  #define LOG(x)
#endif
