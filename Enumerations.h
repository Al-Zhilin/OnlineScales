#ifndef ENUMERATIONS_H
#define ENUMERATIONS_H

enum class ScaleErrors : uint8_t {
  SUCCESS,
  NOT_AVAILABLE,
  TARE_ERROR
};

enum class ColorPins : uint8_t {                               // пины, соответствующие каждому цвету (выводу) RGB светодиода
  RED = 5,
  BLUE = 3,
  GREEN = 4
};

enum class BlinkModes : uint8_t {
  TOTAL_ERROR,
  SCALE,
  INFO,
  NONE
};

#endif