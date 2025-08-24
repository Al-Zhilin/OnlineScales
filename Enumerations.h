#ifndef ENUMERATIONS_H
#define ENUMERATIONS_H

enum class ScaleErrors : uint8_t {
  SUCCESS,
  TIMEOUT_ERROR,
  NEED_RECALL
};

enum class ColorPins : uint8_t {                               // пины, соответствующие каждому цвету (выводу) RGB светодиода
  RED = 5,
  BLUE = 3,
  GREEN = 4
};

enum class BlinkModes : uint8_t {
  TOTAL_ERROR,
  SCALE,
  INFO
};

#endif