#ifndef ERROR_CODES_H
#define ERROR_CODES_H

enum class ScaleErrors : uint8_t {
  SUCCESS = 0,
  TIMEOUT_ERROR = 1,
  NEED_RECALL_FUNCTION = 2,
}

enum class LedBlink : uint8_t {
  SCALE_SUCCESS = 0,
  SCALE_TIMEOUT = 1,
  SCALE_WAITING = 2,
}

#endif