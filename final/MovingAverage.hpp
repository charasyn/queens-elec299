#ifndef MOVINGAVERAGE_HPP
#define MOVINGAVERAGE_HPP

#include <stdint.h>

class MovingAverage {
  public:
    MovingAverage(int numSamples);
    ~MovingAverage();
    uint16_t GetCurrentAvg();
    uint16_t AddSample(uint16_t sample);
    uint16_t ResetToValue(uint16_t sample);
  private:
    int _numSamples;
    int _lastSample;
    uint16_t _curAccum;
    uint16_t * _samples;
};

#endif //MOVINGAVERAGE_HPP
