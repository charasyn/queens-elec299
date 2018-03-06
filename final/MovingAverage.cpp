#include "MovingAverage.hpp"

MovingAverage::MovingAverage(int numSamples) {
  _numSamples = numSamples;
  _curAccum = 0;
  _lastSample = 0;
  _samples = new uint16_t[numSamples];
}

MovingAverage::~MovingAverage() {
  delete[] _samples;
}

uint16_t MovingAverage::GetCurrentAvg() {
  return _curAccum / _numSamples;
}

uint16_t MovingAverage::AddSample(uint16_t sample) {
  uint16_t toRemove = _samples[_lastSample];
  _curAccum = _curAccum + sample - toRemove;
  _samples[_lastSample] = sample;
  _lastSample = (_lastSample + 1) % _numSamples;
  return GetCurrentAvg();
}

void MovingAverage::ResetToValue(uint16_t sample) {
  for(int i = 0; i < _numSamples; i++) {
    _samples[i] = sample;
  }
  _curAccum = sample * _numSamples;
  _lastSample = 0;
}

