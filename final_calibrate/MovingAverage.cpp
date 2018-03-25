#include "MovingAverage.hpp"

MovingAverage::MovingAverage(int numSamples) {
  _numSamples = numSamples;
  _curAccum = 0;
  // Allocate memory for the array that will contain the most recent samples
  // This array will be used as a circular buffer, where we keep track of the
  //   last index that we added in _lastSample and every time we add a new value,
  //   we replace the value at _lastSample and move to the next one
  _samples = new uint16_t[numSamples];
  _lastSample = 0;
}

MovingAverage::~MovingAverage() {
  // Since we allocated it, we need to make sure we deallocate it
  // This doesn't really matter since we never destroy a MovingAverage object,
  //   but it's good to have
  delete[] _samples;
}

uint16_t MovingAverage::GetCurrentAvg() {
  // We keep track of the average before dividing so that we can adjust when
  //   we have a new sample without summing over the entire array
  return _curAccum / _numSamples;
}

uint16_t MovingAverage::AddSample(uint16_t sample) {
  // Get the value that was previously at _lastSample,
  //   subtract it from the accumulator,
  //   add the new sample,
  //   and put the new sample in the array.
  
  uint16_t toRemove = _samples[_lastSample];
  _curAccum = _curAccum + sample - toRemove;
  _samples[_lastSample] = sample;

  // Make sure we advance the index
  _lastSample = (_lastSample + 1) % _numSamples;

  // Return the average, because if we adjusted the moving average
  //   we probably want to know what the value is.
  return GetCurrentAvg();
}

uint16_t MovingAverage::ResetToValue(uint16_t sample) {
  // Reset everything in array to given value
  for(int i = 0; i < _numSamples; i++) {
    _samples[i] = sample;
  }

  // Set accumulator in one line without iterating
  _curAccum = sample * _numSamples;

  // Resetting the _lastSample to 0 isn't necessary, but we might as well
  _lastSample = 0;

  return GetCurrentAvg();
}

