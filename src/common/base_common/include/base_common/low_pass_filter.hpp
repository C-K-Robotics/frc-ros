#ifndef BASE_COMMON__LOW_PASS_FILTER_HPP_
#define BASE_COMMON__LOW_PASS_FILTER_HPP_

#define ERROR_CHECK (false)
#include <math.h>
#include <cmath>
#include <iostream>

class LowPassFilter
{
public:
  // constructors
  LowPassFilter()
  : output(0),
    ePow(0) {}
  LowPassFilter(const double & iCutOffFrequency, const double & iDeltaTime)
  : output(0),
    ePow(1 - exp(-iDeltaTime * 2 * M_PI * iCutOffFrequency))
  {
    #if ERROR_CHECK
    if (iDeltaTime <= 0) {
      std::cout << "Warning: A LowPassFilter instance has been configured with 0 s as delta time.";
      ePow = 0;
    }
    if (iCutOffFrequency <= 0) {
      std::cout <<
        "Warning: A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.";
      ePow = 0;
    }
    #endif
  }
  // functions
  double update(const double & input)
  {
    if (isnan(input)) {
      return output;
    }
    return output += (input - output) * ePow;
  }
  double update(const double & input, const double & deltaTime, const double & cutoffFrequency)
  {
    reconfigureFilter(deltaTime, cutoffFrequency);  // Changes ePow accordingly.
    if (isnan(input)) {
      return output;
    }
    return output += (input - output) * ePow;
  }
  // get and configure funtions
  double getOutput() const {return output;}
  void resetOutput(const double & new_output = 0.0) {output = new_output;}
  void reconfigureFilter(const double & deltaTime, const double & cutoffFrequency)
  {
    #if ERROR_CHECK
    if (deltaTime <= 0) {
      std::cout << "Warning: A LowPassFilter instance has been configured with 0 s as delta time.";
      ePow = 0;
    }
    if (cutoffFrequency <= 0) {
      std::cout <<
        "Warning: A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.";
      ePow = 0;
    }
    #endif
    ePow = 1 - exp(-deltaTime * 2 * M_PI * cutoffFrequency);
  }

private:
  double output;
  double ePow;
};

#endif  // BASE_COMMON__LOW_PASS_FILTER_HPP_
