#ifndef HIGH_GAIN_VELOCITY_OBSERVER_H
#define HIGH_GAIN_VELOCITY_OBSERVER_H

#include <chrono>

class HighGainVelocityObserver {
  private:
    float velocityEstimate; // Estimated velocity
    float k; // High gain observer gain
    float lastPosition; // Last measured position
    std::chrono::time_point<std::chrono::steady_clock> lastUpdateTime; // Time of last update

  public:
    HighGainVelocityObserver(float k);

    float update(float position);
};

#endif
