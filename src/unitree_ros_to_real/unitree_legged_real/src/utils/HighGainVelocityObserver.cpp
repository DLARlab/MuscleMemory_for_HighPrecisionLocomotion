#include "HighGainVelocityObserver.h"

using namespace std;
using namespace std::chrono;

HighGainVelocityObserver::HighGainVelocityObserver(float k) {
  this->k = k;
  this->velocityEstimate = 0;
  this->lastPosition = 0;
  this->lastUpdateTime = steady_clock::now();
}

float HighGainVelocityObserver::update(float position) {
  auto now = steady_clock::now();
  float dt = duration_cast<duration<double>>(now - this->lastUpdateTime).count();
  float velocityError = position - this->lastPosition;
  this->velocityEstimate += this->k * velocityError / dt;
  this->lastPosition = position;
  this->lastUpdateTime = now;
  return this->velocityEstimate;
}
