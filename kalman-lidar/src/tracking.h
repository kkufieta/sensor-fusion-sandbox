#ifndef TRACKING_H_
#define TRACKING_H_

#include "kalman_filter.h"
#include "measurement_package.h"
#include <fstream>
#include <string>
#include <vector>

class Tracking {
public:
  Tracking();
  virtual ~Tracking();
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  KalmanFilter kf_;

private:
  bool is_initialized_;
  int64_t previous_timestamp_;
  void CalcQ(const float dt);

  // acceleration noise components
  float noise_ax;
  float noise_ay;
};

#endif // TRACKING_H_