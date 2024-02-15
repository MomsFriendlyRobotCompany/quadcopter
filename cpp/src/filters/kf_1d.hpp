
#pragma once

/*
1D Kalman Filter
*/
class KalmanFilter {
  float P{1.0};  // Initial estimate error
  float F{1.0};  // Assuming constant velocity
  float x{1.0};  // state estimate
  float Q;       // processes noise
  float R;       // measurement noise

  public:
  KalmanFilter(float q, float r) {
    Q = q;
    R = r;
  }

  float filter(float z) {
    // Prediction
    float xx = x * F
    float P = P + Q

    // Update
    float K = P / (P + R)
    x = xx + K * (z - xx)
    P = (1.0f - K) * P

    return x
  }
};