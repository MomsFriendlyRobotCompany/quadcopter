/*
Add some filters here
*/
#pragma once

/*
class kalman_filter:
  def __init__(self, process_variance, measurement_variance):
    self.estimate_error = 1.0  # Initial estimate error
    self.state_transition = 1.0  # Assuming constant velocity
    self.state_estimate = 1.0
    self.process_variance = process_variance
    self.measurement_variance = measurement_variance

  def filter(self,measurement):
    # Prediction
    prediction = self.state_estimate * self.state_transition
    prediction_error = self.estimate_error + self.process_variance

    # Update
    kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
    self.state_estimate = prediction + kalman_gain * (measurement - prediction)
    self.estimate_error = (1 - kalman_gain) * prediction_error

    return self.state_estimate
*/

class KalmanFilter {
  float P{1.0};  // Initial estimate error
  float F{1.0};  // Assuming constant velocity
  float x{1.0};
  float Q;
  float R;

  public:
  KalmanFilter(float q, float r) {
    Q = q;
    R = r;
  }

  float filter(self,z) {
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