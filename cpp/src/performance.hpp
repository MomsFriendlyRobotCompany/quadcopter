
#pragma once

#include <cstdlib>
#include <cmath>

/**
 This tracks any failures and reports periodically.

 mavlink sys_status has errors
 mavlink log_data hold 90bytes
*/
struct Performance {
  uint32_t i2c_err{0};
  uint32_t mutex_err{0};
  uint32_t imu_err{0};
  uint32_t mag_err{0};
  uint32_t pt_err{0};
  uint32_t gps_err{0};
  uint32_t loop_late_err{0};
};