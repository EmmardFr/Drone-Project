// Moving integral
// #define ROLLING_WINDOW_SIZE 256
#define ROLLING_WINDOW_SIZE 0
// Accumulating integral + safety limit
#define I_LIMIT 30.0f

struct PID_AXIS{
  float roll;
  float pitch;
  float yaw;
};

struct PREVIOUS_GYRO{
  float roll;
  float pitch;
  float yaw;
};

struct ROT_AXIS{
  uint32_t roll;
  uint32_t pitch;
  uint32_t yaw;
};

struct PID{
  float p;
  float i;
  float d;
};

struct ROLLING_WINDOW{
  float roll[ROLLING_WINDOW_SIZE];
  float pitch[ROLLING_WINDOW_SIZE];
  float yaw[ROLLING_WINDOW_SIZE];
};

// constexpr PID roll_coeff  = {1.10, 2.50, 0.0001}; // 
// constexpr PID pitch_coeff = {1.10, 1.90, 0.0001}; // 
// constexpr PID yaw_coeff   = {1.10, 1.50, 0.0001}; //

constexpr PID roll_coeff  = {1.20, 4.00, 0.045}; // 
constexpr PID pitch_coeff = {1.10, 4.00, 0.045}; // 
constexpr PID yaw_coeff   = {1.40, 2.20, 0.04}; //

// constexpr PID roll_coeff  = {1.10, 4.00, 0.04}; // good...
// constexpr PID pitch_coeff = {1.10, 4.00, 0.04}; // good!

// constexpr float dt = 0.001;

PID_AXIS pid = {0.0, 0.0, 0.0};

PID roll_pid  = {0.00, 0.00, 0.00};
PID pitch_pid = {0.00, 0.00, 0.00};
PID yaw_pid   = {0.00, 0.00, 0.00};

constexpr float D_CUTOFF_HZ = 10.0f;
constexpr float RC = 1.0f / (2.0f * PI * D_CUTOFF_HZ);

PID updateErrors(float current_control_rate, float current_gyro_rate, float previous_gyro_rate, uint32_t &currentIndex, float (&rolling_window)[ROLLING_WINDOW_SIZE], float &integral, float &d_filtered, float dt)
{
  PID errors = {0.0, 0.0, 0.0};

  float current_error = current_control_rate - current_gyro_rate;
  
  errors.p = current_error;

  // errors.d = -(current_gyro_rate - previous_gyro_rate) / dt;
  float alpha = dt / (RC + dt);
  float raw_d = -(current_gyro_rate - previous_gyro_rate) / dt;
  d_filtered += alpha * (raw_d - d_filtered);
  errors.d = d_filtered;

  if(ROLLING_WINDOW_SIZE != 0)
  {
    float oldest_error = rolling_window[currentIndex];
    rolling_window[currentIndex] = current_error * dt;
    currentIndex = (currentIndex + 1) % ROLLING_WINDOW_SIZE;
    errors.i = integral + current_error * dt - oldest_error;
    integral = errors.i;
  }
  else
  {
    integral += current_error * dt;
    integral = constrain(integral, -I_LIMIT, I_LIMIT);

    errors.i = integral;
  }

  return errors;
}

void computePid(float control_roll_rate, float control_pitch_rate, float control_yaw_rate, float gyro_roll_rate, float gyro_pitch_rate, float gyro_yaw_rate, float dt)
{
  static PID roll_errors  = {0.0, 0.0, 0.0};
  static PID pitch_errors = {0.0, 0.0, 0.0};
  static PID yaw_errors   = {0.0, 0.0, 0.0};

  static float roll_integral = 0;
  static float pitch_integral = 0;
  static float yaw_integral = 0;

  static float roll_d_filtered  = 0.0f;
  static float pitch_d_filtered = 0.0f;
  static float yaw_d_filtered   = 0.0f;

  static PREVIOUS_GYRO previous_gyro_rate = {gyro_roll_rate, gyro_pitch_rate, gyro_yaw_rate};

  static ROT_AXIS currentIndex = {0, 0, 0};
  static ROLLING_WINDOW rolling_window;

  dt = constrain(dt, 0.0005f, 0.005f); // to avoid crazy i and d terms when dt is occasionnally getting big

  roll_errors  = updateErrors(control_roll_rate,  gyro_roll_rate,   previous_gyro_rate.roll,  currentIndex.roll,  rolling_window.roll,  roll_integral,  roll_d_filtered,  dt);
  pitch_errors = updateErrors(control_pitch_rate, gyro_pitch_rate,  previous_gyro_rate.pitch, currentIndex.pitch, rolling_window.pitch, pitch_integral, pitch_d_filtered, dt);
  yaw_errors   = updateErrors(control_yaw_rate,   gyro_yaw_rate,    previous_gyro_rate.yaw,  currentIndex.yaw,    rolling_window.yaw,   yaw_integral,   yaw_d_filtered,   dt);

  previous_gyro_rate.roll  = gyro_roll_rate;
  previous_gyro_rate.pitch = gyro_pitch_rate;
  previous_gyro_rate.yaw   = gyro_yaw_rate;

  roll_pid  = {roll_coeff.p * roll_errors.p,   roll_coeff.i * roll_errors.i,   roll_coeff.d * roll_errors.d};
  pitch_pid = {pitch_coeff.p * pitch_errors.p, pitch_coeff.i * pitch_errors.i, pitch_coeff.d * pitch_errors.d};
  yaw_pid   = {yaw_coeff.p * yaw_errors.p,     yaw_coeff.i * yaw_errors.i,     yaw_coeff.d * yaw_errors.d};

  pid.roll  = roll_pid.p  + roll_pid.i  + roll_pid.d;
  pid.pitch = pitch_pid.p + pitch_pid.i + pitch_pid.d;
  pid.yaw   = yaw_pid.p   + yaw_pid.i   + yaw_pid.d;
}

void printPID(bool doIt = true)
{
  if(doIt)
  {
    Serial.print(pid.roll);
    Serial.print('\t');
    Serial.print(pid.pitch);
    Serial.print('\t');
    Serial.println(pid.yaw);
  }
}