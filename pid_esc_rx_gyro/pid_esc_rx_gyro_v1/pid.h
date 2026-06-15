#define ROLLING_WINDOW_SIZE 8

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

constexpr PID roll_coeff  = {1, 0.00, 0.00};
constexpr PID pitch_coeff = {1, 0.00, 0.00};
constexpr PID yaw_coeff   = {1, 0.00, 0.00};

constexpr float dt = 0.001;

PID_AXIS pid = {0.0, 0.0, 0.0};

PID updateErrors(float current_control_rate, float current_gyro_rate, float previous_gyro_rate, uint32_t &currentIndex, float (&rolling_window)[ROLLING_WINDOW_SIZE], float &integral)
{
  PID errors = {0.0, 0.0, 0.0};

  float current_error = current_control_rate - current_gyro_rate;
  
  errors.p = current_error;
  errors.d = -(current_gyro_rate - previous_gyro_rate) / dt;

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
    errors.i += current_error;
  }

  return errors;
}

void computePid(float control_roll_rate, float control_pitch_rate, float control_yaw_rate, float gyro_roll_rate, float gyro_pitch_rate, float gyro_yaw_rate)
{
  static PID roll_errors  = {0.0, 0.0, 0.0};
  static PID pitch_errors = {0.0, 0.0, 0.0};
  static PID yaw_errors   = {0.0, 0.0, 0.0};

  static float roll_integral = 0;
  static float pitch_integral = 0;
  static float yaw_integral = 0;

  static PREVIOUS_GYRO previous_gyro_rate = {0.00, 0.00, 0.00};

  static ROT_AXIS currentIndex = {0, 0, 0};
  static ROLLING_WINDOW rolling_window;

  roll_errors  = updateErrors(control_roll_rate,  gyro_roll_rate,   previous_gyro_rate.roll,  currentIndex.roll,  rolling_window.roll,  roll_integral);
  pitch_errors = updateErrors(control_pitch_rate, gyro_pitch_rate,  previous_gyro_rate.pitch, currentIndex.pitch, rolling_window.pitch, pitch_integral);
  yaw_errors   = updateErrors(control_yaw_rate,   gyro_yaw_rate,    previous_gyro_rate.yaw,  currentIndex.yaw,    rolling_window.yaw,   yaw_integral);

  previous_gyro_rate.roll  = gyro_roll_rate;
  previous_gyro_rate.pitch = gyro_pitch_rate;
  previous_gyro_rate.yaw   = gyro_yaw_rate;

  pid.roll  = roll_coeff.p * roll_errors.p   + roll_coeff.i * roll_errors.i   + roll_coeff.d * roll_errors.d;
  pid.pitch = pitch_coeff.p * pitch_errors.p + pitch_coeff.i * pitch_errors.i + pitch_coeff.d * pitch_errors.d;
  pid.yaw   = yaw_coeff.p * yaw_errors.p     + yaw_coeff.i * yaw_errors.i     + yaw_coeff.d * yaw_errors.d;
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