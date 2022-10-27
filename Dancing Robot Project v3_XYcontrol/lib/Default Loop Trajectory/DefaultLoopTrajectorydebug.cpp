#include <TrajectoryTracking.h>

void configure_imu() {
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      ledcWriteNote(BUZZ_CHANNEL, NOTE_C, 4);
      delay(500);
      ledcWriteNote(BUZZ_CHANNEL, NOTE_G, 4);
      delay(500);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
}

void read_imu(float& w_z) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  w_z = g.gyro.z;
}

void est_imu_bias(float& E_w_z, int N_samples) {
  float E_w_z_acc = 0.0;
  for (unsigned int i = 0; i < N_samples; i++) {
    float w_z;
    read_imu(w_z);
    E_w_z_acc += w_z;
    delay(5);
  }
  E_w_z = E_w_z_acc / N_samples;
}

void configure_motor_pins() {
  ledcSetup(M1_IN_1_CHANNEL, M_PWM_FREQ, M_PWM_BITS);
  ledcSetup(M1_IN_2_CHANNEL, M_PWM_FREQ, M_PWM_BITS);
  ledcSetup(M2_IN_1_CHANNEL, M_PWM_FREQ, M_PWM_BITS);
  ledcSetup(M2_IN_2_CHANNEL, M_PWM_FREQ, M_PWM_BITS);

  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);
}

// Positive means forward, negative means backwards
void set_motors_pwm(float left_pwm, float right_pwm) {
  if (isnan(left_pwm)) left_pwm = 0.0;
  if (left_pwm  >  255.0) left_pwm  =  255.0;
  if (left_pwm  < -255.0) left_pwm  = -255.0;
  if (isnan(right_pwm)) right_pwm = 0.0;
  if (right_pwm >  255.0) right_pwm =  255.0;
  if (right_pwm < -255.0) right_pwm = -255.0;

  if (left_pwm > 0) {
    ledcWrite(M1_IN_1_CHANNEL, 0);
    ledcWrite(M1_IN_2_CHANNEL, (uint32_t)(left_pwm));
  } else {
    ledcWrite(M1_IN_1_CHANNEL, (uint32_t)-left_pwm);
    ledcWrite(M1_IN_2_CHANNEL, 0);
  }

  if (right_pwm > 0) {
    ledcWrite(M2_IN_1_CHANNEL, 0);
    ledcWrite(M2_IN_2_CHANNEL, (uint32_t)(right_pwm));
  } else {
    ledcWrite(M2_IN_1_CHANNEL, (uint32_t)-right_pwm);
    ledcWrite(M2_IN_2_CHANNEL, 0);
  }
}

float update_pid(float dt, float kp, float ki, float kd,
                 float x_d, float x,
                 float& int_e, float abs_int_e_max, // last_x and int_e are updated by this function
                 float& last_x) {
  // Calculate or update intermediates
  float e = x_d - x; // Error

  // Integrate error with anti-windup
  int_e = int_e + e * dt;
  if (int_e >  abs_int_e_max) int_e =  abs_int_e_max;
  if (int_e < -abs_int_e_max) int_e = -abs_int_e_max;

  // Take the "Derivative of the process variable" to avoid derivative spikes if setpoint makes step change
  // with abuse of notation, call this de
  float de = -(x - last_x) / dt;
  last_x = x;

  float u = kp * e + ki * int_e + kd * de;
  return u;
}

// a smooth and interesting trajectory
// https://en.wikipedia.org/wiki/Lemniscate_of_Bernoulli
void CIRCLE(float t, float a, float& x, float& y) {
  //float sin_t = sin(t);
  //float den = 1 + sin_t * sin_t;
  x = a * cos(t);
  y = a * sin(t);
}

void BACKCIRCLE(float t, float a, float& x, float& y) {
  //float sin_t = sin(t);
  //float den = 1 + sin_t * sin_t;
  x = a * cos(t);
  y = -a * sin(t);
}

void SPIRAL(float t, float a, float& x, float& y) {
  //float sin_t = sin(t);
  //float den = 1 + sin_t * sin_t;
  x = a * exp(t/20)*cos(t);
  y = -a * exp(t/20)*sin(t);
}

void ZIGZAG(float t, float a, float& x, float& y) {
  //float sin_t = sin(t);
  //float den = 1 + sin_t * sin_t;
  x = a * cos(t);
  y = -t;
}

void DIAMOND(float t, float a, float& x, float& y) {
  //float sin_t = sin(t);
  //float den = 1 + sin_t * sin_t;
  x = a * pow(cos(t),3);
  y = -a * pow(sin(t),3);
}

void STAR(float t, float a, float& x, float& y) {
  //float sin_t = sin(t);
  //float den = 1 + sin_t * sin_t;
  x = -(abs(a)+4)*sin(2*t) - a*sin(3*t);
  y = (abs(a)+4)*cos(2*t) - a*cos(3*t);
}

void STOP(float& x, float& y) {
  x = 0;
  y = 0;
}

// Signed angle from (x0, y0) to (x1, y1)
// assumes norms of these quantities are precomputed
float signed_angle(float x0, float y0, float n0, float x1, float y1, float n1) {
  float normed_dot = (x1 * x0 + y1 * y0) / (n1 * n0);
  if (normed_dot > 1.0) normed_dot = 1.0; // Possible because of numerical error
  float angle = acosf(normed_dot);
  
  // use cross product to find direction of rotation
  // https://en.wikipedia.org/wiki/Cross_product#Coordinate_notation
  float s3 = x0 * y1 - x1 * y0;
  if (s3 < 0) angle = -angle;

  return angle;
}

void setDefault_Trajectory() {
  Serial.begin(115200);

  // Disalbe the lightbar ADC chips so they don't hold the SPI bus used by the IMU
  pinMode(ADC_1_CS, OUTPUT);
  pinMode(ADC_2_CS, OUTPUT);
  digitalWrite(ADC_1_CS, HIGH);
  digitalWrite(ADC_2_CS, HIGH);

  ledcAttachPin(BUZZ, BUZZ_CHANNEL);

  pinMode(VCC_SENSE, INPUT);

  configure_motor_pins();
  configure_imu();

  Serial.println("Starting!");
}

float* defaultLoop(Encoder& enc1, Encoder& enc2, int check, int mode) {
  // Create the encoder objects after the motor has
  // stopped, else some sort exception is triggered
  static int target_period_ms;
  static float leminscate_a;
  static float leminscate_t_scale;
  static float x0, y0;//if something goes wrong, might have to do with x0y0 and last_xlast_y
  static float last_x, last_y;
  static float last_dx;
  static float last_dy;
  static float last_target_v;
  static float target_theta;
  static float kp_left;
  static float ki_left;
  static float kd_left;
  static float kf_left;
  static float target_pos_left;
  static float last_pos_left;
  static float integral_error_pos_left;
  static float max_integral_error_pos_left; // Max effect is the nominal battery voltage

  static float kp_right;
  static float ki_right;
  static float kd_right;
  static float kf_right;
  static float last_pos_right;
  static float target_pos_right;
  static float integral_error_pos_right;
  static float max_integral_error_pos_right; // Max effect is the nominal battery voltage
  static float theta;
  static float bias_omega;
  static float ktheta;
  static float start_t;
  static float last_t;
  if (check == 0) {
  // Loop period
    target_period_ms = 2; // Loop takes about 3 ms so a delay of 2 gives 200 Hz or 5ms

  // States used to calculate target velocity and heading
    leminscate_a = 0.5; // Radius
    leminscate_t_scale = 2.0; // speedup factor

    switch(mode) {
   case 0  :
      STOP(x0, y0);
      STOP(last_x, last_y);
      Serial.print("stopstart\n");
      break;
   case 1  :
      CIRCLE(0.0, leminscate_a, x0, y0);
      CIRCLE(-leminscate_t_scale * target_period_ms / 1000.0, leminscate_a, last_x, last_y);
      Serial.print("circlestart\n");
      break;
   case 2  :
      BACKCIRCLE(0.0, leminscate_a, x0, y0);
      BACKCIRCLE(-leminscate_t_scale * target_period_ms / 1000.0, leminscate_a, last_x, last_y);
      Serial.print("backstart\n");
      break;
   case 3  :
      SPIRAL(0.0, leminscate_a, x0, y0);
      SPIRAL(-leminscate_t_scale * target_period_ms / 1000.0, leminscate_a, last_x, last_y);
      Serial.print("spiralstart\n");
      break;
   case 4  :
      ZIGZAG(0.0, leminscate_a, x0, y0);
      ZIGZAG(-leminscate_t_scale * target_period_ms / 1000.0, leminscate_a, last_x, last_y);
      Serial.print("zigstart\n");
      break;
   case 5  :
      DIAMOND(0.0, leminscate_a, x0, y0);
      DIAMOND(-leminscate_t_scale * target_period_ms / 1000.0, leminscate_a, last_x, last_y);
      Serial.print("diamondstart\n");
      break;
   case 6  :
      STAR(0.0, leminscate_a, x0, y0);
      STAR(-leminscate_t_scale * target_period_ms / 1000.0, leminscate_a, last_x, last_y);
      Serial.print("starstart\n");
      break;
}
    
    last_dx = (x0 - last_x) / ((float)target_period_ms / 1000.0);
    last_dy = (y0 - last_y) / ((float)target_period_ms / 1000.0);
    last_target_v = sqrtf(last_dx * last_dx + last_dy * last_dy);
    target_theta = 0.0; // This is an integrated quantity

  // Motors are controlled by a position PID
  // with inputs interpreted in meters and outputs interpreted in volts
  // integral term has "anti-windup"
  // derivative term uses to derivative of process variable (wheel position)
  // instead of derivative of error in order to avoid "derivative kick"
    kp_left = 200.0;
    ki_left = 20.0;
    kd_left = 20.0;
    kf_left = 10.0;
    target_pos_left  = 0.0;
    last_pos_left = 0.0;
    integral_error_pos_left = 0.0;
    max_integral_error_pos_left = 1.0 * 8.0 / ki_left; // Max effect is the nominal battery voltage

    kp_right = 200.0;
    ki_right = 20.0;
    kd_right = 20.0;
    kf_right = 10.0;
    last_pos_right = 0.0;
    target_pos_right = 0.0;
    integral_error_pos_right = 0.0;
    max_integral_error_pos_right = 1.0 * 8.0 / ki_right; // Max effect is the nominal battery voltage

  // IMU Orientation variables
    theta = 0.0;
    //bias_omega;
  // Gain applied to heading error when offseting target motor velocities
  // currently set to 360 deg/s compensation for 90 degrees of error
    ktheta = (2 * 3.14159) / (90.0 * 3.14159 / 180.0);
    est_imu_bias(bias_omega, 500);// Could be expanded for more quantities

  // The real "loop()"
  // time starts from 0
    start_t = (float)micros() / 1000000.0;
    last_t = -target_period_ms / 1000.0; // Offset by expected looptime to avoid divide by zero
  }


    // Get the time elapsed
    float t = ((float)micros()) / 1000000.0 - start_t;
    float dt = ((float)(t - last_t)); // Calculate time since last update
    // Serial.print("t "); Serial.print(t);
    Serial.print(" dt "); Serial.print(dt * 1000.0);
    last_t = t;

    // Get the distances the wheels have traveled in meters
    // positive is forward
    float pos_left  =  (float)enc1.read() * METERS_PER_TICK;
    float pos_right = -(float)enc2.read() * METERS_PER_TICK; // Take negative because right counts upwards when rotating backwards
  
    // TODO Battery voltage compensation, the voltage sense on my mouse is broken for some reason
    // int counts = analogRead(VCC_SENSE);
    // float battery_voltage = counts * ADC_COUNTS_TO_VOLTS;
    // if (battery_voltage <= 0) Serial.println("BATTERY INVALID");
  
    // Read IMU and update estimate of heading
    // positive is counter clockwise
    float omega;
    read_imu(omega); // Could be expanded to read more things
    omega -= bias_omega; // Remove the constant bias measured in the beginning
    theta = theta + omega * dt;
    // Serial.print(" omega "); Serial.print(omega);
    // Serial.print(" theta "); Serial.print(theta);

    // Serial.print(" last_x "); Serial.print(last_x);
    // Serial.print(" last_y "); Serial.print(last_y);
    // Serial.print(" last_dx "); Serial.print(last_dx);
    // Serial.print(" last_dy "); Serial.print(last_dy);
    // Serial.print(" last tv "); Serial.print(last_target_v);

    // Calculate target forward velocity and target heading to track the leminscate trajectory
    // of 0.5 meter radius
    float x, y;
  switch(mode) {
   case 0  :
      STOP(x,y);
      Serial.print("stop\n");
      break;
   case 1  :
      CIRCLE(leminscate_t_scale * t, leminscate_a, x, y);
      Serial.print("circle\n");
      break;
   case 2  :
      BACKCIRCLE(leminscate_t_scale * t, leminscate_a, x, y);
      Serial.print("back\n");
      break;
   case 3  :
      SPIRAL(leminscate_t_scale * t, leminscate_a, x, y);
      Serial.print("spiral\n");
   case 4  :
      ZIGZAG(leminscate_t_scale * t, leminscate_a, x, y);
      Serial.print("zig\n");
      break;
   case 5  :
      DIAMOND(leminscate_t_scale * t, leminscate_a, x, y);
      Serial.print("diamond\n");
      break;
   case 6  :
      STAR(leminscate_t_scale * t, leminscate_a, x, y);
      Serial.print("star\n");
      break;
}

    // Serial.print(" x "); Serial.print(x);
    // Serial.print(" y "); Serial.print(y);

    float dx = (x - last_x) / dt;
    float dy = (y - last_y) / dt;
    float target_v = sqrtf(dx * dx + dy * dy); // forward velocity

    // Serial.print(" dx "); Serial.print(dx);
    // Serial.print(" dy "); Serial.print(dy);
    // Serial.print(" tv "); Serial.print(target_v);

    // Compute the change in heading using the normalized dot product between the current and last velocity vector
    // using this method instead of atan2 allows easy smooth handling of angles outsides of -pi / pi at the cost of
    // a slow drift defined by numerical precision
    float target_omega = signed_angle(last_dx, last_dy, last_target_v, dx, dy, target_v) / dt;
    target_theta = target_theta + target_omega * dt;

    // Serial.print(" target_omega "); Serial.print(target_omega);
    // Serial.print(" t theta "); Serial.print(target_theta);

    last_x = x;
    last_y = y;
    last_dx = dx;
    last_dy = dy;
    last_target_v = target_v;
  
    // Calculate target motor speeds from target forward speed and target heading
    // Could also include target path length traveled and target angular velocity
    float error_theta_z = target_theta - theta;
    float requested_v = target_v;
    float requested_w = ktheta * error_theta_z;

    float target_v_left  = requested_v - TURNING_RADIUS_METERS * requested_w;
    float target_v_right = requested_v + TURNING_RADIUS_METERS * requested_w;
    target_pos_left  = target_pos_left  + dt * target_v_left;
    target_pos_right = target_pos_right + dt * target_v_right;

    // Serial.print(" tpl "); Serial.print(target_pos_left);
    // Serial.print(" pl "); Serial.print(pos_left);
    // Serial.print(" tpr "); Serial.print(target_pos_right);
    // Serial.print(" pr "); Serial.print(pos_right);

    // Left motor position PID
    float left_voltage = update_pid(dt, kp_left, ki_left, kd_left,
                                    target_pos_left, pos_left,
                                    integral_error_pos_left, max_integral_error_pos_left,
                                    last_pos_left);
    left_voltage = left_voltage + kf_left * target_v_left;
    float left_pwm = (float)MAX_PWM_VALUE * (left_voltage / 8.0); // TODO use actual battery voltage

    // Right motor position PID
    float right_voltage = update_pid(dt, kp_right, ki_right, kd_right,
                                     target_pos_right, pos_right,
                                     integral_error_pos_right, max_integral_error_pos_right,
                                     last_pos_right);
    left_voltage = right_voltage + kf_right * target_v_right;
    float right_pwm = (float)MAX_PWM_VALUE * (right_voltage / 8.0); // TODO use actual battery voltage

  // Serial.print(" l voltage " ); Serial.print(left_voltage);
  // Serial.print(" r voltage " ); Serial.print(right_voltage);

  set_motors_pwm(left_pwm, right_pwm);

  // Serial.println();
  delay(target_period_ms);
  static float r[2];
  r[0] = requested_v;
  r[1] = requested_w;
  return r;
}