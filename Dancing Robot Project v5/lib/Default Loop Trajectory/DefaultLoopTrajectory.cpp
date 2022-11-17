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
void CIRCLE(float t, float a, float& x, float& y, float &axff, float &ayff) {
  //float sin_t = sin(t);
  //float den = 1 + sin_t * sin_t;
  x = a * sin(t);
  y = -a * cos(t)+ a;
  //set axff and ayff to the double derivatives of the positional functions
  axff = -axff*axff*a*sin(t);
  ayff = ayff*ayff*a*cos(t);
}

void BACKCIRCLE(float t, float a, float& x, float& y, float &axff, float &ayff) {
  //float sin_t = sin(t);
  //float den = 1 + sin_t * sin_t;
  x = -a * sin(t);
  y = a * cos(t) - a;
  axff = axff*axff*a*sin(t);
  ayff = -ayff*ayff*a*cos(t);
}

void SPIRAL(float t, float a, float& x, float& y, float &axff, float &ayff) {
  //float sin_t = sin(t);
  //float den = 1 + sin_t * sin_t;
  x = a * exp(t/20)*cos(t) - a;
  y = -a * exp(t/20)*sin(t);
  axff = a * (-(axff*axff*(1/20)*((1/20) - 20)*exp(t/20)*cos(t)) - (axff*axff*(1/10))*exp(t/20)*sin(t));
  ayff = -a * (-(ayff*ayff*(1/20)*((1/20) - 20)*exp(t/20)*sin(t)) + (ayff*ayff*(1/10))*exp(t/20)*cos(t));
}

void ZIGZAG(float t, float a, float& x, float& y, float &axff, float &ayff) {
  //float sin_t = sin(t);
  //float den = 1 + sin_t * sin_t;
  x = t;
  y = a*sin(t);
  axff = 0;
  ayff = -ayff*ayff*a*sin(t);
}

void DIAMOND(float t, float a, float& x, float& y, float &axff, float &ayff) {
  //float sin_t = sin(t);
  //float den = 1 + sin_t * sin_t;
  x = a * pow(cos(t),3) - a;
  y = -a * pow(sin(t),3);
  axff = -a * axff * axff * 3 * ((-2*sin(t)*sin(t)*cos(t)) + (pow(cos(t),3)));
  ayff = -a * ayff * ayff * 3 * ((-2*cos(t)*cos(t)*sin(t)) + (pow(sin(t),3)));
}

void STAR(float t, float a, float& x, float& y, float &axff, float &ayff) {
  //float sin_t = sin(t);
  //float den = 1 + sin_t * sin_t;
  x = -(a+1)*sin(2*t) - a*sin(3*t);
  y = (a+1)*cos(2*t) - a*cos(3*t) - 1;
  axff = (abs(a)+4)*4*axff*axff*sin(2*t) + a*9*axff*axff*sin(3*t);
  ayff = (abs(a)+4)*-4*ayff*ayff*cos(2*t) + a*9*ayff*ayff*cos(3*t);
}

void STOP(float& x, float& y, float &axff, float &ayff, float posx, float posy) {
  x = posx;
  y = posy;
  axff = 0;
  ayff = 0;
}

void LINE(float& x, float& y, float& newx, float& newy, float &axff, float &ayff) {
  x = newx;
  y = newy;
  axff = 0;
  ayff = 0;
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

float* defaultLoop(Encoder& enc1, Encoder& enc2, int check, int mode, float posx, float posy, float newx, float newy, float &x, float &y) {
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

  static float integral_error_pos_x;      //for new XY control PID
  static float max_integral_error_pos_x;  //for new XY control PID

  static float integral_error_pos_y;      //for new XY control PID
  static float max_integral_error_pos_y;  //for new XY control PID
  static float target_v;
  static float dummy_axff, dummy_ayff;    //unused variables

  if (check == 0) {
  // Loop period
    target_period_ms = 2; // Loop takes about 3 ms so a delay of 2 gives 200 Hz or 5ms

  // States used to calculate target velocity and heading
    leminscate_a = 0.5; // Radius, might wanna chance to 0.5
    leminscate_t_scale = 1.11408460164; // speedup factor, 7/2pi

    switch(mode) {
   case 0  :
      STOP(x0, y0, dummy_axff, dummy_ayff, posx, posy);
      STOP(last_x, last_y, dummy_axff, dummy_ayff, posx, posy);
      break;
   case 1  :
      CIRCLE(0.0, leminscate_a, x0, y0, dummy_axff, dummy_ayff);
      CIRCLE(-leminscate_t_scale * target_period_ms / 1000.0, leminscate_a, last_x, last_y, dummy_axff, dummy_ayff);
      break;
   case 2  :
      BACKCIRCLE(0.0, leminscate_a, x0, y0, dummy_axff, dummy_ayff);
      BACKCIRCLE(-leminscate_t_scale * target_period_ms / 1000.0, leminscate_a, last_x, last_y, dummy_axff, dummy_ayff);
      break;
   case 3  :
      SPIRAL(0.0, leminscate_a, x0, y0, dummy_axff, dummy_ayff);
      SPIRAL(-leminscate_t_scale * target_period_ms / 1000.0, leminscate_a, last_x, last_y, dummy_axff, dummy_ayff);
      break;
   case 4  :
      ZIGZAG(0.0, leminscate_a, x0, y0, dummy_axff, dummy_ayff);
      ZIGZAG(-leminscate_t_scale * target_period_ms / 1000.0, leminscate_a, last_x, last_y, dummy_axff, dummy_ayff);
      break;
   case 5  :
      DIAMOND(0.0, leminscate_a, x0, y0, dummy_axff, dummy_ayff);
      DIAMOND(-leminscate_t_scale * target_period_ms / 1000.0, leminscate_a, last_x, last_y, dummy_axff, dummy_ayff);
      break;
   case 6  :
      STAR(0.0, leminscate_a, x0, y0, dummy_axff, dummy_ayff);
      STAR(-leminscate_t_scale * target_period_ms / 1000.0, leminscate_a, last_x, last_y, dummy_axff, dummy_ayff);
      break;
    case 7 :
      LINE(x0,y0,newx, newy, dummy_axff, dummy_ayff);
      LINE(last_x,last_y,newx,newy, dummy_axff, dummy_ayff);
}
    
    last_dx = (x0 - last_x) / ((float)target_period_ms / 1000.0);
    last_dy = (y0 - last_y) / ((float)target_period_ms / 1000.0);
    last_target_v = sqrtf(last_dx * last_dx + last_dy * last_dy);
    target_v = 0.55704230082;
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

    integral_error_pos_x = 0.0;
    max_integral_error_pos_x = 1.0 * 8.0 / 20.0;
    integral_error_pos_y = 0.0;
    max_integral_error_pos_y = 1.0 * 8.0 / 20.0;

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
    
    last_t = t;

    // Get the distances the wheels have traveled in meters
    // positive is forward
    float pos_left  =  (float)enc1.read() * METERS_PER_TICK;
    float pos_right = -(float)enc2.read() * METERS_PER_TICK; // Take negative because right counts upwards when rotating backwards
  
  
  
    // Read IMU and update estimate of heading
    // positive is counter clockwise
    float omega;
    read_imu(omega); // Could be expanded to read more things
    omega -= bias_omega; // Remove the constant bias measured in the beginning
    theta = theta + omega * dt;

    // Calculate target forward velocity and target heading to track the leminscate trajectory
    // of 0.5 meter radius
    //float x, y; supposedly these are passed in by reference now
    float axff = leminscate_t_scale;  //this temp acceleration value is critical for calculating acceleration
    float ayff = leminscate_t_scale;
  switch(mode) {
   case 0  :
      STOP(x,y, axff, ayff, posx, posy);
      break;
   case 1  :
      CIRCLE(leminscate_t_scale * t, leminscate_a, x, y, axff, ayff);
      break;
   case 2  :
      BACKCIRCLE(leminscate_t_scale * t, leminscate_a, x, y, axff, ayff);
      break;
   case 3  :
      SPIRAL(leminscate_t_scale * t, leminscate_a, x, y, axff, ayff);
   case 4  :
      ZIGZAG(leminscate_t_scale * t, leminscate_a, x, y, axff, ayff);
      break;
   case 5  :
      DIAMOND(leminscate_t_scale * t, leminscate_a, x, y, axff, ayff);
      break;
   case 6  :
      STAR(leminscate_t_scale * t, leminscate_a, x, y, axff, ayff);
      break;
    case 7 :
      LINE(x,y,newx,newy, axff, ayff);
      break;
}        
    float dx = (x - last_x) / dt;
    float dy = (y - last_y) / dt;

    //accelerations for new XY control
    float ax = update_pid(dt, 2.0, 2.0, 2.0, x, posx, integral_error_pos_x, max_integral_error_pos_x, last_x); //variables might 
    float ay = update_pid(dt, 2.0, 2.0, 2.0, y, posy, integral_error_pos_y, max_integral_error_pos_x, last_y); //need to be updated
    ax = axff;
    ay = ayff;
    //ax should equal update_pid + a feedworward, update move functions to supply a target acceleration (like the ones you have below)
    //and then add those together for ax, ay
    //ax = -0.6205922498*sin(1.11408460164*t);
    //ay = 0.6205922498*cos(1.11408460164*t);
  //TEST SOME SAMPLE AX AND AY VALUES
    //ax = double derivative of costx wrt t
    //ay = double derivative of sintx wrt t
    
    
    

    //Feedback linearization for new XY control
    float dv = (cos(target_theta)*ax + sin(target_theta)*ay); //might want to swap target_theta to theta and target_v to v_measured
    target_v = target_v + dv * dt;
    if (target_v == 0.0) {
      target_v = 0.0000001;
    }
    if (target_v > 2)  {
      target_v = 2;
    }
    float v_measured_left = ((pos_left - last_pos_left)/dt);
    float v_measured_right = ((pos_right - last_pos_right)/dt);
    float v_measured = (v_measured_left + v_measured_right)/2;
    if (v_measured < 0.02)  {
      v_measured = 0.02;
    }
    float target_omega = (-1/(target_v))*(sin(target_theta)*ax - cos(target_theta)*ay);
    Serial.print(target_v);
    Serial.print("\t");
    Serial.print(target_omega);
    Serial.println();

    if (mode == 0) {
        target_v = 0.0;
    }
    // Compute the change in heading using the normalized dot product between the current and last velocity vector
    // using this method instead of atan2 allows easy smooth handling of angles outsides of -pi / pi at the cost of
    // a slow drift defined by numerical precision
    //float target_omega = signed_angle(last_dx, last_dy, last_target_v, dx, dy, target_v) / dt; //old target omega
    target_theta = target_theta + target_omega * dt;


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


  set_motors_pwm(left_pwm, right_pwm);

  // Serial.println();
  delay(target_period_ms);
  static float r[2];
  r[0] = requested_v;
  r[1] = requested_w;
  return r;
}