#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

/******************** IMU AXIS SELECTION ********************/
#define AXIS_MODE 0

/******************** PINS ********************/
const int PWM_PIN   = 14; // IN1: GREEN - GREEN
const int DIR_PIN   = 15; // IN2: ORANGE - BLACK
const int BRAKE_PIN = 16; // IN3: YELLOW - YELLOW

const int I2C_SDA_PIN = 8; // GREEN
const int I2C_SCL_PIN = 9; // YELLOW

// CHANGE THESE TO MATCH YOUR ACTUAL HALL INPUT WIRING
const uint8_t H1_PIN = 10; // RED-YELLOW
const uint8_t H2_PIN = 11; // GREEN
const uint8_t H3_PIN = 12; // BLUE 

/******************** POLARITY ********************/
const bool BRAKE_ACTIVE_LOW   = true;
const bool DIR_FORWARD_IS_LOW = true;

const int MOTOR_SIGN = -1;   // flip if motor direction is reversed
const int ANGLE_SIGN = +1;   // flip if measured tilt sign is reversed
const int HALL_SIGN  = +1;   // flip if omega sign is reversed
const int CTRL_SIGN  = +1;   // flip if controller correction is reversed

/******************** PWM ********************/
const uint32_t PWM_FREQ_HZ = 2000;
const uint8_t PWM_RES_BITS = 8;
const int PWM_MAX = (1 << PWM_RES_BITS) - 1;

const int PWM_MIN_RUN = 1;    // low but enough to start moving
const int PWM_STEP    = 7;    // output slew limit

static bool ledc_ok = false;

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#if defined(ESP_ARDUINO_VERSION) && defined(ESP_ARDUINO_VERSION_VAL) && \
    (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
#define ARDUINO_ESP32_V3 1
#else
#define ARDUINO_ESP32_V3 0
#endif

static inline void pwmWriteU8(uint8_t duty) {
  if (!ledc_ok) return;
#if ARDUINO_ESP32_V3
  ledcWrite(PWM_PIN, duty);
#else
  ledcWrite(0, duty);
#endif
}

/******************** TIMING ********************/
const float CONTROL_DT = 0.01f;
const uint32_t CONTROL_DT_US = 10000;
uint32_t lastControlUs = 0;

/******************** IMU ********************/
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;

const float ACC_LSB_PER_G = 16384.0f;
const float GYRO_LSB_PER_DPS = 131.0f;

float theta = 0.0f;        // rad
float theta_dot = 0.0f;    // rad/s
float theta_zero = 0.0f;   // rad
float gyro_zero_dps = 0.0f;
bool imu_ok = false;

float theta_f = 0.0f;
float theta_dot_f = 0.0f;
bool imu_stable = true;
uint32_t imuStableStartMs = 0;

uint8_t imuZeroStreak = 0;
uint8_t imuBadAccelStreak = 0;
uint32_t lastGoodImuMs = 0;

/******************** HALL / OMEGA ********************/
const uint8_t POLE_PAIRS = 23;
const uint16_t TRANS_PER_MECH_REV = 6u * POLE_PAIRS;

volatile int32_t hallCountSigned = 0;
volatile uint8_t lastHall = 0xFF;

int32_t lastHallCount = 0;
float omega_motor = 0.0f;      // rad/s
uint8_t dbg_hall = 0;
int32_t dbg_dCount = 0;

volatile uint8_t hallRawNow = 0;
volatile int8_t hallStepNow = 0;
volatile uint32_t hallEdgeCount = 0;

int32_t hallCountTotal = 0;
float hall_rps_meas = 0.0f;
float hall_rpm_meas = 0.0f;
int hall_dir_meas = 0;   // +1 forward, -1 reverse, 0 stopped

/******************** Direction-Braking ********************/
const bool BRAKE_DURING_REVERSAL   = true;
const float REVERSE_ZERO_OMEGA     = 0.4f;   // rad/s
const uint16_t REVERSE_TIMEOUT_MS  = 400;    // max wait before forcing reverse
const uint16_t REVERSE_BRAKE_HOLD_MS = 200;   // minimum brake pulse during reversal

/******************** PID / DAMPING ********************/
// Start conservative.
// Ki stays 0 initially. Add only after sign and basic response are correct.
float Kp = 150.0f;     // output units per rad
float Ki = 0.0f;      // start at 0
float Kd = 1.0f;      // output units per rad/s
float Komega = 0.6f;  // extra damping from motor speed

float e_int = 0.0f;
const float E_INT_MAX = 1.5f;

// This is the max "command magnitude" (Volts) before mapping to PWM.
// Assuming your motor supply is 48V. LQR computes outputs directly in Volts.
const float U_MAX = 40.0f;
const float CMD_DEADBAND = 1.0f;

/********************** LQR **********************/
// LQR gains -- state: [qp, qp_d, qr, qr_d] (all in radians / rad/s)
// Updated experimentally for R=1000.0 to reduce peak current spikes
//-1485.794815  -386.409343    -0.1         -2.189462
// -1719.220837, -545.343180, -0.100000, -2.20089

//#define K00 -1719.220837f //-1409.282983f  // Lean angle gain          (qp)
//#define K01 -545.343180f //-366.507281f   // Lean rate gain           (qp_d)
//#define K02 -0.1f // -0.031623f     // Wheel relative angle     (qr)
//#define K03 -2.200890f //-2.110835f     // Wheel absolute velocity  (qr_d)
/*
#define K00 -1637.883494f   // Lean angle gain          (qp)
#define K01 -519.539973f   // Lean rate gain            (qp_d)
#define K02 -0.044721f   // Wheel relative angle gain (qr)
#define K03 -2.127716f   // Wheel relative velocity gain (qr_d)
/*
#define K00 -1409.800264f  // Lean angle gain          (qp)
#define K01 -369.242998f   // Lean rate gain           (qp_d)
#define K02 -0.044721f     // Wheel relative angle     (qr)
#define K03 -2.122605f     // Wheel absolute velocity  (qr_d)


#define K00 -1671.174934f   // Lean angle gain          (qp)
#define K01 -530.101240f   // Lean rate gain            (qp_d)
#define K02 -0.070711f   // Wheel relative angle gain (qr)
#define K03 -2.158734f   // Wheel relative velocity gain (qr_d)


#define K00 -1719.220837f   // Lean angle gain          (qp)
#define K01 -545.343180f   // Lean rate gain            (qp_d)
#define K02 -0.100000f   // Wheel relative angle gain (qr)
#define K03 -2.200890f   // Wheel relative velocity gain (qr_d)
*/ 

// #define K00 -1804.855408f   // Lean angle gain          (qp)
// #define K01 -572.509654f   // Lean rate gain            (qp_d)
// #define K02 -0.141421f   // Wheel relative angle gain (qr)
// #define K03 -2.272605f   // Wheel relative velocity gain (qr_d)

 // #define K00 -2257.175337f   // Lean angle gain          (qp)
 // #define K01 -716.000482f   // Lean rate gain            (qp_d)
 // #define K02 -0.141421f   // Wheel relative angle gain (qr)
 // #define K03 -2.581971f   // Wheel relative velocity gain (qr_d)
//#define K00 -2713.065598f   // Lean angle gain          (qp)
//#define K01 -860.623986f   // Lean rate gain            (qp_d)
//#define K02 -0.141421f   // Wheel relative angle gain (qr)
//#define K03 -2.893779f   // Wheel relative velocity gain (qr_d)

//#define K00 -1684.933038f   // Lean angle gain          (qp)
//#define K01 -534.466328f   // Lean rate gain            (qp_d)
//#define K02 -0.141421f   // Wheel relative angle gain (qr)
//#define K03 -2.190584f   // Wheel relative velocity gain (qr_d)

//#define K00 -1719.220837f   // Lean angle gain          (qp)
//#define K01 -545.343180f   // Lean rate gain            (qp_d)
//#define K02 -0.100000f   // Wheel relative angle gain (qr)
//#define K03 -2.200890f   // Wheel relative velocity gain (qr_d)

//#define K00 -934.267130f   // Lean angle gain          (qp)
//#define K01 -296.869956f   // Lean rate gain            (qp_d)
//#define K02 -0.100000f   // Wheel relative angle gain (qr)
//#define K03 -2.202371f   // Wheel relative velocity gain (qr_d)
/*
#define K00 //-592.641979f   // Lean angle gain          (qp)
#define K01 //-188.732614f   // Lean rate gain            (qp_d)
#define K02 //-0.100000f   // Wheel relative angle gain (qr)
#define K03 //-2.204416f   // Wheel relative velocity gain (qr_d)

#define K00 -1719.220837f   // Lean angle gain          (qp)
#define K01 -545.343180f   // Lean rate gain            (qp_d)
#define K02 -0.100000f   // Wheel relative angle gain (qr)
#define K03 -2.200890f   // Wheel relative velocity gain (qr_d)

#define K00 -1637.883494f   // Lean angle gain          (qp)
#define K01 -519.539973f   // Lean rate gain            (qp_d)
#define K02 -0.044721f   // Wheel relative angle gain (qr)
#define K03 -2.127716f   // Wheel relative velocity gain (qr_d)
*/

#define K00 -1628.118868f   // Lean angle gain          (qp)
#define K01 -516.442280f   // Lean rate gain            (qp_d)
#define K02 -0.035355f   // Wheel relative angle gain (qr)
#define K03 -2.118065f   // Wheel relative velocity gain (qr_d)

float LQR_K_THETA     = K00; 
float LQR_K_THETA_DOT = K01;
float LQR_K_WHEEL     = K02;
float LQR_K_OMEGA     = K03;

/******************** THRESHOLDS ********************/
const float TILT_DEADBAND_DEG = 0.0f;
const float TILT_CTRL_CLAMP_DEG = 30.0f;
const float ANGLE_CUTOFF_DEG = 45.0f;
const float QUIET_RATE_DPS = 5.0f;

/******************** DEBUG ********************/
float dbg_theta_acc_deg = 0.0f;
float dbg_rate_dps = 0.0f;
float dbg_u = 0.0f;
int dbg_pwm = 0;
bool dbg_brake = false;
int dbg_dir = +1;
bool momentum_dump_active = false;

/******************** OUTPUT HELPERS ********************/
void setBrake(bool on) {
  dbg_brake = on;
  digitalWrite(BRAKE_PIN, BRAKE_ACTIVE_LOW ? (on ? LOW : HIGH)
                                           : (on ? HIGH : LOW));
}

void setForward(bool forward) {
  bool physicalForward = (MOTOR_SIGN > 0) ? forward : !forward;
  dbg_dir = physicalForward ? +1 : -1;
  digitalWrite(DIR_PIN, DIR_FORWARD_IS_LOW ? (physicalForward ? LOW : HIGH)
                                           : (physicalForward ? HIGH : LOW));
}

void setPWM(int pwm) {
  pwm = constrain(pwm, 0, PWM_MAX);
  dbg_pwm = pwm;
  pwmWriteU8((uint8_t)pwm);
}

void safeStop() {
  setPWM(0);
  setBrake(true);
}

void coastStop() {
  setPWM(0);
  setBrake(false);
}

/******************** IMU AXIS HELPERS ********************/
float selectedAccelAngleRad(float ax_g, float ay_g, float az_g) {
#if AXIS_MODE == 0
  return atan2f( ay_g, az_g);
#elif AXIS_MODE == 1
  return atan2f(-ay_g, az_g);
#elif AXIS_MODE == 2
  return atan2f( ax_g, az_g);
#elif AXIS_MODE == 3
  return atan2f(-ax_g, az_g);
#elif AXIS_MODE == 4
  return atan2f( ax_g, ay_g);
#elif AXIS_MODE == 5
  return atan2f(-ax_g, ay_g);
#else
  return atan2f( ay_g, az_g);
#endif
}

float selectedGyroDps(float gx_dps, float gy_dps, float gz_dps) {
#if AXIS_MODE == 0
  return gx_dps;
#elif AXIS_MODE == 1
  return gx_dps;
#elif AXIS_MODE == 2
  return gy_dps;
#elif AXIS_MODE == 3
  return gy_dps;
#elif AXIS_MODE == 4
  return gz_dps;
#elif AXIS_MODE == 5
  return gz_dps;
#else
  return gx_dps;
#endif
}

void resetIMUFusion() {
  theta = 0.0f;
  theta_dot = 0.0f;
  theta_f = 0.0f;
  theta_dot_f = 0.0f;
  imu_stable = false;
  imuStableStartMs = millis();
}

/******************** HALL HELPERS ********************/
bool isValidHall(uint8_t h) {
  return (h != 0b000) && (h != 0b111);
}

uint8_t readHall() {
  return (digitalRead(H3_PIN) << 2) |
         (digitalRead(H2_PIN) << 1) |
         (digitalRead(H1_PIN) << 0);
}

// Forward sequence:
// 001 -> 101 -> 100 -> 110 -> 010 -> 011 -> 001
int8_t hallStep(uint8_t prev, uint8_t curr) {
  uint8_t key = (prev << 3) | curr;

  switch (key) {
    case 0b001101:
    case 0b101100:
    case 0b100110:
    case 0b110010:
    case 0b010011:
    case 0b011001:
      return +1;

    case 0b101001:
    case 0b100101:
    case 0b110100:
    case 0b010110:
    case 0b011010:
    case 0b001011:
      return -1;

    default:
      return 0;
  }
}


void IRAM_ATTR hallISR() {
  uint8_t h = readHall();
  hallRawNow = h;

  if (!isValidHall(h)) return;

  if (lastHall != 0xFF) {
    int8_t step = hallStep(lastHall, h);
    hallStepNow = step;

    if (step != 0) {
      hallCountSigned += step;
      hallEdgeCount++;
    }
  }

  lastHall = h;
}


void updateMotorSpeed() {
  static uint32_t lastUs = 0;
  const uint32_t MIN_UPDATE_US = 5000;

  uint32_t nowUs = micros();
  uint32_t dUs = nowUs - lastUs;
  if (dUs < MIN_UPDATE_US) return;
  lastUs = nowUs;

  float dt = dUs * 1e-6f;
  if (dt <= 0.0f) return;

  int32_t countNow;
  uint8_t hallNow;
  int8_t stepNow;
  uint32_t edgeNow;

  noInterrupts();
  countNow = hallCountSigned;
  hallNow = hallRawNow;
  stepNow = hallStepNow;
  edgeNow = hallEdgeCount;
  interrupts();

  int32_t dCount = countNow - lastHallCount;
  lastHallCount = countNow;

  dbg_dCount = dCount;
  dbg_hall = hallNow;

  hallCountTotal = countNow;

  float mechRevs = (float)(HALL_SIGN * dCount) / (float)TRANS_PER_MECH_REV;
  hall_rps_meas = mechRevs / dt;
  hall_rpm_meas = hall_rps_meas * 60.0f;

  // This is the measured flywheel angular velocity from hall sensors
  omega_motor = hall_rps_meas * 2.0f * (float)M_PI;

  if (dCount > 0) hall_dir_meas = +1;
  else if (dCount < 0) hall_dir_meas = -1;
  else hall_dir_meas = 0;
}

/******************** IMU CALIBRATION ********************/
void calibrateIMU() {
  const int N = 400;
  float angle_sum = 0.0f;
  float rate_sum = 0.0f;

  for (int i = 0; i < N; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float ax_g = (float)ax / ACC_LSB_PER_G;
    float ay_g = (float)ay / ACC_LSB_PER_G;
    float az_g = (float)az / ACC_LSB_PER_G;

    float gx_dps = (float)gx / GYRO_LSB_PER_DPS;
    float gy_dps = (float)gy / GYRO_LSB_PER_DPS;
    float gz_dps = (float)gz / GYRO_LSB_PER_DPS;

    angle_sum += selectedAccelAngleRad(ax_g, ay_g, az_g);
    rate_sum += selectedGyroDps(gx_dps, gy_dps, gz_dps);

    delay(3);
  }

  theta_zero = angle_sum / (float)N;
  gyro_zero_dps = rate_sum / (float)N;
  theta = 0.0f;
  theta_dot = 0.0f;
}

/******************** IMU UPDATE ********************/
bool updateIMU() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  bool allZero = (ax == 0 && ay == 0 && az == 0 && gx == 0 && gy == 0 && gz == 0);
  if (allZero) {
    imuZeroStreak++;
    // tolerate a couple of bad frames before declaring failure
    if (imuZeroStreak < 3) {
      return true;   // hold previous theta/theta_dot
    }
    return false;
  }
  imuZeroStreak = 0;

  float ax_g = (float)ax / ACC_LSB_PER_G;
  float ay_g = (float)ay / ACC_LSB_PER_G;
  float az_g = (float)az / ACC_LSB_PER_G;

  float gx_dps = (float)gx / GYRO_LSB_PER_DPS;
  float gy_dps = (float)gy / GYRO_LSB_PER_DPS;
  float gz_dps = (float)gz / GYRO_LSB_PER_DPS;

  float theta_acc = selectedAccelAngleRad(ax_g, ay_g, az_g) - theta_zero;
  float rate_dps  = selectedGyroDps(gx_dps, gy_dps, gz_dps) - gyro_zero_dps;

  theta_acc *= ANGLE_SIGN;
  rate_dps  *= ANGLE_SIGN;

  dbg_theta_acc_deg = theta_acc * RAD_TO_DEG;
  dbg_rate_dps = rate_dps;

  // reject absurd gyro spikes, but don't hard-fault immediately
  if (fabs(rate_dps) > 300.0f) {
    rate_dps = 0.0f;
  }

  theta_dot = rate_dps * DEG_TO_RAD;

  // light LPF on gyro
  theta_dot_f += 0.2f * (theta_dot - theta_dot_f);
  theta_dot = theta_dot_f;

  float theta_pred = theta_f + theta_dot * CONTROL_DT;

  float a_norm = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

  // If accel magnitude is suspicious, ignore accel correction and trust gyro briefly
  if (a_norm > 0.80f && a_norm < 1.20f) {
    imuBadAccelStreak = 0;
    theta_f = 0.995f * theta_pred + 0.005f * theta_acc;
  } else {
    imuBadAccelStreak++;
    theta_f = theta_pred;

    // only fail after sustained bad accel
    if (imuBadAccelStreak > 25) {   // about 250 ms at 100 Hz
      return false;
    }
  }

  theta = theta_f;
  lastGoodImuMs = millis();
  return true;
}

void applyDutyCommand(float u_cmd) {
  dbg_u = u_cmd;

  static int pwmApplied = 0;
  static int lastDesiredDir = +1;
  static bool reversing = false;
  static uint32_t reverseStartMs = 0;

  if (fabs(u_cmd) < CMD_DEADBAND) {
    e_int = 0.0f;

    // Normal stop near zero command: ramp down gently, brake off
    if (pwmApplied > 0) {
      pwmApplied -= PWM_STEP;
      if (pwmApplied < 0) pwmApplied = 0;
      setPWM(pwmApplied);
      setBrake(false);
    } else {
      coastStop();
    }

    reversing = false;
    return;
  }

  int desiredDir = (u_cmd >= 0.0f) ? +1 : -1;


  if (desiredDir != lastDesiredDir) {
    if (!reversing) {
      reversing = true;
      reverseStartMs = millis();
    }

    // Drop PWM immediately
    pwmApplied = 0;
    setPWM(0);

    // Hold brake during reversal wait
    setBrake(BRAKE_DURING_REVERSAL);

    uint32_t t = millis() - reverseStartMs;
    bool brakePulseDone = (t >= REVERSE_BRAKE_HOLD_MS);
    bool wheelSlow = (fabs(omega_motor) <= REVERSE_ZERO_OMEGA);
    bool timedOut = (t >= REVERSE_TIMEOUT_MS);

    if (!brakePulseDone || (!wheelSlow && !timedOut)) {
      return;
    }

    setForward(desiredDir > 0);
    lastDesiredDir = desiredDir;
    reversing = false;
    //setBrake(false);
  }

  // -------- Normal command application --------
  float mag = fabs(u_cmd);
  mag = constrain(mag, 0.0f, U_MAX);

  int pwmTarget = (int)lroundf((mag / U_MAX) * PWM_MAX);
  if (pwmTarget > 0 && pwmTarget < PWM_MIN_RUN) pwmTarget = PWM_MIN_RUN;

  if (pwmApplied < pwmTarget) pwmApplied += PWM_STEP;
  if (pwmApplied > pwmTarget) pwmApplied -= PWM_STEP;

  pwmApplied = constrain(pwmApplied, 0, PWM_MAX);
  setBrake(false);
  setPWM(pwmApplied);
}

/********************** LQR Algorithm ***********************/
float computeLQRCommand(float theta_for_ctrl,
                        float theta_dot_for_ctrl,
                        float wheel_angle_for_ctrl,
                        float omega_for_ctrl) {
  float u = 0.0f;
  
  // u = -K * x -> Note the sign on theta_for_ctrl since our defined setpoint is 0.
  // Using user's signed convention where positive u = motor forward
  // And positive theta_for_ctrl = falling forward
  
  // To counteract falling forward (theta > 0), motor must accelerate wheel forward
  // causing a backwards reaction torque. Let's calculate Vin:
  float Vin = -(K00 * theta_for_ctrl + K01 * theta_dot_for_ctrl + K02 * wheel_angle_for_ctrl + K03 * omega_for_ctrl);

  // DEBUGGING: Your true mathematical gain (K00 = 1409) still commands 
  // ~24 Volts of power for a single 1 degree tilt! Ensure you slowly 
  // increase this scale factor from 0.1 to 1.0 to find the limit of your battery/EMI.
  const float LQR_SCALE_FACTOR = 1.0f; 
  u = Vin * LQR_SCALE_FACTOR;

  u = constrain(u, -U_MAX, +U_MAX);
  return CTRL_SIGN * u;
}

/******************** I2C RECOVERY ********************/
void recoverI2C() {
    Wire.end(); // Release hardware I2C control
    delay(10);

    // Clock out up to 9 pulses to un-stick a held SDA line
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C_SCL_PIN, OUTPUT);
    for (int i = 0; i < 9; i++) {
        digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(5);
        digitalWrite(I2C_SCL_PIN, LOW);  delayMicroseconds(5);
        if (digitalRead(I2C_SDA_PIN)) break;  // SDA released, done
    }
    
    // Re-initialize the bus with our specific pins and safe 100kHz frequency
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 50000);
}

/******************** SETUP ********************/
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);

  pinMode(H1_PIN, INPUT);
  pinMode(H2_PIN, INPUT);
  pinMode(H3_PIN, INPUT);

  lastHall = readHall();

  attachInterrupt(digitalPinToInterrupt(H1_PIN), hallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H2_PIN), hallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(H3_PIN), hallISR, CHANGE);

#if ARDUINO_ESP32_V3
  ledc_ok = ledcAttach(PWM_PIN, PWM_FREQ_HZ, PWM_RES_BITS);
#else
  ledcSetup(0, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PWM_PIN, 0);
  ledc_ok = true;
#endif

  safeStop();
  setForward(true);

  // Dropped I2C clock to standard 100kHz (from 200kHz) to make it more resistant to high-frequency EMI noise
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  
  // DEBUGGING: Add I2C Timeout to prevent infinite hangs caused by motor EMI.
  // Standard ESP32 Wire library hangs forever if noise corrupts the I2C bus.
  #if defined(ESP_ARDUINO_VERSION_VAL) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
    Wire.setTimeout(150); 
  #else
    Wire.setTimeOut(150);
  #endif

  mpu.initialize();
  imu_ok = mpu.testConnection();

  if (imu_ok) {
    delay(1000);
    calibrateIMU();
    resetIMUFusion();
    Serial.println("MPU6050 connected and calibrated");
  } else {
    Serial.println("ERROR: MPU6050 not detected");
  }

  Serial.println("ESP32-S3 hall-verified PID duty-mode test started");
  lastControlUs = micros();
}

/******************** LOOP ********************/
void loop() {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastControlUs) < CONTROL_DT_US) return;
  lastControlUs = nowUs;

  if (!imu_stable) {
    if (millis() - imuStableStartMs > 2000) {
      imu_stable = true;
    } else {
      coastStop();
      return;
    }
  }

  if (!imu_ok) {
    safeStop();
    return;
  }

  if (!updateIMU()) {
    // coast down first instead of hard-stop on a single fault event
    momentum_dump_active = false;
    applyDutyCommand(0.0f);

    static uint32_t lastErrMs = 0;
    if (millis() - lastErrMs > 3000) {
      Serial.println("** ALERT: MPU6050 DATA INVALID **");
      Serial.println("Stopping drive, recovering I2C, and resetting IMU fusion...");

      recoverI2C();
      mpu.initialize();
      imu_ok = mpu.testConnection();

      if (imu_ok) {
        delay(200);
        calibrateIMU();

        theta = 0.0f;
        theta_dot = 0.0f;
        theta_f = 0.0f;
        theta_dot_f = 0.0f;
        imuZeroStreak = 0;
        imuBadAccelStreak = 0;
        lastGoodImuMs = millis();
      }

      lastErrMs = millis();
    }
    return;
  }

  updateMotorSpeed();

  float theta_deg = theta * RAD_TO_DEG;

  if (fabsf(theta_deg) > ANGLE_CUTOFF_DEG) {
    e_int = 0.0f;
    safeStop();
    return;
  }

  // Quiet zone near upright
  if (fabsf(theta_deg) < TILT_DEADBAND_DEG &&
      fabsf(theta_dot * RAD_TO_DEG) < QUIET_RATE_DPS) {
    e_int = 0.0f;
    momentum_dump_active = false;
    applyDutyCommand(0.0f);
  } else {
    float theta_eff_deg = constrain(theta_deg, -TILT_CTRL_CLAMP_DEG, +TILT_CTRL_CLAMP_DEG);
    float theta_eff = theta_eff_deg * DEG_TO_RAD;
    /*
    float e = -theta_eff;
    e_int += e * CONTROL_DT;
    e_int = constrain(e_int, -E_INT_MAX, E_INT_MAX);

    float u = CTRL_SIGN * (
        Kp * e
      - Kd * theta_dot
      - Komega * omega_motor
      + Ki * e_int
    );*/ 

    // We bypass the absolute wheel angle (qr) state for now by passing 0.0f. 
    // Modifying qr with a washout filter breaks the mathematical relationship 
    // expected by LQR (d/dt qr = omega), which can instantly cause violent instability 
    // and crash the MCU. K02 is tiny anyway (-0.316) so ignoring it is completely safe 
    // and will actually make the balancing much smoother.
    float u = computeLQRCommand(theta_eff, theta_dot, 0.0f, omega_motor);
    
    // Check if the wheel is spinning against the tilt without the 0.5 deg deadband delay
    momentum_dump_active = ((theta * omega_motor < 0) && (fabsf(omega_motor) > REVERSE_ZERO_OMEGA));
    
    if (momentum_dump_active) {
      setPWM(0);
      setBrake(true);
    } else {
      applyDutyCommand(u);
    }
  }

  static uint32_t dbg = 0;
  if (++dbg >= 10) {
    dbg = 0;

   // Serial.print("theta_deg=");
    Serial.print(theta_deg, 2);
    Serial.print(",");

    //Serial.print(" | theta_dot_deg_s=");
    Serial.print(theta_dot * RAD_TO_DEG, 1);
    Serial.print(",");

    //Serial.print(" | hall_raw=");
    Serial.print(dbg_hall, BIN);
    Serial.print(",");

    //Serial.print(" | dCount=");
    Serial.print(dbg_dCount);
    Serial.print(",");

    //Serial.print(" | hallCountTotal=");
    Serial.print(hallCountTotal);
    Serial.print(",");

    //Serial.print(" | omega_meas_rad_s="); //measured motor angular velocity
    Serial.print(omega_motor, 2);
    Serial.print(",");

    //Serial.print(" | hall_rps="); //revolutions per sec
    Serial.print(hall_rps_meas, 3);
    Serial.print(",");

    //Serial.print(" | hall_rpm="); //revolutions per min
    Serial.print(hall_rpm_meas, 1);
    Serial.print(",");

    //Serial.print(" | hall_dir=");
    if (hall_dir_meas > 0) Serial.print("F");
    else if (hall_dir_meas < 0) Serial.print("R");
    else Serial.print("0");
    Serial.print(",");

    //Serial.print(" | u_cmd=");
    Serial.print(dbg_u, 2);
    Serial.print(",");

    //Serial.print(" | pwm=");
    Serial.print(dbg_pwm);
    Serial.print(",");

    //Serial.print(" | brake=");
    Serial.print(dbg_brake ? "ON" : "OFF");
    Serial.print(",");

    //Serial.print(" | dir_cmd=");
    Serial.print(dbg_dir > 0 ? "F" : "R");
    Serial.print(",");

    Serial.println(momentum_dump_active ? "MD:ON" : "MD:OFF");
  }
}











