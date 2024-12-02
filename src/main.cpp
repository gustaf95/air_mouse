#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <MadgwickAHRS.h>
#include <BleMouse.h>

MPU6050 mpu;
Madgwick filter;
BleMouse bleMouse("ESP32_AirMouse");  // BLEMouse -> BleMouse

#define FILTER_TYPE 7  // 7: Madgwick + Adaptive Kalman + Savitzky-Golay Filter

// Kalman Filter parameters
float adaptiveKalmanAngleX = 0, adaptiveKalmanAngleY = 0, adaptiveKalmanAngleZ = 0;
float P[3][2][2] = {{{0.01, 0}, {0, 0.01}}, {{0.01, 0}, {0, 0.01}}, {{0.01, 0}, {0, 0.01}}}; // Adjusted after calibration
float R = 0.01;  // Measurement noise covariance
float Q_angle = 0.001;  // Process noise covariance for the angle
float Q_gyro = 0.003;   // Process noise covariance for the gyro

// Savitzky-Golay filter parameters
#define SG_ORDER 9  // Number of coefficients
float sgCoeffs[] = {-0.09090909, 0.06060606, 0.16883117, 0.23376623, 0.25541126, 0.23376623, 0.16883117, 0.06060606, -0.09090909};
float sgBufferX[SG_ORDER] = {0};
float sgBufferY[SG_ORDER] = {0};
float sgBufferZ[SG_ORDER] = {0};

// Timer variable for time difference calculation
unsigned long timer = 0;

// Calibration variables
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

void calibrateSensors() {
  const int calibrationSamples = 1000;
  long gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;

  Serial.println("Calibrating sensors...");
  for (int i = 0; i < calibrationSamples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    gyroXSum += gx;
    gyroYSum += gy;
    gyroZSum += gz;

    delay(2); // Small delay between samples
  }

  gyroBiasX = gyroXSum / calibrationSamples / 131.0;
  gyroBiasY = gyroYSum / calibrationSamples / 131.0;
  gyroBiasZ = gyroZSum / calibrationSamples / 131.0;

  // Calculate initial angles from accelerometer
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  adaptiveKalmanAngleX = atan2(ay, az) * 180 / PI;
  adaptiveKalmanAngleY = atan2(ax, az) * 180 / PI;
  adaptiveKalmanAngleZ = 0;  // Assuming initial Z angle is 0

  Serial.println("Calibration completed.");
  Serial.println(gyroBiasX);
  Serial.println(gyroBiasY);
  Serial.println(gyroBiasZ);
}

// Adaptive Kalman Filter function
float adaptiveKalmanFilter(float accelAngle, float gyroRate, float dt, float &adaptiveKalmanAngle, float P[2][2]) {
  // Predict error covariance matrix P
  float Pdot[2] = {Q_angle - P[0][1] - P[1][0], -P[1][1]};
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[1] * dt;
  P[1][1] += Q_gyro * dt;

  // Calculate Kalman gain
  float S = P[0][0] + R;
  float K[2] = {P[0][0] / S, P[1][0] / S};

  // Update angle with measurement
  float y = accelAngle - adaptiveKalmanAngle;
  adaptiveKalmanAngle += K[0] * y;

  // Update error covariance matrix P
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return adaptiveKalmanAngle;
}

// Savitzky-Golay Filter function
float savitzkyGolayFilter(float currentValue, float buffer[]) {
  // Shift buffer values to the right
  for (int i = SG_ORDER - 1; i > 0; i--) {
    buffer[i] = buffer[i - 1];
  }
  buffer[0] = currentValue;

  // Apply Savitzky-Golay coefficients to calculate smoothed value
  float result = 0;
  for (int i = 0; i < SG_ORDER; i++) {
    result += buffer[i] * sgCoeffs[i];
  }
  return result;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  bleMouse.begin();
  filter.begin(50);  // Initialize Madgwick filter with sampling rate of 50 Hz
  
  calibrateSensors();

  timer = micros();
}

char buffer[200] = {0,};
float prevMadgwickX = 0, prevMadgwickY = 0, prevMadgwickZ = 0;

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print(">"); //for using serial-plotter
  
  // Convert gyroscope values to degrees/sec and remove bias
  float gyroXrate = ((gx / 131.0) - gyroBiasX) * (PI / 180);
  float gyroYrate = ((gy / 131.0) - gyroBiasY) * (PI / 180);
  float gyroZrate = ((gz / 131.0) - gyroBiasZ) * (PI / 180);
  sprintf(buffer, "gyroXrate:%f,gyroYrate:%f,gyroZrate:%f,", gyroXrate, gyroYrate, gyroZrate);
  Serial.print(buffer);

  // Time difference
  float dt = (micros() - timer) / 1000000.0;
  timer = micros();

  // Step 1: Apply Madgwick Filter
  // Convert accelerometer values to g-force units
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;
  
  // Apply Madgwick Filter with correct units for accelerometer and gyroscope
  filter.updateIMU(gyroXrate, gyroYrate, gyroZrate, ax_g, ay_g, az_g);
  float madgwickX = filter.getPitch();
  float madgwickY = filter.getRoll();
  float madgwickZ = filter.getYaw();

  // Calculate derivatives to compare with gyroscope values
  float madgwickX_rate = (madgwickX - prevMadgwickX) / dt;
  float madgwickY_rate = (madgwickY - prevMadgwickY) / dt;
  float madgwickZ_rate = (madgwickZ - prevMadgwickZ) / dt;

  // Update previous values for next iteration
  prevMadgwickX = madgwickX;
  prevMadgwickY = madgwickY;
  prevMadgwickZ = madgwickZ;

  // Serial output to compare
  sprintf(buffer, "madgwickX_rate:%f,madgwickY_rate:%f,madgwickZ_rate:%f", madgwickX_rate, madgwickY_rate, madgwickZ_rate);
  Serial.print(buffer);

  // Step 2: Apply Adaptive Kalman Filter
  float filteredX = adaptiveKalmanFilter(madgwickX, gyroXrate, dt, adaptiveKalmanAngleX, P[0]);
  float filteredY = adaptiveKalmanFilter(madgwickY, gyroYrate, dt, adaptiveKalmanAngleY, P[1]);
  float filteredZ = adaptiveKalmanFilter(madgwickZ, gyroZrate, dt, adaptiveKalmanAngleZ, P[2]);

  // Step 3: Apply Savitzky-Golay Filter
  filteredX = savitzkyGolayFilter(filteredX, sgBufferX);
  filteredY = savitzkyGolayFilter(filteredY, sgBufferY);
  filteredZ = savitzkyGolayFilter(filteredZ, sgBufferZ);

  if (bleMouse.isConnected()) {
    bleMouse.move(filteredZ / 128.0, -filteredX / 128.0, 0, 0);
  }

  Serial.println("");
  delay(20); // Loop delay
}

