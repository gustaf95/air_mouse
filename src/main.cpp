#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <BleMouse.h>

MPU6050 mpu;
BleMouse bleMouse("ESP32_AirMouse");  // BLEMouse -> BleMouse

#define FILTER_TYPE 7  // 7: Complementary Filter + Savitzky-Golay Filter

#define DBG_COMP_FILTER 1

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

// Complementary Filter parameters
float compAngleX = 0, compAngleY = 0, compAngleZ = 0;
float alpha = 0.9996;

float prevCompAngleX = 0, prevCompAngleY = 0, prevCompAngleZ = 0;

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
  compAngleX = atan2(ay, az) * 180 / PI;
  compAngleY = atan2(ax, az) * 180 / PI;
  compAngleZ = 0;  // Assuming initial Z angle is 0

  Serial.println("Calibration completed.");
  Serial.println(gyroBiasX);
  Serial.println(gyroBiasY);
  Serial.println(gyroBiasZ);
}

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

  calibrateSensors();

  timer = micros();
}

char buffer[200] = {0,};

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print(">"); // for using serial-plotter

  // Convert gyroscope values to degrees/sec and remove bias
  float gyroXrate = ((gx / 131.0) - gyroBiasX);
  float gyroYrate = ((gy / 131.0) - gyroBiasY);
  float gyroZrate = ((gz / 131.0) - gyroBiasZ);
  sprintf(buffer, "gyroXrate:%f,gyroYrate:%f,", gyroXrate, gyroYrate);
  Serial.print(buffer);

  // Time difference
  float dt = (micros() - timer) / 1000000.0;
  timer = micros();

  // Step 1: Apply Complementary Filter
  float accelAngleX = atan2(ay, az) * 180 / PI;
  float accelAngleY = atan2(ax, az) * 180 / PI;
  float accelAngleZ = 0; // Assuming no direct calculation for Z angle from accelerometer

  compAngleX = alpha * (compAngleX + gyroXrate * dt) + (1.0 - alpha) * accelAngleX;
  compAngleY = alpha * (compAngleY + gyroYrate * dt) + (1.0 - alpha) * accelAngleY;
  compAngleZ = alpha * (compAngleZ + gyroZrate * dt) + (1.0 - alpha) * accelAngleZ;

#if DBG_COMP_FILTER
  // Calculate derivatives to compare with gyroscope values
  float compAngleX_rate = (compAngleX - prevCompAngleX) / dt;
  float compAngleY_rate = (compAngleY - prevCompAngleY) / dt;
  float compAngleZ_rate = (compAngleZ - prevCompAngleZ) / dt;

  // Update previous values for next iteration
  prevCompAngleX = compAngleX;
  prevCompAngleY = compAngleY;
  prevCompAngleZ = compAngleZ;

  // Serial output to compare
  sprintf(buffer, "compAngleX_rate:%f,compAngleY_rate:%f,", compAngleX_rate, compAngleY_rate);
  Serial.print(buffer);
#endif //DBG_COMP_FILTER

  // Step 2: Apply Savitzky-Golay Filter
  float filteredX = savitzkyGolayFilter(compAngleX_rate, sgBufferX);
  float filteredY = savitzkyGolayFilter(compAngleY_rate, sgBufferY);
  float filteredZ = savitzkyGolayFilter(compAngleZ_rate, sgBufferZ);

  // Serial output to compare
  sprintf(buffer, "filteredX:%f,filteredY:%f", filteredX, filteredY);
  Serial.print(buffer);
  Serial.println("");

  if (bleMouse.isConnected()) {
    bleMouse.move(filteredY / 4.0, -filteredX / 4.0, 0, 0);
  }

  delay(20); // Loop delay
}
