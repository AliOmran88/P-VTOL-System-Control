#include <Servo.h>
#include <Wire.h>
#include <avr/wdt.h>

// MPU6050
const int MPU_addr = 0x68;

int16_t AcX, AcY, AcZ, GyY;

// Angle and Angular Velocity
float theta = 0.0;        // filtered angle [deg]
float theta_acc = 0.0;    // accelerometer angle [deg]

float omega_deg = 0.0;    // gyro velocity [deg/s]
float omega = 0.0;        // gyro velocity [rad/s]

// Calibration offsets
float baseY = 0.0;
float gyroY_offset = 0.0;

// Complementary filter coefficient
float alpha = 0.99;

// Motors
const int motor1Pin = 9;
const int motor2Pin = 10;

Servo motor1;
Servo motor2;

// Timing
unsigned long lastTime = 0;
const unsigned long Ts_ms = 20;

// State Feedback Parameters
float theta_ref = 0;
float K_theta = 0.4;
float K_omega = 0.8;

// ESC Parameters

int u0 = 1100;

int u_min = 1000;
int u_max = 1300;

float pwm_slew = 6.0;
float u1_prev = 1000;
float u2_prev = 1000;

// Read MPU6050 using I2C

void readMPU()
{
  Wire.beginTransmission(MPU_addr);
  // Start reading from ACCEL_XOUT_H register
  Wire.write(0x3B);
  Wire.endTransmission(false);
  // Request 14 bytes
  Wire.requestFrom(MPU_addr, 14, true);
  // Accelerometer
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  // Skip temperature
  Wire.read();
  Wire.read();
  // Skip GyX
  Wire.read();
  Wire.read();
  // Read GyY
  GyY = Wire.read() << 8 | Wire.read();
  // GyZ discarded automatically
}

void readSerialData()
{
  char buffer[32];

  if (Serial.available() > 0)
  {
    int len = Serial.readBytesUntil('\n',
                                    buffer,
                                    sizeof(buffer) - 1);

    buffer[len] = '\0';

    float ref, kp, kd;

    int parsed = sscanf(buffer,
                        "%f,%f,%f",
                        &ref,
                        &kp,
                        &kd);

    // Make sure ALL values received correctly
    if (parsed == 3)
    {
      // Safety limits
      if (ref >= -40.0 && ref <= 40.0)
      {
        theta_ref = ref;
      }

      if (kp >= 0.0 && kp <= 20.0)
      {
        K_theta = kp;
      }

      if (kd >= 0.0 && kd <= 20.0)
      {
        K_omega = kd;
      }
    }
  }
}

void readSerialDat()
{
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');

        input.trim();

        // Find commas
        int firstComma  = input.indexOf(',');
        int secondComma = input.indexOf(',', firstComma + 1);

        // Validate format
        if (firstComma > 0 && secondComma > firstComma)
        {
            String refStr = input.substring(0, firstComma);

            String kpStr  = input.substring(firstComma + 1,
                                            secondComma);

            String kdStr  = input.substring(secondComma + 1);

            // Convert to float
            float ref = refStr.toFloat();
            float kp  = kpStr.toFloat();
            float kd  = kdStr.toFloat();

            // Safety limits
            if (ref >= -40 && ref <= 40)
                theta_ref = ref;

            if (kp >= 0 && kp <= 20)
                K_theta = kp;

            if (kd >= 0 && kd <= 20)
                K_omega = kd;
        }
    }
}

void setup()
{
  Serial.begin(115200);
  Wire.setWireTimeout(3000, true);
  Wire.begin();
  // Wake up MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(1000);

  // ESC Setup
  motor1.attach(motor1Pin, 1000, 2000);
  motor2.attach(motor2Pin, 1000, 2000);

  // Safe startup
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);

  u1_prev = 1000;
  u2_prev = 1000;

  delay(5000);
  
  readMPU();

  // Initial angle reference
  baseY = atan2(AcX, AcZ) * RAD_TO_DEG;
  // Initial gyro bias
  gyroY_offset = GyY;
  // Initialize filtered angle at zero
  theta = 0.0;

  lastTime = millis();
  wdt_enable(WDTO_250MS);  // reset if loop stalls
}

void loop()
{
  wdt_reset();
  unsigned long now = millis();

  if (now - lastTime >= Ts_ms)
  {
    float dt = (now - lastTime) / 1000.0;

    // 1. Read MPU
    readMPU();
    readSerialDat();

    // 2. Accelerometer Angle
    theta_acc = atan2(AcX, AcZ) * RAD_TO_DEG - baseY;

    // 3. Gyroscope Angular Velocity
    omega_deg = -((GyY - gyroY_offset) / 131.0);

    // 4. Gyro Deadband
    if (abs(omega_deg) < 0.5)
    {
      omega_deg = 0.0;
    }

    // 5. Complementary Filter
    theta = alpha * (theta + omega_deg * dt)
          + (1.0 - alpha) * theta_acc;

    // 6. Stationary Drift Correction
    if (abs(theta_acc) < 0.5 && abs(omega_deg) < 0.03)
    {
      theta = theta_acc;
    }

    // 7. Convert omega to rad/s
    //omega = omega_deg * DEG_TO_RAD;
    static float omega_f = 0;

    omega_f = 0.9*omega_f + 0.1*omega_deg;

    omega = omega_f * DEG_TO_RAD;

    //readSerialData();

    // 8. State Feedback Control
     float du = -(K_theta * (theta - theta_ref) + K_omega * omega);

    // 9. Differential Motor Commands
    int u1 = u0 + du;
    int u2 = u0 - du;
    //kjkhigh

    // ESC saturation
    u1 = constrain(u1, u_min, u_max);
    u2 = constrain(u2, u_min, u_max);

    // 10. Slew Rate Limiting
    u1 = u1_prev + constrain(u1 - u1_prev, -pwm_slew, pwm_slew);

    u2 = u2_prev + constrain(u2 - u2_prev, -pwm_slew, pwm_slew);

    // Save previous PWM values
    u1_prev = u1;
    u2_prev = u2;

    // 11. Apply to ESCs
    motor1.writeMicroseconds((int)u1);
    motor2.writeMicroseconds((int)u2);

    // 12. Serial Logging
    static int print_cnt = 0;
    print_cnt++;
    if (print_cnt >= 10) {   // 10 Hz printing
      print_cnt = 0;
    Serial.print(theta, 2);
    Serial.print(",");

    Serial.print(omega, 2);
    Serial.print(",");

    Serial.print(u1);
    Serial.print(",");

    Serial.println(u2);
    }

    // Update Time
    lastTime = now;
  }
}