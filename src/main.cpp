#include <Adafruit_LSM6DS33.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

// Function Prototypes =====================================================================
float getAltitude();
bool getAcceleration();
void updateAccelHistory(float history[], float newVal);
void computeJerk(const float ax[], const float ay[], const float az[], float dt,
                 float &jerkX, float &jerkY, float &jerkZ);

// Global Objects ===========================================================================
Adafruit_BMP280 bmp;
Adafruit_LSM6DS33 lsm6ds33;
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, 18, NEO_GRB + NEO_KHZ800);  // RGB NeoPixel on pin D18

// Global Variables =========================================================================
float accelX[4] = {0}, accelY[4] = {0}, accelZ[4] = {0};
float currAx = 0, currAy = 0, currAz = 0;
int accelChecker = 0;

float prevJerkX = 0.0;
float prevJerkY = 0.0;
float prevJerkZ = 0.0;

unsigned long lastTime = 0;
unsigned long internalClock = 0;

// =========================================================================================
// SETUP ====================================================================================
// =========================================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(250);

  bmp.begin(0x77);
  lsm6ds33.begin_I2C();
  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds33.setAccelDataRate(LSM6DS_RATE_104_HZ);

  pixel.begin();
  pixel.show();  // Initialize NeoPixel to off

  lastTime = millis();
  internalClock = 0;

  Serial.println("Time [s], Internal Clock [ms], Altitude [m], Accel X [m/s^2], Accel Y [m/s^2], Accel Z [m/s^2], Jerk X [m/s^3], Jerk Y [m/s^3], Jerk Z [m/s^3]");
}

// =========================================================================================
// LOOP =====================================================================================
// =========================================================================================
void loop() {
  float altitude = getAltitude();

  if (!getAcceleration()) {
    delay(1000);
    return;
  }

  updateAccelHistory(accelX, currAx);
  updateAccelHistory(accelY, currAy);
  updateAccelHistory(accelZ, currAz);

  if (accelChecker < 4) {
    accelChecker++;
    delay(1000);
    return;
  }

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  internalClock += 1000;

  float jerkX, jerkY, jerkZ;
  computeJerk(accelX, accelY, accelZ, dt, jerkX, jerkY, jerkZ);

  float percentChangeX = fabs(jerkX - prevJerkX) / max(fabs(prevJerkX), 0.01);
  float percentChangeY = fabs(jerkY - prevJerkY) / max(fabs(prevJerkY), 0.01);
  float percentChangeZ = fabs(jerkZ - prevJerkZ) / max(fabs(prevJerkZ), 0.01);

  float threshold = 5.0;

  if (fabs(jerkX) > threshold || fabs(jerkY) > threshold || fabs(jerkZ) > threshold) {
    if (fabs(jerkX) >= fabs(jerkY) && fabs(jerkX) >= fabs(jerkZ)) {
        pixel.setPixelColor(0, jerkX > 0 ? pixel.Color(255, 0, 0) : pixel.Color(0, 0, 255));  // X: Red / Blue
    } else if (fabs(jerkY) >= fabs(jerkX) && fabs(jerkY) >= fabs(jerkZ)) {
        pixel.setPixelColor(0, jerkY > 0 ? pixel.Color(0, 255, 0) : pixel.Color(255, 255, 0));  // Y: Green / Yellow
    } else {
        pixel.setPixelColor(0, jerkZ > 0 ? pixel.Color(128, 0, 128) : pixel.Color(255, 255, 255));  // Z: Purple / White
    }
} else {
    pixel.setPixelColor(0, pixel.Color(0, 0, 0));  // Turn off the NeoPixel
}
pixel.show();

  prevJerkX = jerkX;
  prevJerkY = jerkY;
  prevJerkZ = jerkZ;

  Serial.print(internalClock / 1000.0, 3); Serial.print(", ");
  Serial.print(internalClock); Serial.print(", ");
  Serial.print(altitude, 3); Serial.print(", ");
  Serial.print(currAx, 3); Serial.print(", ");
  Serial.print(currAy, 3); Serial.print(", ");
  Serial.print(currAz, 3); Serial.print(", ");
  Serial.print(jerkX, 3); Serial.print(", ");
  Serial.print(jerkY, 3); Serial.print(", ");
  Serial.print(jerkZ, 3); Serial.print(", ");

  Serial.print("LED: ");
  if (fabs(jerkX) > threshold || fabs(jerkY) > threshold || fabs(jerkZ) > threshold) {
    if (fabs(jerkX) >= fabs(jerkY) && fabs(jerkX) >= fabs(jerkZ)) {
        if (jerkX > 0) {
            Serial.println("RED");
        } else {
            Serial.println("BLUE");
        }
    } else if (fabs(jerkY) >= fabs(jerkX) && fabs(jerkY) >= fabs(jerkZ)) {
        if (jerkY > 0) {
            Serial.println("GREEN");
        } else {
            Serial.println("YELLOW");
        }
    } else {
        if (jerkZ > 0) {
            Serial.println("PURPLE");
        } else {
            Serial.println("WHITE");
        }
    }
} else {
    Serial.println("OFF");
}

  delay(1000);
}

// =========================================================================================
// HELPER FUNCTIONS ========================================================================
// =========================================================================================

float getAltitude() {
  return bmp.readAltitude(1013.25);
}

bool getAcceleration() {
  sensors_event_t accel, gyro, temp;
  if (!lsm6ds33.getEvent(&accel, &gyro, &temp)) return false;

  currAx = accel.acceleration.x;
  currAy = accel.acceleration.y;
  currAz = accel.acceleration.z;
  return true;
}

void updateAccelHistory(float history[], float newVal) {
  for (int i = 0; i < 3; ++i)
    history[i] = history[i + 1];
  history[3] = newVal;
}

void computeJerk(const float ax[], const float ay[], const float az[], float dt,
                 float &jerkX, float &jerkY, float &jerkZ) {
  jerkX = (ax[3] - ax[2]) / dt;
  jerkY = (ay[3] - ay[2]) / dt;
  jerkZ = (az[3] - az[2]) / dt;
}
