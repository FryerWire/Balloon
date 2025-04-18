
/*
    Balloon Project - Main Program
    This program reads data from an LSM6DS33 accelerometer and a BMP280 barometric pressure sensor.
    It calculates the altitude, acceleration, and jerk (rate of change of acceleration) in three axes (X, Y, Z).
    Based on the jerk values, it controls a NeoPixel LED to display different colors.
    The program also logs data to the Serial Monitor for analysis.
    The update interval for calculations and logging is set to 1 second.
*/



#include <Adafruit_LSM6DS33.h>  // LSM6DS33 6-DoF Accelerometer and Gyroscope
#include <Adafruit_BMP280.h>    // BMP280 Barometric Pressure Sensor
#include <Adafruit_NeoPixel.h>  // NeoPixel RGB LED
#include <Wire.h>               // I2C library



// Function Prototypes =====================================================================
float getAltitude();
bool getAcceleration();
void updateAccelHistory(float history[], float newVal);
void computeJerk(const float ax[], const float ay[], const float az[], float dt, float &jerkX, float &jerkY, float &jerkZ);



// Global Objects ==========================================================================
Adafruit_BMP280 bmp;                                                       // BMP280 sensor object
Adafruit_LSM6DS33 lsm6ds33;                                                // LSM6DS33 sensor object
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(1, 18, NEO_GRB + NEO_KHZ800);  // NeoPixel LED



// Global Variables ========================================================================
float accelX[4] = {0}, accelY[4] = {0}, accelZ[4] = {0};  // Acceleration history arrays
float currAx = 0, currAy = 0, currAz = 0;                 // Current acceleration values
float prevJerkX = 0.0, prevJerkY = 0.0, prevJerkZ = 0.0;  // Previous jerk values

int accelChecker = 0;                                     // Counter to ensure enough data points are collected before computing jerk

unsigned long lastTime = 0;                               // Tracks the last time the loop was executed
unsigned long internalClock = 0;                          // Internal clock for logging



/*
    setup() - Initializes the I2C bus, BMP280, LSM6DS33, and NeoPixel.
    It also sets the accelerometer range and data rate, and initializes the NeoPixel to off.
*/
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(250);

  bmp.begin(0x77);                                 // Initialize BMP280 sensor
  lsm6ds33.begin_I2C();                            // Initialize LSM6DS33 sensor
  lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);  // Set accelerometer range
  lsm6ds33.setAccelDataRate(LSM6DS_RATE_104_HZ);   // Set accelerometer data rate

  pixel.begin();  // Initialize NeoPixel
  pixel.show();   // Turn off NeoPixel

  lastTime = millis();  // Initialize timing variables
  internalClock = 0;

  // Print header for Serial Monitor logging
  Serial.println("Time [s], Internal Clock [ms], Altitude [m], Accel X [m/s^2], Accel Y [m/s^2], Accel Z [m/s^2], Jerk X [m/s^3], Jerk Y [m/s^3], Jerk Z [m/s^3]");
}



/*
    loop() - Main loop that updates the accelerometer data, computes jerk, and controls the NeoPixel based on jerk values.
    It also logs data to the Serial Monitor every second.
    The LED color changes based on the direction of the jerk.
    The threshold for jerk detection is set to 1.0 m/s^3.
*/
void loop() {
  static unsigned long lastUpdateTime = 0;    // Tracks the last update time
  static unsigned long lastLogTime = 0;       // Tracks the last log time
  const unsigned long updateInterval = 1000;  // Update interval in milliseconds
  unsigned long currentTime = millis();       // Get current time

  // Check if the update interval has passed ------------------------------------------------------
  if (currentTime - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentTime;

    float altitude = getAltitude();  // Get altitude from BMP280

    // Get acceleration data from LSM6DS33 --------------------------------------------------------
    if (!getAcceleration()) {  
      return;
    }

    // Update acceleration history ----------------------------------------------------------------
    updateAccelHistory(accelX, currAx);
    updateAccelHistory(accelY, currAy);
    updateAccelHistory(accelZ, currAz);

    // Ensure enough data points are collected ----------------------------------------------------
    if (accelChecker < 4) {  
      accelChecker++;
      return;
    }

    float dt = (currentTime - lastTime) / 1000.0;  // Calculate time interval in seconds
    lastTime = currentTime; 
    internalClock += 1000;

    float jerkX, jerkY, jerkZ;
    computeJerk(accelX, accelY, accelZ, dt, jerkX, jerkY, jerkZ);  // Compute jerk values

    float threshold = 1.0;  // Jerk threshold for LED activation

    // Control NeoPixel LED based on jerk values --------------------------------------------------
    if (fabs(jerkX) > threshold || fabs(jerkY) > threshold || fabs(jerkZ) > threshold) {
      if (fabs(jerkX) >= fabs(jerkY) && fabs(jerkX) >= fabs(jerkZ)) {
        pixel.setPixelColor(0, jerkX > 0 ? pixel.Color(255, 0, 0) : pixel.Color(0, 0, 255));        // X: Red / Blue
      } else if (fabs(jerkY) >= fabs(jerkX) && fabs(jerkY) >= fabs(jerkZ)) {
        pixel.setPixelColor(0, jerkY > 0 ? pixel.Color(0, 255, 0) : pixel.Color(255, 255, 0));      // Y: Green / Yellow
      } else {
        pixel.setPixelColor(0, jerkZ > 0 ? pixel.Color(128, 0, 128) : pixel.Color(255, 255, 255));  // Z: Purple / White
      }
    } else {
      pixel.setPixelColor(0, pixel.Color(0, 0, 0));                                                 // Turn off the NeoPixel
    }

    pixel.show();

    prevJerkX = jerkX;
    prevJerkY = jerkY;
    prevJerkZ = jerkZ;

    // Log data to Serial Monitor -----------------------------------------------------------------
    if (currentTime - lastLogTime >= updateInterval) {
      lastLogTime = currentTime;

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
            Serial.print("RED");
          } else {
            Serial.print("BLUE");
          }
        } else if (fabs(jerkY) >= fabs(jerkX) && fabs(jerkY) >= fabs(jerkZ)) {
          if (jerkY > 0) {
            Serial.print("GREEN");
          } else {
            Serial.print("YELLOW");
          }
        } else {
          if (jerkZ > 0) {
            Serial.print("PURPLE");
          } else {
            Serial.print("WHITE");
          }
        }
      } else {
        Serial.print("OFF");
      }

      Serial.print(", THRESHOLD: ");
      Serial.println(threshold, 2);  
    }
  }
}



/*
    getAltitude() - Reads the altitude from the BMP280 sensor.
    Returns the altitude in meters.
*/
float getAltitude() {
  return bmp.readAltitude(1013.25);
}



/*
    getAcceleration() - Reads acceleration data from the LSM6DS33 sensor.
    Updates the global variables `currAx`, `currAy`, and `currAz` with the acceleration values.
    Returns true if the data is successfully read, false otherwise.
*/
bool getAcceleration() {
  sensors_event_t accel, gyro, temp;
  if (!lsm6ds33.getEvent(&accel, &gyro, &temp)) return false;

  currAx = accel.acceleration.x;
  currAy = accel.acceleration.y;
  currAz = accel.acceleration.z;
  return true;
}



/*
    updateAccelHistory() - Updates the history array for acceleration values.
    Shifts the old values and adds the new value to the end of the array.
*/
void updateAccelHistory(float history[], float newVal) {
  for (int i = 0; i < 3; ++i)
    history[i] = history[i + 1];
  history[3] = newVal;
}



/*
    computeJerk() - Computes the jerk (rate of change of acceleration) for each axis.
    Takes the acceleration history arrays, time interval `dt`, and references to jerk variables.
    Updates the jerk values for X, Y, and Z axes.
*/
void computeJerk(const float ax[], const float ay[], const float az[], float dt, float &jerkX, float &jerkY, float &jerkZ) {
  jerkX = (ax[3] - ax[2]) / dt;
  jerkY = (ay[3] - ay[2]) / dt;
  jerkZ = (az[3] - az[2]) / dt;
}
