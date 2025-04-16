
#include <Adafruit_BMP280.h>  // BMP280 Sensor from library
#include <Wire.h>



// Function Prototype =============================================================================
void readAltitudeData(float currentAltitude);
void waitOneSecond();



// Global Objects and Constants ===================================================================
Adafruit_BMP280 bmp; // Default I2C address is 0x77

#define GREEN_LED_PIN 1
#define RED_LED_PIN 2

float fourPointAltitude[4] = {0};
int fourPointChecker = 0;



/*
    setup function initializes the serial communication, sets up the LED pins, and initializes the BMP280 sensor.
    It also configures the BMP280 sensor for oversampling and filtering.
*/
void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial to be ready

  // Initialize LEDs
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

  // Initialize BMP280
  if (!bmp.begin(0x77)) {
    Serial.println("Could not find BMP280 sensor!");
    while (1);
  }

  // Optional: Configure oversampling for better accuracy
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
}



/*
    loop function continuously reads the altitude from the BMP280 sensor and processes it.
    It calls the readAltitudeData function with the current altitude value and waits for 1 second before repeating.
*/
void loop() {
  float altitude = bmp.readAltitude(1013.25);  // Replace with your known sea level pressure
  readAltitudeData(altitude);
  waitOneSecond();  // Sample at 1 Hz
}



/*
    waitOneSecond function introduces a delay of 1 second (1000 milliseconds) to simulate a 1 Hz internal clock tick.
    It uses the delay function from the Arduino library to achieve this.
*/
void waitOneSecond() {
  delay(999);  // 1-second delay (1000 ms)
}



/*
    readAltitudeData function processes the current altitude value and calculates the jerk based on the last four altitude readings.
    It also determines the LED state based on the calculated jerk value.
*/
void readAltitudeData(float currentAltitude) {
  // Four Point Checkers ==========================================================================
  for (int i = 0; i < 3; i++) {
    fourPointAltitude[i] = fourPointAltitude[i + 1];
  }

  fourPointAltitude[3] = currentAltitude;

  if (fourPointChecker < 4) {
    fourPointChecker++;
    Serial.println("Collecting the data");

    return;
  }

  float dt = 1.0;

  // Jerk Calulcations ============================================================================
  // Velocity -------------------------------------------------------------------------------------
  float v1 = (fourPointAltitude[1] - fourPointAltitude[0]) / dt;
  float v2 = (fourPointAltitude[2] - fourPointAltitude[1]) / dt;
  float v3 = (fourPointAltitude[3] - fourPointAltitude[2]) / dt;

  // Acceleration ---------------------------------------------------------------------------------
  float a1 = (v2 - v1) / dt;
  float a2 = (v3 - v2) / dt;

  // Jerk -----------------------------------------------------------------------------------------
  float jerk = (a2 - a1) / dt;


  // Outputs ======================================================================================
  // Output Results -------------------------------------------------------------------------------
  Serial.print("Altitude: "); Serial.print(currentAltitude);
  Serial.print(" | Jerk: "); Serial.println(jerk);

  // LED Output -----------------------------------------------------------------------------------
  if (jerk > 0) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, LOW);
  } else if (jerk < 0) {
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH);
  } else {
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);
  }
}
