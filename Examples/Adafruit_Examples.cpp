// Adafruit_Examples.cpp

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SD.h> // For SD card

// Create an instance of the accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Define the LED pin
const int ledPin = 13; // Built-in LED on most Arduino boards

// Acceleration threshold to turn on the LED
const float accelerationThreshold = 2.0; // Adjust as needed

// SD card chip select pin
const int chipSelect = 10;

// File object for the SD card
File dataFile;

// Logging frequency (in milliseconds)
const unsigned long logInterval = 100; // Log data at 10 Hz
unsigned long previousMillis = 0;

// Internal clock variables
unsigned long startMillis = 0; // Tracks when the program started
unsigned long elapsedMillis = 0; // Tracks elapsed time since the program started

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Accelerometer Test");

  // Initialize the accelerometer
  if (!accel.begin()) {
    Serial.println("No ADXL345 detected. Check your wiring!");
    while (1);
  }

  // Set the range of the accelerometer
  accel.setRange(ADXL345_RANGE_16_G);

  // Print the range to the serial monitor
  Serial.print("Range set to: ");
  switch (accel.getRange()) {
    case ADXL345_RANGE_2_G: Serial.println("+-2G"); break;
    case ADXL345_RANGE_4_G: Serial.println("+-4G"); break;
    case ADXL345_RANGE_8_G: Serial.println("+-8G"); break;
    case ADXL345_RANGE_16_G: Serial.println("+-16G"); break;
  }

  // Set the LED pin as output
  pinMode(ledPin, OUTPUT);

  // Initialize the SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialized.");

  // Initialize the internal clock
  startMillis = millis(); // Record the start time
}

void loop() {
  // Update the elapsed time
  elapsedMillis = millis() - startMillis;

  // Get a new sensor event
  sensors_event_t event;
  accel.getEvent(&event);

  // Print acceleration data to the serial monitor
  Serial.print("Elapsed Time (ms): "); Serial.print(elapsedMillis); Serial.print(" ");
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print(" m/s^2 ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print(" m/s^2 ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.println(" m/s^2 ");

  // Check if the acceleration exceeds the threshold
  if (abs(event.acceleration.x) > accelerationThreshold ||
      abs(event.acceleration.y) > accelerationThreshold ||
      abs(event.acceleration.z) > accelerationThreshold) {
    digitalWrite(ledPin, HIGH); // Turn on the LED
  } else {
    digitalWrite(ledPin, LOW); // Turn off the LED
  }

  // Log data to the SD card at the specified frequency
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= logInterval) {
    previousMillis = currentMillis;

    // Open the file for appending
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      // Write elapsed time and acceleration data to the file
      dataFile.print("Elapsed Time (ms): "); dataFile.print(elapsedMillis);
      dataFile.print(", X: "); dataFile.print(event.acceleration.x);
      dataFile.print(", Y: "); dataFile.print(event.acceleration.y);
      dataFile.print(", Z: "); dataFile.println(event.acceleration.z);
      dataFile.close();
    } else {
      Serial.println("Error opening datalog.txt");
    }
  }

  // Delay for a short period
  delay(10); // Small delay to avoid overwhelming the serial monitor
}