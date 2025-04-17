
#include <iostream>
#include <fstream>
#include <string>



// Function Prototype =============================================================================
void readAltitudeData(float currentAltitude);



// Global Variables ===============================================================================
float fourPointAltitude[4] = {0};  // Array to hold four altitude values
int fourPointChecker = 0;          // Counter to track how many altitude values have been read



/*
    waitOneSecond() function is a busy-wait loop that counts down from a specified number of cycles.
    The number of cycles is calculated based on the CPU frequency (64 MHz) and the desired delay (1 second).
*/
void waitOneSecond() {
    volatile unsigned long long cycles = 64000000;  // 64 million cycles for 1 second at 64 MHz

    while (cycles > 0) {
        cycles--;
    }
}



/*
    main function reads altitude data from a file named "altitude_data.txt" located on the desktop.
    It processes each line of the file, converting it to a float and passing it to the readAltitudeData function.
*/
int main() {
    std::ifstream inFile("C:\\Users\\maxwe\\OneDrive\\Documents\\PlatformIO\\Projects\\Balloon\\Altitude Testing\\altitude_data.txt");  // Absolute path to the file
    if (!inFile) {
        std::cerr << "Error: Could not open altitude_data.txt" << std::endl;
        return 1;
    }

    std::string line;

    while (std::getline(inFile, line)) {
        try {
            float altitude = std::stof(line);  // Convert string to float
            readAltitudeData(altitude);        // Call the function with the altitude value

            waitOneSecond();                   // Simulate 1 Hz internal clock tick
        } 
        catch (const std::exception& e) {
            std::cerr << "Error: Invalid data in file - " << e.what() << std::endl;
        }
    }

    inFile.close(); // Close the file

    return 0;
}



/*
    readAltitudeData function processes the current altitude value and calculates the jerk based on the last four altitude readings.
    It also determines the LED state based on the calculated jerk value.
*/
void readAltitudeData(float currentAltitude) {
    // Shifts the fourPointAltitude values to the left
    for (int i = 0; i < 3; i++) {
        fourPointAltitude[i] = fourPointAltitude[i + 1];
    }

    fourPointAltitude[3] = currentAltitude;  // Add the new altitude value to the end of the array

    // Checks if there are four altitude values in the array
    if (fourPointChecker < 4) {
        fourPointChecker++;
        std::cout << "Need more data points" << std::endl;

        return;
    }

    float dt = 1.0;  // Time interval (in seconds) between altitude readings

    // Velocity -----------------------------------------------------------------------------------
    float v1 = (fourPointAltitude[1] - fourPointAltitude[0]) / dt;
    float v2 = (fourPointAltitude[2] - fourPointAltitude[1]) / dt;
    float v3 = (fourPointAltitude[3] - fourPointAltitude[2]) / dt;

    // Acceleration -------------------------------------------------------------------------------
    float a1 = (v2 - v1) / dt;
    float a2 = (v3 - v2) / dt;

    // Jerk ---------------------------------------------------------------------------------------
    float jerk = (a2 - a1) / dt;

    // LED Output -------------------------------------------------------------------------------
    if (jerk > 0) {
        std::cout << "GREEN" << std::endl;  // Turn on Green LED if jerk is positive
    } else if (jerk < 0) {
        std::cout << "RED" << std::endl;    // Turn on Red LED if jerk is negative
    } else {
        std::cout << "OFF" << std::endl;    // Turn off LED if jerk is zero
    }

    // std::cout << "Jerk: " << jerk << std::endl;  // Print the calculated jerk value
}
