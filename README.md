
# Balloon Project with Adafruit Clue nRF52840  
**Code Written By:** Maxwell Seery



## Overview  
A weather balloon platform that collects, logs, and analyzes atmospheric data using the Adafruit Clue board and various onboard sensors. The main objective of this project is to measure the rate of change in acceleration (jerk) and compare the collected flight data to standard atmospheric models. The project is divided into two primary parts:

- **Data Acquisition**: Performed in-flight using C++ code on the Adafruit Clue board.
- **Post-Processing**: Conducted using Python to analyze and visualize the data.



## Features
- Real-time sensor logging using BMP280 and LSM6DS33
- Data storage on 2MB internal QSPI flash with `SdFat`
- Python-based visualization and statistical analysis
- LED feedback system driven by jerk thresholds per axis
- LaTeX-based report generation and PDF compilation

**LED Behavior Table**:
| Axis | Jerk < 0 | Jerk > 0 |
|------|----------|----------|
| X    | Blue     | Red      |
| Y    | Yellow   | Green    |
| Z    | White    | Purple   |



## Hardware Info
- **Microcontroller**: Adafruit Clue nRF52840 (Cortex-M4, 64 MHz, 1MB code flash)
- **Sensors**:
  - BMP280 – Altitude, Pressure, and Temperature
  - LSM6DS33 – Accelerometer and Gyroscope (6-DoF)
- **Power Supply**: USB or 3x AAA batteries
- **Onboard Storage**: 2MB QSPI flash (formatted to FAT32)
- **Outputs**: NeoPixel RGB LED, white illumination LEDs



## Software Info
### Requirements
- **PlatformIO** 
- **Python 3.8+** 
- **LaTeX Setup**:
  - [MaTeX](https://github.com/sympy/matex) for inline LaTeX rendering in Python
  - [Strawberry Perl](https://strawberryperl.com/) is required for live `.tex` compilation and rendering



## Libraries Used
### PlatformIO (C++)
```ini
lib_deps =
  adafruit/Adafruit Unified Sensor
  adafruit/Adafruit BMP280 Library
  adafruit/Adafruit LSM6DS
  adafruit/Adafruit NeoPixel@^1.12.5
  adafruit/SdFat - Adafruit Fork@^2.2.54
  adafruit/Adafruit SPIFlash@^5.1.1
```



## File Structure
### File Tree
```
Balloon/
├── Data/
│   ├── live_serial_monitoring.py
│   ├── post_data_processing.py
│   ├── static_data_test.csv
│   ├── dynamic_data_test.csv
│   └── README.md
├── Documentation/
│   ├── LaTeX/
│   │   ├── docs.aux
│   │   ├── docs.fdb_latexmk
│   │   ├── docs.fls
│   │   ├── docs.log
│   │   ├── docs.pdf
│   │   ├── docs.synctex.gz
│   │   ├── docs.tex
│   │   └── README.md
│   ├── changelog.txt
│   └── README.md
├── Images/
│   ├── image.png
│   └── README.md
├── src/
│   ├── main.cpp
│   ├── testing.exe
│   └── README.md
├── Utilities/
│   ├── adafruit-circuitpython-clue_nrf52840_express-en_US-9.2.7.uf2
│   └── README.md
```

### File Descriptions
#### Data/
- `live_serial_monitoring.py`: Real-time serial monitor for incoming sensor data.
- `post_data_processing.py`: Reads CSV logs, computes statistics, and generates graphs.
- `static_data_test.csv`: Sample test data for post-processing validation.
- `dynamic_data_test.csv`: Sample test data for post-processing validation.
- `README.md`: Explains the purpose and usage of the `Data/` folder.

#### Documentation/
- `LaTeX/docs.tex`: Primary LaTeX source file for full project report.
- `LaTeX/docs.pdf`: Compiled report document.
- `LaTeX/*.aux, *.fdb_latexmk, *.fls, *.log, *.synctex.gz`: Auxiliary LaTeX files.
- `changelog.txt`: Chronological list of project changes and additions.
- `README.md`: Overview and compilation instructions for documentation.

#### Images/
- `image.png`: Output graph generated from data processing.
- `README.md`: Explains the origin and purpose of included images.

#### src/
- `main.cpp`: Core embedded software written for the Clue board.
- `testing.exe`: Executable simulation for data testing without hardware.
- `README.md`: Overview and usage for the source folder.

#### Utilities/
- `adafruit-circuitpython-clue_nrf52840_express-en_US-9.2.7.uf2`: CircuitPython firmware for Clue board (used for formatting internal flash).
- `README.md`: Documentation for firmware and board configuration.
