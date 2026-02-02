# WaveGo OLED Display Module

OLED display module for WaveGo robot, showing battery level, IP address, and motion status.

## Features

- Real-time battery level display (0-100%)
- IP address display (supports both static and DHCP)
- Motion status display with moving/idle indicators
- Automatic refresh with configurable intervals
- Thread-safe display updates
- Simulated data for testing

## Hardware Requirements

- ESP32 development board
- SSD1306 OLED display (128x64, I2C interface)
- I2C connections:
  - SCL: GPIO22
  - SDA: GPIO21
  - VCC: 3.3V
  - GND: GND

## Software Requirements

- ESP-IDF v4.4 or later
- FreeRTOS
- SSD1306 driver library

## Project Structure

```
wavego_display/
├── main/
│   └── main.c              # Main application
├── communication/
│   └── display/
│       ├── display.h       # Display module header
│       ├── display.c       # Display module implementation
│       ├── CMakeLists.txt  # Component CMake configuration
│       └── component.mk    # Component makefile
├── CMakeLists.txt          # Main CMake configuration
├── sdkconfig.defaults      # Default SDK configuration
└── README.md               # This file
```

## API Usage

### Initialization

```c
#include "display.h"

// Initialize display module
if (!display_init()) {
    // Handle initialization error
}
```

### Updating Display Data

```c
// Update battery level (0-100%)
display_update_battery(75);

// Update IP address
display_update_ip("192.168.1.100");

// Update motion status
display_update_motion_status("Walking", true);
```

### Refreshing Display

```c
// Manual refresh
display_refresh();

// Or let the display task handle automatic refresh
```

### Creating Display Task

```c
// Create display task
xTaskCreate(display_task, "display_task", 4096, NULL, 2, NULL);
```

## Configuration

### I2C Pins

Default I2C configuration:
- SCL: GPIO22
- SDA: GPIO21

To change pins, modify `display.c`:
```c
#define I2C_MASTER_SCL_IO  22
#define I2C_MASTER_SDA_IO  21
```

### Display Settings

Display settings in `display.c`:
- Display width: 128 pixels
- Display height: 64 pixels
- I2C address: 0x3C
- I2C frequency: 400kHz

### Task Configuration

- Display task stack size: 4096 bytes
- Display task priority: 2 (above idle)
- Refresh interval: 1 second

## Building and Flashing

1. Set up ESP-IDF environment:
   ```bash
   . $IDF_PATH/export.sh
   ```

2. Configure the project:
   ```bash
   idf.py set-target esp32
   idf.py menuconfig
   ```

3. Build the project:
   ```bash
   idf.py build
   ```

4. Flash to device:
   ```bash
   idf.py -p /dev/ttyUSB0 flash
   ```

5. Monitor serial output:
   ```bash
   idf.py -p /dev/ttyUSB0 monitor
   ```

## Testing

The main application includes a simulation task that:
- Gradually drains battery from 100% to 0%
- Cycles through different motion states
- Generates simulated IP addresses
- Updates display every 10-60 seconds

To test with real data, modify `main.c` to use actual sensor readings and network information.

## Dependencies

- FreeRTOS
- ESP32 I2C driver
- SSD1306 OLED driver
- ESP32 WiFi stack (for real IP address)

## License

This project is part of the WaveGo robot system.