# SmartShoppingCart

This project implements remote control, automatic following, and beacon-triggered recall for a smart shopping cart, based on the STM32 microcontroller and UWB anchors/tags.

## Features

* **Remote Control**: Control the shopping cart using a UWB-based remote controller
* **Automatic Following**: The cart automatically follows the user using UWB AoA (Angle of Arrival) technology
* **Beacon-Triggered Recall**: In NO\_FULL mode, the beacon sends a signal to the base station to initiate motor-driven following for automatic return
* **Multiple Operation Modes**:

  * Lock Mode
  * Follow Mode
  * Recall Mode
  * Remote Control Mode
* **Real-time Display**: OLED display shows status information
* **Battery Monitoring**: Low battery detection and charging status
* **Motor Control**: Precise motor speed and direction control using encoders

## Hardware Requirements

* **Microcontroller**: STM32F103 series (STM32F103RC/C8/T8)
* **UWB Module**: UWB AoA positioning module with anchors and tags
* **Display**: OLED I2C display
* **Motors**: DC motors with encoders
* **Sensors**:

  * TOF (Time of Flight) sensors (optional)
  * Battery voltage monitoring
* **Communication**: UART interfaces for UWB communication

## Project Structure

```
SmartShoppingCart/
├── Components/
│   ├── HAL/                    # Hardware Abstraction Layer
│   │   ├── control.c         # Main control logic and motor control
│   │   ├── uwb.c             # UWB communication and data processing
│   │   ├── motor.c           # Motor driver and encoder handling
│   │   ├── uart4_process.c   # UART data queue processing
│   │   ├── hal_usart.c       # USART hardware abstraction
│   │   ├── hal_iic.c         # I2C hardware abstraction
│   │   ├── oled_i2c.c        # OLED display driver
│   │   ├── tof_i2c.c         # TOF sensor driver
│   │   ├── timer.c           # Timer configuration
│   │   ├── delay.c           # Delay functions
│   │   ├── dataoperation.c   # Data processing utilities
│   │   ├── Periph_init.c     # Peripheral initialization
│   │   └── stm32f10x_it.c    # Interrupt handlers
│   ├── OSAL/                   # Operating System Abstraction Layer
│   │   ├── OSAL.c            # Core OSAL implementation
│   │   ├── OSAL_Task.c       # Task management
│   │   ├── OSAL_Clock.c      # Clock and timing services
│   │   ├── OSAL_Error.c      # Error handling
│   │   └── OSAL_Comdef.h       # Common definitions
│   └── Main/                   # Main application
│       ├── main.c              # Entry point and system initialization
│       └── stm32f10x_conf.h    # STM32 configuration
├── Projects/
│   └── USER/                   # Keil MDK project files
│       └── RTE/                # ARM CMSIS RTE components
└── README.md
```

## Software Dependencies

* **IDE**: Keil MDK-ARM (recommended)
* **CMSIS**: ARM Cortex Microcontroller Software Interface Standard
* **STM32 Standard Peripheral Library**: For STM32F10x series
* **OSAL**: Custom Operating System Abstraction Layer for task management

## Getting Started

### 1\. Hardware Setup

1. Connect the UWB module to the STM32 via UART
2. Connect the OLED display via I2C
3. Connect motor drivers to the designated GPIO pins
4. Connect encoders to the EXTI pins
5. Ensure proper power supply for all components

### 2\. Build Instructions

1. Open the project in Keil MDK-ARM
2. Navigate to `Projects/USER/` directory
3. Open the `.uvprojx` project file
4. Select the appropriate target device (STM32F103RC/C8/T8)
5. Build the project (F7)
6. Download to the target hardware

### 3\. Configuration

Key configuration options can be found in:

* `Components/HAL/uwb.h`: UWB communication settings
* `Components/HAL/control.h`: Control parameters and modes
* `Components/Main/stm32f10x_conf.h`: STM32 peripheral configuration

## Usage

### Operation Modes

1. **Lock Mode**: Cart remains stationary and locked
2. **Follow Mode**: Cart automatically follows the user at a set distance
3. **Recall Mode**: Cart returns to the user's position
4. **Remote Mode**: Manual control via remote controller

### Remote Controller Commands

* **Turn Up/Down**: Move forward/backward
* **Turn Left/Right**: Steer left/right
* **Mode Button**: Switch between operation modes
* **Recall Button**: Activate beacon-triggered recall function (in NO\_FULL mode)
* **Lock Button**: Lock/unlock the cart

## Technical Details

### UWB AoA Positioning

The system uses UWB (Ultra-Wideband) technology with Angle of Arrival (AoA) measurement to determine:

* Distance between tag and anchor (in cm)
* Angle of the tag relative to the anchor (in degrees)
* Signal strength (RSSI) for quality assessment

### Control Algorithm

The control system implements:

* Kalman filtering for smooth angle and distance estimation
* PID-like control for motor speed adjustment
* Direction determination based on AoA data

### Communication Protocol

The UWB communication uses a custom frame structure:

* Start of Packet (SOP): 0x2A
* End of Packet (FCS): 0x23
* Support for multiple frame types (position, configuration, heartbeat, etc.)

## File Descriptions

### Key Source Files

|File|Description|
|-|-|
|`main.c`|System initialization and main loop|
|`control.c`|Core control logic, motor control algorithms|
|`uwb.c`|UWB data parsing and AoA processing|
|`motor.c`|Motor PWM control and encoder reading|
|`oled_i2c.c`|OLED display driver for status output|
|`uart4_process.c`|Recall function implementation|

## Safety Considerations

* Ensure proper motor current limiting to prevent damage
* Implement emergency stop functionality
* Monitor battery levels to prevent over-discharge
* Test in safe environments before deployment

## License

This project is provided as-is for educational and development purposes.

## Contact

For questions or contributions, please refer to the project repository.

