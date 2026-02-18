# rpico_mpy_monitoring_system
Application example of a small monitoring system using Raspberry Pi Pico board and MicroPython language. Target audience are enthusiasts, hobbysts and students who wishes to deep their learning in microcontrollers and embedded systems, using this example as reference for their own designs or applications.

# Tools Configuration
The application code was developed using the following configuration:
* Hardware: Raspberry Pi Pico RP2040.
* IDE: Thonny 4.1.7.
* Interpreter: MicroPython (Raspberry Pi Pico).
* Additional Package: micropython-bme280 v2.1.3.
* Bluetooth Debug App: Serial Bluetooth Terminal by Kai Morich (v1.49 03/25, obtained in Google Play Store app for Android OS).

# Hardware Setup
The following parts are used to assemble the setup:
* Raspberry Pi Pico Development board (I used the version 1 here, but I belive it can work with others from Pico Series as well).
* HC-05 Module for Bluetooth Protocol.
* BMP280 Module for Temperature and Pressure Sensors.
* HC-SR04 Module for Ultrasonic Distance Sensor.
* One LED and one Resistor with 220r for visual indicator.
* One Active Buzzer for audible indicator.
* One Tactile Switch for manual interaction.
* Breadboard (mine has 830 tie points, more than enough for this application).
* Set of Wire Jumpers: either Double Male and Male-Female connections.

# System Behavior
This system behave as following:
* Collect Temperature, Pressure and Distance values from sensors.
* Send the data to a smartphone through Bluetooth protocol and print in the IDE terminal.
* Uses the Buzzer and a LED as indicators when collecting & sending data.
* Execute the sense periodically, when a Button is pressed or when a object is very close to the distance sensor.

# Board Peripherals
The Raspberry Pi Pico peripherals used are:
* PWM for Buzzer operation.
* UART Protocol to interface with the Bluetooth Module.
* SPI Protocol to interface with the Sensors Module.
* GPIOs to interface with the Ultrasonic Sensor, LED and Tactile Switch.
* Timers to adjust the monitoring period and indicators control.
* Interruption Request for the Tactile Switch detection.

# Board Connections
Parts shall be connected to Raspberry Pi Pico board according to the following pinout:  
```
01 - UART0_TX_GP0 <-> HC05_RXD     40 - VBUS <-> HC05_VDD  
02 - UART0_RX_GP1 <-> HC05_TXD     39 - VSYS <-> HCSR04_VDD  
03 - GND                           38 - GND <-> HC05_GND  
04 - I2C1_SDA_GP2 <-> BMP280_SDA   37 - 3V3_EN  
05 - I2C1_SCL_GP3 <-> BMP280_SCL   36 - 3V3_OUT <-> BUTTON(+), BMP280_VDD  
06 -                               35 -  
07 -                               34 -  
08 - GND                           33 - GND <-> BMP280_GND  
09 -                               32 -  
10 -                               31 -  
11 -                               30 -  
12 -                               29 -  
13 - GND <-> BUZZER(-)             28 - GND  
14 -                               27 -  
15 -                               26 -  
16 -                               25 -  
17 - GP13 <-> BUZZER(+)            24 -  
18 - GND <-> 220R <-> LED(-)       23 - GND <-> HCSR04_GND  
19 - GP14 <-> BUTTON(-)            22 - GP17 <-> HCSR04_TRIG  
20 - GP15 <-> LED(+)               21 - GP16 <-> HCSR04_ECHO  
```

# Code Execution
Code was tested using Thonny IDE interface opening the "monitoring_system.py" and clicking in the Run button (green play icon).

Operation adjustments can be made changing the values in the Parameterized Constants section.

To run this code automatically after the power-up sequence, save the code directly in the board internal storage main folder or use the command "File -> Save as -> Raspberry Pi Pico" in Thonny IDE, renaming the file to "main.py". Make sure the interpreter and additional packages were installed previously in the board.

# Auxiliary Documentation Files
The file "application_flowchart.png" shows how the peripherals are interconnected.

The file "assemble_connection.png" shows how to connect all the parts to assemble the setup.

The file "rpi_pico_gpio_pinout.png" shows the Raspberry Pi Pico GPIO Table for board connections reference.

Pictures "setup_assembled_01-02.jpg" shows the setup assembled and used during my development.

The video "monitoring_system_test.mp4" shows a test of the developed system, running with the expected behavior. 

More information about the Raspberry Pi Pico board and the modules used, please check their respective datasheets, which can be obtained visiting the manufacturers websites or browsing in the internet.
