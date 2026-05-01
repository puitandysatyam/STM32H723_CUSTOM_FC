STM32H723 Custom Firmware: Feature Report
Here is a comprehensive summary of the current features and capabilities of your custom STM32H723 Flight Controller firmware, integrating the state-of-the-art Betaflight core with your custom hardware layer.

1. Flight Dynamics & Core Control
Betaflight PID Engine (bf_pid.c): Implements Betaflight's advanced Proportional, Integral, Derivative (measurement-based), and FeedForward logic, including D-Term Lowpass PT1 cascades to filter out high-frequency frame vibrations.
Betaflight Mixer (bf_mixer.c): Handles the translation of the PID outputs into physical motor commands specifically for a QuadX frame configuration.
8kHz Pseudo-loop: The mathematical core expects and processes data at an extremely fast 0.000125 second delta-time.
2. Hardware Sensors & Boot Sequence
Physical IMU Integration (bf_sensor_lsm6dso.c): Interfaces with the physical LSM6DSO accelerometer/gyroscope over SPI, reading and scaling the raw registers into G-forces and Degrees-Per-Second (dps).
Pre-Arm Safety Boot: Upon powering on, the firmware enters a blocking boot sequence. It refuses to start the motor mixing loops until it successfully verifies SPI communication with the IMU and receives at least one valid control packet from the remote controller.
3. Remote Control (RC) Link
High-Speed ESP32 Bridge (bf_rc_link.c): Instead of processing raw RC protocols natively, the flight controller receives normalized 32-byte control structures over a DMA-driven UART connection from an ESP32 bridge.
NRF24 Controller Compatibility: The data format matches the 1000-2000 mapping sent by your Arduino NRF24 controller (Throttle, Yaw, Pitch, Roll, and Arm Switch).
4. Motor Output
DShot600 Protocol (bf_dshot.c): The firmware utilizes Timer 1 (TIM1) and DMA to generate highly precise, digital DShot600 signals to drive standard ESCs, bypassing the inaccuracies of traditional analog PWM.
5. Visual Telemetry (TFT Display)
Onboard HUD (bf_display.c): Drives an ST7789-based TFT display over SPI4, refreshing at 10Hz.
Multi-Page Interface: You can cycle through three distinct telemetry pages using the K1 hardware button on the WeAct board:
Page 1 (System Status): Displays the online/offline status of the IMU, Baro, and Mag.
Page 2 (Gyro Vectors): Displays live Roll, Pitch, and Yaw degrees-per-second directly from the physical sensors.
Page 3 (RC Link): Displays the Arming status and total packet counts received from the remote.
6. USB Serial Debugging (New!)
USB CDC Virtual COM Port: The board enumerates as a USB Serial device when plugged into a computer.
Live Telemetry Stream: In the background, a heartbeat task at 2Hz (500ms) transmits a formatted string over the USB port (e.g., IMU[0.12, -0.05, 0.01] | RC_PKTS: 1205 | ARMED: 0), allowing you to monitor the physical sensors and RC link without looking at the TFT screen.