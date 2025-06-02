# PG-DESD_project
CAN BASED SMART VEHICLE SUBSYSTEM SIMULATION AND DIAGNOSTICS
Project Title:
Advanced Simulation and Diagnostics of a CAN-Based Smart Vehicle Subsystem Incorporating a Battery Management Unit
Objective:
This project endeavors to model, develop, and validate a real-time intelligent vehicular diagnostic system. It integrates STM32-based embedded nodes, Controller Area Network (CAN) communication, a BeagleBone Black platform for GUI-based visualization and data logging, and cloud-enabled mobile alerts. Central to the system is a modularized architecture featuring a dedicated Battery Management Subsystem (BMS1) with real-time SoC (State of Charge) and SoH (State of Health) computation. Predictive diagnostics are achieved via machine learning algorithms trained on real-world sensor data.
________________________________________
Hardware Architecture and Components:
1.	STM32F407VGT6 Microcontroller Boards – 2 Units
o	Node 1: Core vehicular metrics (RPM, speed, temperature, brake pressure, GPS)
o	Node 2: Dedicated Battery Management Subsystem (voltage, current, battery temperature, SoC, SoH)
2.	Sensors
o	RPM Sensor (Hall effect) – 1
o	Temperature Sensors (LM35/DS18B20) – 2
o	Brake Pressure Sensor (MPX5700AP) – 1
o	Speed Sensor (Wheel encoder) – 1
o	GPS Module (NEO-6M) – 1
o	Voltage/Current Sensor (INA219) – 1
3.	CAN Communication Interface
o	MCP2551 Transceivers – 2 (one per STM32 node)
o	SN65HVD230 CAN Transceiver – 1 (BeagleBone interface)
o	Termination Resistors (120 Ω) – 2 (for bus integrity)
4.	Computing and Communication Nodes
o	BeagleBone Black – 1 (acts as data aggregator, logger, dashboard host, and ML runtime environment)
o	ESP8266/ESP32 – 1 (used for real-time fault broadcasting to cloud and IFTTT-based mobile notification system)
5.	Power Supply Infrastructure
o	12V Li-Ion Battery Pack
o	5V Buck Converters / AMS1117 Voltage Regulators
6.	Optional Enhancements
o	OLED Display Module
o	Electromechanical actuators (relays) and indicators (buzzers, LEDs)
o	Manual Fault Injection Interface (Push Buttons)
________________________________________
System Block Diagram:
 
________________________________________
System Wiring and Physical Interconnection:
![Wiring Diagram](/mnt/data/WhatsApp Image 2025-06-01 at 12.59.04_0279757a.jpg)
________________________________________
Step-wise Hardware Connection Procedure:
1.	Mount both STM32F407VGT6 boards on your breadboard or PCB base.
2.	Connect RPM sensor to STM32 Node 1 using GPIO with interrupt support.
3.	Interface LM35/DS18B20 temperature sensors to ADC/GPIO pins on both STM32 nodes.
4.	Connect Brake Pressure Sensor (analog output) to an ADC channel on Node 1.
5.	Connect Speed Sensor (digital encoder) to a GPIO Timer Input Capture pin on Node 1.
6.	Wire GPS Module to UART interface of Node 1 (e.g., USART2).
7.	Interface INA219 Voltage/Current sensor to I2C bus of Node 2 (e.g., I2C1 on PB6/PB7).
8.	Connect MCP2551 CAN transceivers to each STM32:
o	CAN_TX to TX pin, CAN_RX to RX pin
o	CANH and CANL to shared CAN bus line
o	Include 120 Ω termination resistors at each end of CAN line
9.	Connect BeagleBone Black to CAN bus using SN65HVD230 transceiver
10.	Interface ESP8266/ESP32 via UART to STM32 Node 2 or BeagleBone for fault broadcast
11.	Power both STM32 boards with 5V (from buck converter)
12.	Connect battery pack to INA219 input and power infrastructure
13.	Optionally, connect OLED display and buzzer to STM32 GPIOs for local status display
14.	Add push buttons connected to GPIOs on STM32 for fault injection (e.g., simulate brake failure)
________________________________________
Development Lifecycle and Methodology:
1.	Requirements Engineering & System Decomposition
o	Define functional boundaries of each ECU node
o	Establish CAN message arbitration, identifiers, and payload structures
2.	Hardware Setup and Integration
o	Configure analog and digital sensor connections to STM32s
o	Initialize I2C, UART, and ADC peripherals via STM32CubeMX
o	Deploy MCP2551 CAN transceivers; verify bus integrity
3.	Embedded Software Development
o	Develop firmware for periodic sensor acquisition and fault monitoring
o	Implement Coulomb counting and SoC/SoH algorithms
o	Format and transmit structured CAN messages
o	Simulate fault events via GPIO-triggered injection routines
4.	BeagleBone Black Integration
o	Use Python-can to parse CAN traffic
o	Develop Flask-based dashboard for live telemetry
o	Log readings into timestamped CSV datasets
5.	Machine Learning Pipeline
o	Preprocess historical data with labeled fault states
o	Train predictive models (e.g., Random Forest, SVM) and evaluate with cross-validation
o	Deploy inference model for real-time fault classification
6.	IoT-Based Alert System
o	ESP8266 transmits fault event data to ThingSpeak cloud platform
o	IFTTT triggers notifications to registered devices (SMS/email)
7.	Industry-Oriented Demonstration
o	Operate full system in real-time with live telemetry
o	Simulate faults using push-button interface
o	Visualize data on dashboard; display alerts on mobile device
________________________________________
Justification for STM32 Integration:
•	STM32 microcontrollers, particularly the F4 series, offer deterministic real-time performance with hardware CAN peripherals, multiple ADC channels, and rich I/O configuration capabilities.
•	Their extensive software ecosystem (STM32CubeIDE, HAL libraries) facilitates modular firmware development.
•	Proven deployment in industrial and automotive systems positions STM32 as the de facto choice for embedded diagnostics platforms.
Necessity of Dual STM32 Architecture:
•	The bifurcation of the system into two independent microcontroller units (Vehicle Node + BMS Node) reflects the distributed nature of modern automotive ECUs.
•	This modularization enhances scalability, fault isolation, and mirrors industry topology for subsystems such as engine control, battery management, and telematics.
•	Each STM32 unit communicates via CAN, enabling robust node-to-node fault detection and message relay.
Alternatives to STM32: Comparative Evaluation
Microcontroller	Advantages	Limitations
Arduino Uno/Nano	Simple, low-cost, beginner-friendly	No native CAN, 8-bit, limited ADC resolution
ESP32	Integrated WiFi/BT, cost-effective	Software-emulated CAN, less reliable ADC
Arduino Due	32-bit ARM Cortex-M3, CAN support	Outdated support, lower ecosystem maturity
Raspberry Pi Pico	Dual-core MCU, affordable	No CAN controller, weaker RTOS compatibility
Teensy 4.1	High speed, extensive I/O capabilities	Higher cost, niche developer base
STM32 strikes a critical balance between peripheral richness, real-time processing, and integration flexibility.
________________________________________
Fault Simulation Scenarios and Diagnostic Thresholds:
Fault Category	Simulation Strategy	Evaluation Criteria
Engine Failure	Disconnect RPM sensor	RPM signal = 0 while ignition ON
Brake System Fault	Zero pressure during braking	Pressure reading = 0 psi
Thermal Overload	Artificially heat LM35	Temp reading > 90°C
Overspeeding	Pulse encoder with high frequency	Speed exceeds set threshold
Battery Drain	Induce rapid SoC drop	SoC < 10%
Battery Aging	Emulate degraded capacity	SoH < 70%
________________________________________
Expected Results and Demonstrable Outcomes:
•	Accurate acquisition and transmission of subsystem sensor data
•	Effective CAN-based message routing and fault broadcasting
•	Real-time dashboard visualization and time-series data logging
•	Fault prediction accuracy exceeding 90% via deployed ML model
•	Cloud-mediated alert delivery with minimal latency
This architecture, which adheres to contemporary vehicular diagnostic frameworks, offers a scalable, modular, and intelligent solution suitable for applications in next-generation electric and autonomous vehicles.

