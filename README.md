**CAN-Based Smart Vehicle Subsystem Simulation & Diagnostics**
-----------------------------------------------------------------------------------------------------------------------------------------------
**Project Overview**

The project titled "CAN-Based Smart Vehicle Subsystem Simulation and Diagnostics" aims to simulate and implement a vehicle health monitoring system using STM32F407VGT6, BeagleBone Black, CAN Bus, and ESP modules. It collects real-time data from sensors measuring engine RPM, temperature, brake status and speed. The collected data is saved in CSV files and analyzed using machine learning algorithms for predictive fault detection alert. The system alerts users before faults occur and sends notifications to a dashboard, cloud, and mobile devices.
 ---------------------------------------------------------------------------------------------------------------------------------------       
**Project Functionalities Implemented**

1. Real-time data acquisition from sensors (RPM, temperature, brake pressure, speed and Battery status).
2. CAN Bus communication to transfer sensor data.
3. Dashboard display using BeagleBone Black.
4. Fault injection and simulation for testing.
5. Machine Learning algorithms for fault prediction.
6. Alerts for engine failure, overheating, brake failure, and overspeeding.
7. Cloud logging and mobile alert via ESP and ThingSpeak/Thingsboard/IFTTT(optional).
---------------------------------------------------------------------------------------------------------------------------------------
**Hardware Requirements**

**Mandatory:**
- STM32F407VGT6 microcontroller
- BeagleBone Black
- MCP2551 + SN65HVD230 CAN Transceivers
- ESP32 for WiFi and cloud communication

** Sensors:**
- RPM Sensor: LM393-based Hall effect sensor
- Temperature Sensor: LM35
- Brake Pressure Sensor: MPX5700AP
- Battery sensor (INA219)
- Speed Detection: Ultrasonic sensor

---------------------------------------------------------------------------------------------------------------------------------------
**Connection/Wiring Diagram :**




**Block Diagram:**
 


---------------------------------------------------------------------------------------------------------------------------------------
**Flow of Project:**

**1.	Sensor Data Acquisition**

•	Sensors are connected to two STM32F407VGT6 microcontrollers.
•	Node 1 handles vehicle parameters: RPM, brake pressure, temperature, and GPS.
•	Node 2 manages battery parameters: voltage, current, temperature, and calculates SoC (State of Charge) and SoH (State of Health).

**2.	Local Processing by STM32 Nodes**

•	Each STM32 reads analog and digital signals from sensors.
•	Battery Node performs SoC/SoH estimation using coulomb counting or voltage mapping.
•	Each node packages its data into CAN frames.

**3.	CAN Bus Communication**

•	Both STM32s transmit their data on the CAN bus using MCP2551 transceivers.
•	CAN Bus ensures robust, low-latency communication across nodes.
•	A common bus connects the STM32s to the BeagleBone Black via an SN65HVD230 transceiver.

**4.	BeagleBone Black: Central Processing & Dashboard**

•	BeagleBone listens to CAN messages using python-can (CAN0 interface).
•	It decodes data and displays real-time sensor values on a web-based dashboard.
•	Logs are saved as CSV files for machine learning training and evaluation.

**5.	Fault Detection Using Machine Learning**

•	A trained machine learning model (e.g., Random Forest or SVM) runs on the BeagleBone.
•	The model uses sensor inputs to classify the system state: normal or fault.
•	Faults include overheating, battery drain, overspeeding, brake failure, etc.

**6.	UART Communication to ESP32**

•	When a fault is detected, BeagleBone sends a fault message (e.g., "FAULT: Battery Overheat") via UART to the ESP32 module.

**7.	Wireless Cloud Communication**

•	ESP32 connects to WiFi and uploads the fault message to ThingSpeak (cloud).
•	The ThingSpeak channel is linked to IFTTT (If This Then That) automation service.

**8.	Mobile Notification via IFTTT**

•	IFTTT monitors the ThingSpeak field.
•	When a new fault message appears, it triggers an SMS or email notification to the user.

**9.	Real-Time Demonstration**

•	Push buttons can be used to simulate faults or manually for demonstration.
•	The system shows real-time sensor changes, CAN transmission, ML-based fault detection, and live alerts to a mobile device.

---------------------------------------------------------------------------------------------------------------------------------------
******Demonstration and Fault Simulation******

The system can demonstrate various fault scenarios by injecting conditions such as:
- Simulating no engine RPM → Engine failure
- Exceeding temperature threshold → Overheating
- No brake pressure detected during motion → Brake failure
- Speed exceeding defined limit → Over speeding

All such conditions trigger alerts to the dashboard, cloud, and mobile.
---------------------------------------------------------------------------------------------------------------------------------------
**Current Status of Predictive Maintenance in Industry**

	OBD-II and CAN-based diagnostics: Fully implemented in vehicles.
	Real-time alerts and remote diagnostics: Implemented in smart vehicles (Tesla, Hyundai, Mahindra).
	Predictive maintenance using ML: Still emerging, mostly in fleets and premium EVs.
	Cloud + ML integration: Implemented in high-end systems with OTA capabilities.
        
---------------------------------------------------------------------------------------------------------------------------------------   
**Conclusion**

This project successfully demonstrates the design and implementation of a real-time, modular smart vehicle diagnostic system using STM32 microcontrollers, CAN communication, and machine learning. By integrating critical vehicle and battery subsystems through a dual-node STM32 architecture, the system simulates the behavior of distributed automotive ECUs found in modern electric and autonomous vehicles.
The BeagleBone Black platform acts as a central data aggregator and intelligent decision-maker, applying machine learning models to accurately detect faults based on sensor patterns. Real-time diagnostics are further enhanced through the integration of an ESP32 module, which enables cloud-based alerting and remote fault notification via ThingSpeak and IFTTT.
The project validates a complete, end-to-end data acquisition and fault detection pipeline—from sensor input to mobile notification—while adhering to industry practices such as CAN protocol usage, SoC/SoH computation, and modular hardware design. This approach not only enhances safety and maintainability in vehicle systems but also enables predictive maintenance using cloud and AI technologies.
Ultimately, the system provides a scalable, cost-effective, and industry-aligned solution for future intelligent vehicle platforms, particularly those adopting electric and connected vehicle architectures.

        
