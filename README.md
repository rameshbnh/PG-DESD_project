CAN-based Smart Vehicle Sub-system: Simulation and Diagnostics

This project demonstrates a smart vehicle sub-system designed for simulating and diagnosing key automotive parameters using the Controller Area Network (CAN) protocol. It integrates embedded systems, sensor data acquisition, real-time data logging, machine learning-based fault detection, and IoT-based remote monitoring.
Table of Contents

    Overview

    System Architecture

    Hardware Components

    Data Flow

    Features

    Machine Learning Fault Detection

    Cloud and Dashboard Integration

    Technologies Used

    Folder Structure

    Future Improvements

Overview

The system is built using three CAN nodes:

    Node-1 (STM32F407) for engine-related data

    Node-2 (STM32F407) for battery-related data

    Node-3 (BeagleBone Black) acting as the central controller and CAN-to-Internet gateway

Each node collects specific sensor data, transmits it over the CAN bus, and allows for real-time monitoring, fault detection, and remote visualization via a Flask dashboard and MQTT-enabled cloud dashboard.
System Architecture

[LM35, Hall Sensor] --> Node-1 (STM32F407) --> MCP2551
                                             ↓
             MCP2551 <-- Node-2 (STM32F407) <-- INA219, LM35, Motor Control
                            ↓
               SN65HVD230 CAN Transceiver
                            ↓
                BeagleBone Black (Node-3)
                            ↓
        Flask Dashboard | MQTT (ThingsBoard) | CSV Logging | ML Model

Hardware Components

    STM32F407VGT6 – Microcontroller for Node-1 and Node-2

    BeagleBone Black – Central controller (Node-3)

    CAN Transceivers:

        MCP2551 (x2) for STM32 nodes

        SN65HVD230 (x1) for BeagleBone Black

    Sensors:

        LM35 (x2) – For engine and battery temperature monitoring

        Hall-Effect Sensor – Measures vehicle speed and RPM

        INA219 – Measures voltage, current, SOC, and SOH of the battery

    Motor:

        Acts as engine load (Node-1) and battery load (Node-2)

    Motor Control Module:

        Potentiometer-based variable control

    Battery:

        11.1V, 3000mAh Lithium battery

Data Flow

    Node-1 collects RPM, speed, and engine temperature data.

    Node-2 measures battery voltage, current, SOC, SOH, and temperature.

    Both nodes transmit data over the CAN bus using MCP2551 transceivers.

    The BeagleBone Black receives data via SN65HVD230 and:

        Logs it to a .csv file

        Sends it to a Flask-based dashboard via socket programming

        Publishes it to ThingsBoard using MQTT (QoS 1)

        Feeds it to a lightweight ML model for fault detection

Features

    Real-time data acquisition from engine and battery subsystems

    CAN-based communication between nodes

    Centralized data processing and forwarding via BeagleBone Black

    Flask-based local dashboard with live socket updates

    Remote monitoring on ThingsBoard (MQTT-based)

    CSV data logging for historical analysis

    Fault detection using ML with automatic email alerts

Machine Learning Fault Detection

The system includes a lightweight fault detection model that classifies system conditions into the following faults:

    Engine Overheat

    Overspeed

    High RPM

    Battery Critical

Input Features:
RPM, Speed, Engine Temperature, Battery Voltage, Current, SOC, SOH, Battery Temperature

When a fault is detected, an email notification is sent containing:

    Fault type

    Timestamp

    Relevant parameter values

Cloud and Dashboard Integration

    Local Dashboard: Built using Flask and socket programming for live display

    Cloud Dashboard: Uses ThingsBoard (demo.thingsboard.io) to display live telemetry using MQTT (QoS 1)

    CSV Logging: All data is logged in structured CSV format for offline review

Technologies Used

    Embedded C (STM32 HAL)

    Python (Flask, socket, MQTT, pandas, scikit-learn)

    CAN Protocol

    ThingsBoard (IoT Cloud)

    Email Alerts (SMTP)

    BeagleBone Black (Linux-based controller)

Folder Structure (Suggested)

├── firmware/
│   ├── node1_stm32/
│   ├── node2_stm32/
├── beaglebone/
│   ├── can_receiver.py
│   ├── flask_dashboard/
│   ├── mqtt_client.py
│   ├── ml_model/
│   ├── data_logger.py
├── requirements.txt
├── README.md

Future Improvements

    Integration with GPS module for geolocation tracking

    More advanced machine learning model with additional fault types

    Enhanced dashboard with filtering, fault history, and charting

    OTA firmware updates for STM32 nodes via BeagleBone Black
