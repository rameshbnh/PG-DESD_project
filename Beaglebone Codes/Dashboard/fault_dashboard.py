#!/usr/bin/env python3
import can
import time
import joblib
import pandas as pd
import json
import socket
import numpy as np
import requests  # Added for ThingsBoard HTTP API

# Load trained fault prediction model
model = joblib.load("fault_type_model.joblib")

# Fault labels
FAULT_LABELS = ["Engine_Overheat", "Overspeed", "High_RPM", "Battery_Critical"]

# ThingsBoard configuration
THINGSBOARD_URL = "https://demo.thingsboard.io"  # or your ThingsBoard instance
ACCESS_TOKEN = "1Ghpp3xs8xvSKfFq1XAI"  # Replace with your device token

# Track last fault state
last_fault_state = [0, 0, 0, 0]

def convert_numpy_types(data):
    """Convert numpy types to native Python types for JSON serialization"""
    if isinstance(data, np.integer):
        return int(data)
    elif isinstance(data, np.floating):
        return float(data)
    elif isinstance(data, np.ndarray):
        return data.tolist()
    elif isinstance(data, dict):
        return {key: convert_numpy_types(value) for key, value in data.items()}
    elif isinstance(data, (list, tuple)):
        return [convert_numpy_types(item) for item in data]
    return data

def send_to_thingsboard(telemetry_data):
    """Send data to ThingsBoard cloud"""
    url = f"{THINGSBOARD_URL}/api/v1/{ACCESS_TOKEN}/telemetry"
    headers = {'Content-Type': 'application/json'}
    
    try:
        response = requests.post(url, data=json.dumps(telemetry_data), headers=headers)
        response.raise_for_status()
        print("Data sent to ThingsBoard successfully")
    except requests.exceptions.RequestException as e:
        print(f"Failed to send data to ThingsBoard: {str(e)}")

def predict_fault(rpm, eng_temp, speed, soc, soh, bat_temp):
    """Predict faults based on current sensor readings"""
    global last_fault_state

    sample = pd.DataFrame([{
        "RPM": rpm,
        "Engine_Temperature_C": eng_temp,
        "Speed_kmph": speed,
        "Battery_SOC_%": soc,
        "Battery_SOH_%": soh,
        "Battery_Temperature_C": bat_temp
    }])

    result = model.predict(sample)[0]
    
    print("\nFault Prediction Results:")
    for label, value in zip(FAULT_LABELS, result):
        print(f"{label}: {'DETECTED' if value else 'Normal'}")
    print("-" * 40)

    # Prepare telemetry data for ThingsBoard
    telemetry_data = {
        "ts": int(time.time() * 1000),  # Timestamp in milliseconds
        "values": {
            "RPM": rpm,
            "Engine_Temperature_C": eng_temp,
            "Speed_kmph": speed,
            "Battery_SOC": soc,
            "Battery_SOH": soh,
            "Battery_Temperature_C": bat_temp,
            "Fault_Engine_Overheat": int(result[0]),
            "Fault_Overspeed": int(result[1]),
            "Fault_High_RPM": int(result[2]),
            "Fault_Battery_Critical": int(result[3])
        }
    }

    # Only send alert for new faults
    if any((new and not old) for new, old in zip(result, last_fault_state)):
        # Add fault details to telemetry
        fault_list = [label for label, flag in zip(FAULT_LABELS, result) if flag]
        telemetry_data["values"]["Active_Faults"] = ", ".join(fault_list) if fault_list else "None"
        print(f"New faults detected: {telemetry_data['values']['Active_Faults']}")
    
    # Send all data to ThingsBoard
    send_to_thingsboard(telemetry_data)
    last_fault_state = result

def main():
    """Main function to monitor CAN bus and detect faults"""
    bus = can.Bus(channel='can0', interface='socketcan')
    print("Starting CAN bus monitoring for fault detection...")
    print("Press Ctrl+C to stop the monitoring")

    # Initialize sensor variables
    rpm = speed = eng_temp = None
    soc = soh = bat_temp = voltage = current = None

    try:
        while True:
            message = bus.recv(timeout=1.0)
            if message is None:
                continue

            if message.arbitration_id == 0x123 and message.dlc == 8:
                data = message.data
                rpm = (data[0] << 8) | data[1]
                speed = (data[2] << 8) | data[3]
                eng_temp = (data[4] << 8) | data[5]
                print(f"Engine Data - RPM: {rpm}, Temp: {eng_temp}C, Speed: {speed}km/h")

            elif message.arbitration_id == 0x124 and message.dlc == 8:
                data = message.data
                if data[6] == 0xAA:
                    if data[7] == 0x01:
                        soc = ((data[0] << 8) | data[1]) / 100.0
                        soh = ((data[2] << 8) | data[3]) / 100.0
                        bat_temp = ((data[4] << 8) | data[5]) / 100.0
                    elif data[7] == 0x02:
                        voltage = ((data[0] << 8) | data[1]) / 100.0
                        current_raw = (data[2] << 8) | data[3]
                        current = (current_raw if current_raw < 32768 else current_raw - 65536) / 1000.0

                        if None not in (rpm, eng_temp, speed, soc, soh, bat_temp):
                            print(f"Battery Data - SOC: {soc:.1f}%, SOH: {soh:.1f}%, Temp: {bat_temp:.1f}C")
                            predict_fault(rpm, eng_temp, speed, soc, soh, bat_temp)

    except KeyboardInterrupt:
        print("\nMonitoring stopped by user")
    finally:
        bus.shutdown()
        print("CAN bus connection closed")

if __name__ == "__main__":
    main()
