#!/usr/bin/env python3
import can
import time
import joblib
import pandas as pd
import smtplib
from email.message import EmailMessage
import json
import socket
import numpy as np

# Load trained fault prediction model
model = joblib.load("fault_type_model.joblib")

# Fault labels
FAULT_LABELS = ["Engine_Overheat", "Overspeed", "High_RPM", "Battery_Critical"]

# Email configuration
EMAIL_SENDER = "cdaclabramesh@gmail.com"
EMAIL_PASSWORD = "towt jmgn mqei ewjc"
EMAIL_RECEIVER = "rameshbnh36@gmail.com"

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

def send_fault_email(faults, sample):
    """Send email notification when faults are detected"""
    msg = EmailMessage()
    msg['Subject'] = 'Vehicle Fault Detected - Immediate Attention Required'
    msg['From'] = EMAIL_SENDER
    msg['To'] = EMAIL_RECEIVER

    fault_list = [label for label, flag in zip(FAULT_LABELS, faults) if flag]
    fault_text = '\n'.join(fault_list)

    body = f"""
Fault(s) Detected:

{fault_text}

Sensor Snapshot:
{sample.to_string(index=False)}

Time: {time.strftime('%Y-%m-%d %H:%M:%S')}
"""

    msg.set_content(body)

    try:
        with smtplib.SMTP_SSL('smtp.gmail.com', 465) as smtp:
            smtp.login(EMAIL_SENDER, EMAIL_PASSWORD)
            smtp.send_message(msg)
            print("Email alert sent successfully")
    except Exception as e:
        print(f"Failed to send email notification: {str(e)}")

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

    # Only trigger alerts for new faults
    if any((new and not old) for new, old in zip(result, last_fault_state)):
        send_fault_email(result, sample)

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
