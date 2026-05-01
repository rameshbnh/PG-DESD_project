#!/usr/bin/env python3
import can
import time
import joblib
import pandas as pd

# Load trained fault prediction model
model = joblib.load("fault_type_model.joblib")

# Fault labels
labels = ["Engine_Overheat", "Overspeed", "High_RPM", "Battery_Critical"]

def predict_fault(rpm, eng_temp, speed, soc, soh, bat_temp):
    sample = pd.DataFrame([{
        "RPM": rpm,
        "Engine_Temperature_C": eng_temp,
        "Speed_kmph": speed,
        "Battery_SOC_%": soc,
        "Battery_SOH_%": soh,
        "Battery_Temperature_C": bat_temp
    }])
    result = model.predict(sample)[0]
    print("\nðFault Prediction Result:")
    for label, value in zip(labels, result):
        print(f"{label}: {'YES' if value else 'NO'}")
    print("-" * 40)

def main():
    bus = can.Bus(channel='can0', interface='socketcan')
    print(" Listening to CAN bus for fault prediction... (Press Ctrl+C to stop)")

    # Variables to hold the latest data
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
                print(f"Engine Data - RPM: {rpm}, Engine_Temperature_C: {eng_temp}, Speed_kmph: {speed}")

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
                            print(
                                f"Battery Data - SOC: {soc:.2f}%, SOH: {soh:.2f}%, Temp: {bat_temp:.2f}Â°C, "
                                f"Voltage: {voltage:.2f}V, Current: {current:.2f}A"
                            )
                            # Call the ML model
                            predict_fault(rpm, eng_temp, speed, soc, soh, bat_temp)

    except KeyboardInterrupt:
        print("\nstopped by user.")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()
