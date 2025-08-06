import can
import threading
import time
import csv
import os

# Shared data for dashboard
latest_data = {
    "RPM": 0,
    "Speed": 0,
    "Engine_Temp": 0,
    "SOC": None,
    "SOH": None,
    "Battery_Temp": None,
    "Voltage": None,
    "Current": None
}

log_file = "logs/can_log.csv"
os.makedirs("logs", exist_ok=True)

def log_to_csv(data):
    headers = list(data.keys())
    file_exists = os.path.isfile(log_file)

    with open(log_file, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        if not file_exists:
            writer.writeheader()
        writer.writerow(data)

def read_can():
    bus = can.Bus(channel='can0', interface='socketcan')
    soc = soh = batt_temp = voltage = current = None

    try:
        while True:
            message = bus.recv(timeout=1.0)
            if message is None:
                continue

            data = message.data
            if message.arbitration_id == 0x123 and message.dlc == 8:
                rpm = (data[0] << 8) | data[1]
                speed = (data[2] << 8) | data[3]
                engine_temp = (data[4] << 8) | data[5]

                latest_data["RPM"] = rpm
                latest_data["Speed"] = speed
                latest_data["Engine_Temp"] = engine_temp

            elif message.arbitration_id == 0x124 and message.dlc == 8:
                if data[6] == 0xAA:
                    if data[7] == 0x01:
                        soc = ((data[0] << 8) | data[1]) / 100.0
                        soh = ((data[2] << 8) | data[3]) / 100.0
                        batt_temp = ((data[4] << 8) | data[5]) / 100.0

                        latest_data["SOC"] = soc
                        latest_data["SOH"] = soh
                        latest_data["Battery_Temp"] = batt_temp

                    elif data[7] == 0x02:
                        voltage = ((data[0] << 8) | data[1]) / 100.0
                        current_raw = (data[2] << 8) | data[3]
                        current = (current_raw if current_raw < 32768 else current_raw - 65536) / 1000.0

                        latest_data["Voltage"] = voltage
                        latest_data["Current"] = current

                        if None not in (soc, soh, batt_temp, voltage, current):
                            log_to_csv(latest_data.copy())
    except KeyboardInterrupt:
        print("Stopping CAN reader.")
    finally:
        bus.shutdown()

def start_can_reader():
    thread = threading.Thread(target=read_can, daemon=True)
    thread.start()

def get_latest_data():
    return latest_data
