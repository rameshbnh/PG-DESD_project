#!/usr/bin/env python3
import can
import time
import json
import paho.mqtt.client as mqtt

# Configuration
THINGSBOARD_HOST = 'demo.thingsboard.io'
ACCESS_TOKEN = 'ZKl4uk1PUdIm8d3kYugW'
PORT = 1883
TOPIC = 'v1/devices/me/telemetry'
CAN_CHANNEL = 'can0'

def setup_mqtt_client():
    # Use the newer MQTT client API version
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set(ACCESS_TOKEN)
    
    def on_connect(client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print("Connected successfully to ThingsBoard")
        else:
            print(f"Connection failed with reason code: {reason_code}")
    
    def on_publish(client, userdata, mid, reason_code, properties):
        print(f"Message published successfully, mid: {mid}")
    
    def on_disconnect(client, userdata, disconnect_flags, reason_code, properties):
        print(f"Disconnected with reason code: {reason_code}")
        if reason_code != 0:
            print("Attempting to reconnect...")
            client.reconnect()
    
    client.on_connect = on_connect
    client.on_publish = on_publish
    client.on_disconnect = on_disconnect
    
    try:
        client.connect(THINGSBOARD_HOST, PORT, 60)
        client.loop_start()
        return client
    except Exception as e:
        print(f"Initial connection failed: {str(e)}")
        exit(1)

def process_engine_data(data):
    try:
        rpm = (data[0] << 8) | data[1]
        speed = (data[2] << 8) | data[3]
        temp_engine = (data[4] << 8) | data[5]
        
        return {
            "ts": int(time.time() * 1000),
            "values": {
                "Engine_RPM": rpm,
                "Vehicle_Speed_kmph": speed,
                "Engine_Temperature_C": temp_engine
            }
        }
    except IndexError:
        print("Invalid engine data format")
        return None

def process_battery_data(data, battery_state):
    try:
        if data[6] != 0xAA:
            return None
        
        if data[7] == 0x01:
            battery_state['soc'] = ((data[0] << 8) | data[1]) / 100.0
            battery_state['soh'] = ((data[2] << 8) | data[3]) / 100.0
            battery_state['temp'] = ((data[4] << 8) | data[5]) / 100.0
            return None
        
        elif data[7] == 0x02:
            battery_state['voltage'] = ((data[0] << 8) | data[1]) / 100.0
            current_raw = (data[2] << 8) | data[3]
            battery_state['current'] = (current_raw if current_raw < 32768 else current_raw - 65536) / 1000.0
            
            if all(v is not None for v in battery_state.values()):
                return {
                    "ts": int(time.time() * 1000),
                    "values": {
                        "Battery_SOC_%": battery_state['soc'],
                        "Battery_SOH_%": battery_state['soh'],
                        "Battery_Temperature_C": battery_state['temp'],
                        "Battery_Voltage_V": battery_state['voltage'],
                        "Battery_Current_A": battery_state['current']
                    }
                }
    except IndexError:
        print("Invalid battery data format")
    
    return None

def main():
    client = setup_mqtt_client()
    
    try:
        bus = can.Bus(channel=CAN_CHANNEL, interface='socketcan')
        print(f"Reading CAN data from {CAN_CHANNEL} and sending to ThingsBoard (Ctrl+C to stop)")
        
        battery_state = {
            'soc': None,
            'soh': None,
            'temp': None,
            'voltage': None,
            'current': None
        }
        
        while True:
            try:
                message = bus.recv(timeout=1.0)
                if message is None:
                    continue
                
                payload = None
                
                if message.arbitration_id == 0x123 and message.dlc == 8:
                    payload = process_engine_data(message.data)
                
                elif message.arbitration_id == 0x124 and message.dlc == 8:
                    payload = process_battery_data(message.data, battery_state)
                
                if payload:
                    print(f"Sending data: {json.dumps(payload)}")
                    result = client.publish(TOPIC, json.dumps(payload), qos=1)
                    if result.rc != mqtt.MQTT_ERR_SUCCESS:
                        print(f"Publish failed with error code: {result.rc}")
                
                time.sleep(0.1)
            
            except can.CanError as e:
                print(f"CAN bus error: {str(e)}")
                time.sleep(1)
    
    except KeyboardInterrupt:
        print("\nStopping CAN receiver")
    except Exception as e:
        print(f"Unexpected error: {str(e)}")
    finally:
        client.loop_stop()
        client.disconnect()
        try:
            bus.shutdown()
        except:
            pass

if __name__ == "__main__":
    main()
