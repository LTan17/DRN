import paho.mqtt.client as mqtt
import keyboard
import time
import json

BROKER = "127.0.0.1"
PORT = 1883
TOPIC = "lt17/drn/cmd"

MAX_ANGLE = 15.0     
DEFAULT_ROLL = 0.0   
DEFAULT_PITCH = 0.0  
THROTTLE_STEP = 5.0  
MAX_THROTTLE = 1000.0

throttle = 0.0

client = mqtt.Client()
client.connect(BROKER, PORT)
client.loop_start()

print("--- MQTT CONTROL STARTED ---")

try:
    while True:
        roll_target = DEFAULT_ROLL
        pitch_target = DEFAULT_PITCH

        if keyboard.is_pressed('w'): pitch_target = MAX_ANGLE
        elif keyboard.is_pressed('s'): pitch_target = -MAX_ANGLE

        if keyboard.is_pressed('a'): roll_target = -MAX_ANGLE
        elif keyboard.is_pressed('d'): roll_target = MAX_ANGLE

        if keyboard.is_pressed('+') or keyboard.is_pressed('='):
            throttle = min(MAX_THROTTLE, throttle + THROTTLE_STEP)
        elif keyboard.is_pressed('-'):
            throttle = max(0.0, throttle - THROTTLE_STEP)

        if keyboard.is_pressed('space'):
            throttle = 0.0

        if keyboard.is_pressed('esc'):
            break

        payload = {
            "t": int(throttle),
            "r": float(roll_target),
            "p": float(pitch_target)
        }
        client.publish(TOPIC, json.dumps(payload), qos=0)
        print(f"[MQTT GỬI] {payload}")
        time.sleep(0.05)

except KeyboardInterrupt:
    pass
finally:
    client.publish(TOPIC, json.dumps({"t": 0, "r": 0, "p": 0}), qos=1)
    client.loop_stop()
    client.disconnect()
    print("Stopped.")