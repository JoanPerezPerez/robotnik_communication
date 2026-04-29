import json
import paho.mqtt.client as mqtt
import time
from robot_controller import RobotController  # tu clase ROS

BROKER = "127.0.0.1"
TOPIC = "drone/telemetry/gps"

# Coordenada objetivo (ejemplo)
TARGET_LAT = 41.275943
TARGET_LON = 1.987887


robot = RobotController(host="100.99.163.44")
robot.connect()

last_trigger = 0
COOLDOWN = 10  # segundos para evitar spam


def on_message(client, userdata, msg):
    global last_trigger

    data = json.loads(msg.payload.decode())

    value = data.get("value", {})
    detection = value.get("detection", False)

    if detection:
        now = time.time()

        if now - last_trigger > COOLDOWN:
            print("[ALERT] Detección recibida → enviando goal")

            robot.send_gps_goal(TARGET_LAT, TARGET_LON)

            last_trigger = now
        else:
            print("[INFO] Detección ignorada (cooldown)")


client = mqtt.Client()
client.on_message = on_message

client.connect(BROKER, 1883, 60)
client.subscribe(TOPIC)

print("[MQTT→ROS] Bridge activo")
client.loop_forever()