import json
import paho.mqtt.client as mqtt
from paho.mqtt.client import CallbackAPIVersion
import time
#from coords import coords_to_ros
from coords_vel_utm import RobotController

BROKER_HOST = "127.0.0.1"                     
BROKER_PORT = 8008
TOPIC_GPS = "drone/telemetry/gps" 
robot = RobotController(host="100.99.163.44")
goal_queue = queue.Queue()
running = True

def on_connect(client, userdata, flags, rc, properties):
    print("Connected with result code", rc)
    client.subscribe(TOPIC_GPS)

def navigation_worker():
    """
    Hilo dedicado exclusivamente a consumir waypoints.
    """

    robot.connect()

    try:
        while running:

            lat, lon = goal_queue.get()

            print(f"[QUEUE] Nuevo objetivo: {lat}, {lon}")

            try:
                robot.send_gps_goal(lat, lon)

            except Exception as e:
                print("[NAV ERROR]", e)

            finally:
                goal_queue.task_done()

    finally:
        robot.shutdown()
    
def on_message(client, userdata, msg):
    print("MSG:", msg.topic, msg.payload.decode())
    try:
        json_msg = json.loads(msg.payload.decode())
        value = json_msg.get("value", {})
        latitude = value.get("lat")
        longitude = value.get("lon")
        detection = value.get("detection")
        timestamp = json_msg.get("timestamp")
        if detection: 
            print(f"[MQTT] Detection -> {latitude}, {longitude}")

            goal_queue.put((latitude, longitude))

            print(f"[QUEUE] Tamaño actual: {goal_queue.qsize()}")
        
    except json.JSONDecodeError as e:
        print("Error al parsear JSON:", e)
        return


nav_thread = threading.Thread(target=navigation_worker, daemon=True)
nav_thread.start()

client = mqtt.Client(CallbackAPIVersion.VERSION2, client_id="mavlink_sub_gpos")
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER_HOST, BROKER_PORT, keepalive=30)


print("[MQTT→ROS] Bridge activo")
client.loop_forever()