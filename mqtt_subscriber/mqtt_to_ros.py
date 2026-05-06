import json
import paho.mqtt.client as mqtt
from paho.mqtt.client import CallbackAPIVersion
import time
#from coords import coords_to_ros

BROKER_HOST = "127.0.0.1"                     
BROKER_PORT = 8008
TOPIC_GPS = "drone/telemetry/gps" 

def on_connect(client, userdata, flags, rc, properties):
    print("Connected with result code", rc)
    client.subscribe(TOPIC_GPS)
    
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
            print(latitude, longitude, timestamp) 
            #coords_to_ros(latitude, longitude)
        
    except json.JSONDecodeError as e:
        print("Error al parsear JSON:", e)
        return


client = mqtt.Client(CallbackAPIVersion.VERSION2, client_id="mavlink_sub_gpos")
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
client.subscribe(TOPIC_GPS)

print("[MQTT→ROS] Bridge activo")
client.loop_forever()