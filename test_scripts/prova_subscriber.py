# Cliente receptor MQTT que recibe datos de telemetría GPS publicados por el cliente MAVLink
import paho.mqtt.client as mqtt
from paho.mqtt.client import CallbackAPIVersion

BROKER_HOST = "127.0.0.1"                     # broker local [web:26]
BROKER_PORT = 8008
TOPIC_GPS = "drone/telemetry/gps"             # mismo topic del publicador

def on_connect(client, userdata, flags, rc, properties):
    print("Connected:", rc)
    client.subscribe(TOPIC_GPS)               # suscribirse [web:31]

def on_message(client, userdata, msg):
    print("MSG:", msg.topic, msg.payload.decode())

cli = mqtt.Client(CallbackAPIVersion.VERSION2, client_id="mavlink_sub_gpos") 
cli.on_connect = on_connect
cli.on_message = on_message
cli.connect(BROKER_HOST, BROKER_PORT, keepalive=30)  # conectar
cli.loop_forever()                                   # mantener red activa 
