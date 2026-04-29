import json
import time
import socket
import struct
import base64
import cv2
import numpy as np
import threading
from pymavlink import mavutil
import paho.mqtt.client as mqtt
from paho.mqtt.client import CallbackAPIVersion

# --- CONFIGURACIÓN ---
BROKER_HOST = "127.0.0.1"
BROKER_PORT = 8008
TOPIC_GPS = "drone/telemetry/gps"
STATUS_TOPIC = "drone/telemetry/status"
IMAGE_PORT = 5010  # El puerto TCP donde envía la Raspberry

# Variable global para guardar la última imagen recibida
latest_base64_image = None
is_running = True

def frame_to_base64(frame):
    """Convierte el frame de OpenCV a la cadena Base64 que pide tu JSON"""
    # Comprimimos un poco más para que el mensaje MQTT no sea gigante
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
    result, buffer = cv2.imencode('.jpg', frame, encode_param)
    if result:
        img_str = base64.b64encode(buffer).decode('utf-8')
        return f"data:image/jpeg;base64,{img_str}"
    return None

def recvall(sock, n):
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet: return None
        data.extend(packet)
    return data

def image_receiver_thread():
    """Hilo que escucha la Raspberry y actualiza la variable global de imagen"""
    global latest_base64_image, is_running
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("0.0.0.0", IMAGE_PORT))
    server.listen(1)
    
    while is_running:
        try:
            conn, addr = server.accept()
            while is_running:
                frame = None
                length_bytes = recvall(conn, 4)
                if not length_bytes: break
                img_len = struct.unpack(">L", length_bytes)[0]
                img_data = recvall(conn, img_len)
                if not img_data: break
                
                np_data = np.frombuffer(img_data, dtype=np.uint8)
                frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
                if frame is not None:
                    latest_base64_image = frame_to_base64(frame)
                else: 
                    latest_base64_image = None
        except socket.timeout:
            continue
        except Exception as e:
            print(f"Error en receptor de imagen: {e}")
            time.sleep(1)
            

def decode_gpos(gpos_msg):
    """Mantiene tu estructura original pero prepara el diccionario value"""
    return {
        "lat": gpos_msg.lat/1e7,
        "lon": gpos_msg.lon/1e7,
        "alt_msl": gpos_msg.alt/1000.0,
        "relative_alt": gpos_msg.relative_alt/1000.0,
        "vx": gpos_msg.vx/100.0,
        "vy": gpos_msg.vy/100.0,
        "vz": gpos_msg.vz/100.0,
        "hdg_deg": (gpos_msg.hdg/100.0 if getattr(gpos_msg, "hdg", 65535) != 65535 else 0),
        "t_s": int(time.time())
    }

def fake_detection():
    """Función de ejemplo que simula detecciones aleatorias"""
    return np.random.rand() < 0.1  # 10% de probabilidad de detección


def main(): 
    global latest_base64_image
    
    # 1. Iniciar hilo de imágenes
    threading.Thread(target=image_receiver_thread, daemon=True).start()

    # 2. Conexión MAVLink
    simulator = mavutil.mavlink_connection('udp:0.0.0.0:14560')
    print("Esperando heartbeat...")
    simulator.wait_heartbeat()
    
    # 3. Configuración MQTT
    sensor = mqtt.Client(CallbackAPIVersion.VERSION2, client_id="drone_full_publisher")
    sensor.will_set(STATUS_TOPIC, "offline", retain=True)
    
    try:
        sensor.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
        sensor.loop_start()
        sensor.publish(STATUS_TOPIC, "online", retain=True)
    except Exception as e:
        print(f"Error MQTT: {e}")
        return

    print("--- PUBLICADOR ACTIVO: GPS + IMAGEN ---")

    try:
        while True:
            # Esperamos datos del dron
            glob_pos = simulator.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
            
            if glob_pos:
                # Extraemos telemetría
                telemetry_data = decode_gpos(glob_pos)
                current_image = latest_base64_image
                latest_base64_image = None
                detection = fake_detection()
                # Construimos el JSON con el formato exacto que pediste
                full_payload = {
                    "value": {
                        **telemetry_data,
                        "radar_image": current_image if current_image else "",  
                        "detection": detection
                    },
                    "timestamp": int(time.time())
                }
                print (full_payload)
                # Publicamos
                sensor.publish(TOPIC_GPS, json.dumps(full_payload), qos=0)
                print(f"[{time.strftime('%H:%M:%S')}] Publicado GPS + Imagen (Base64)")
                
                # El intervalo lo manejamos con el sueño o por frecuencia de mensajes
                time.sleep(1) 

    except KeyboardInterrupt:
        print("Cerrando...")
    finally:
        sensor.publish(STATUS_TOPIC, "offline", retain=True)
        sensor.loop_stop()
        sensor.disconnect()

if __name__ == "__main__":
    main()