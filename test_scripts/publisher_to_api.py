import roslibpy
import time

client = roslibpy.Ros(host='100.99.163.44', port=9090)

print("Conectando...")
client.run()

time.sleep(3)

print("Conectado?", client.is_connected)

if not client.is_connected:
    print("No se pudo conectar")
    exit()

cmd_vel = roslibpy.Topic(
    client,
    '/robot/cmd_vel',
    'geometry_msgs/Twist'
)

cmd_vel.advertise()

msg = roslibpy.Message({
    'linear': {'x': 0.3, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
})

for _ in range(150):
    cmd_vel.publish(msg)
    print("Publicado")
    time.sleep(0.1)

stop = roslibpy.Message({
    'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
})

cmd_vel.publish(stop)

cmd_vel.unadvertise()
client.terminate()