import roslibpy
import time

client = roslibpy.Ros(host='100.99.163.44', port=9090)
client.run()

LAT = 41.275929
LON = 1.987814

time.sleep(2)

print("Conectado?", client.is_connected)

if not client.is_connected:
    print("No se pudo conectar")
    exit()
    
service = roslibpy.Service(
    client,
    '/robot/robot_local_control/NavigationComponent/GoToGPSComponent/add',
    'robot_local_control_msgs/GoToGPSPetition'
)

request = roslibpy.ServiceRequest({
    'procedure': {
        'goals': [
            {
                'latitude': LAT,
                'longitude': LON
            }
        ],
        'max_velocity': 0.5
    }
})

result = service.call(request)

print(request)
print(result)

client.terminate()