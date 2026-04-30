import time
import math
import roslibpy


# =========================
# 1. GPS + HEADING STATE
# =========================

class RobotState:
    def __init__(self):
        self.lat = None
        self.lon = None
        self.yaw = 0.0

    def update_gps(self, msg):
        self.lat = msg["latitude"]
        self.lon = msg["longitude"]

    def update_imu(self, msg):
        q = msg["orientation"]
        self.yaw = self.quaternion_to_yaw(
            q["x"], q["y"], q["z"], q["w"]
        )

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


# =========================
# 2. NAVIGATOR
# =========================

class Navigator:
    def __init__(self, ros_client):
        self.cmd_topic = roslibpy.Topic(
            ros_client,
            '/robot/cmd_vel',
            'geometry_msgs/Twist'
        )
        self.cmd_topic.advertise()

    def send_speed(self, linear_x, angular_z):
        msg = roslibpy.Message({
            'linear': {'x': linear_x, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': angular_z}
        })
        self.cmd_topic.publish(msg)

    def stop(self):
        self.send_speed(0.0, 0.0)


# =========================
# 3. GEO NAVIGATION
# =========================

class GeoPlanner:
    def __init__(self, navigator):
        self.navigator = navigator

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000  # metros
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)

        a = math.sin(dphi / 2)**2 + \
            math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2

        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    def bearing(self, lat1, lon1, lat2, lon2):
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlambda = math.radians(lon2 - lon1)

        y = math.sin(dlambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - \
            math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)

        return math.atan2(y, x)

    def move_to_goal(self, current_lat, current_lon, current_yaw, goal_lat, goal_lon):
        dist = self.haversine_distance(current_lat, current_lon, goal_lat, goal_lon)

        if dist < 1.0:
            self.navigator.stop()
            return True

        target_bearing = self.bearing(current_lat, current_lon, goal_lat, goal_lon)
        angle_error = self.normalize_angle(target_bearing - current_yaw)

        linear_speed = min(0.5, 0.2 * dist)
        angular_speed = 1.2 * angle_error

        if abs(angle_error) > 0.3:
            linear_speed = 0.0

        self.navigator.send_speed(linear_speed, angular_speed)

        print(f"[CTRL] dist={dist:.2f}m angle_error={angle_error:.2f}")

        return False


# =========================
# 4. CONTROLLER
# =========================

class RobotController:
    def __init__(self, host):
        self.client = roslibpy.Ros(host=host, port=9090)

        self.state = RobotState()
        self.nav = Navigator(self.client)
        self.planner = GeoPlanner(self.nav)

        self.gps_sub = roslibpy.Topic(
            self.client,
            '/gps/fix',
            'sensor_msgs/NavSatFix'
        )

        self.imu_sub = roslibpy.Topic(
            self.client,
            '/imu/data',
            'sensor_msgs/Imu'
        )

    def connect(self):
        self.client.run()
        time.sleep(2)

        self.gps_sub.subscribe(self.state.update_gps)
        self.imu_sub.subscribe(self.state.update_imu)

    def go_to(self, goal_lat, goal_lon):
        while self.state.lat is None:
            print("Esperando GPS...")
            time.sleep(1)

        while True:
            reached = self.planner.move_to_goal(
                self.state.lat,
                self.state.lon,
                self.state.yaw,
                goal_lat,
                goal_lon
            )

            if reached:
                print("Objetivo alcanzado")
                break

            time.sleep(0.2)