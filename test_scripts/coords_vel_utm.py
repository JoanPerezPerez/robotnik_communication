import time
import math
import roslibpy
from pyproj import Proj


# =========================
# 1. GPS → MAP CONVERTER
# =========================

class GPSConverter:
    def __init__(self, utm_zone=31):
        self.utm = Proj(proj='utm', zone=utm_zone, ellps='WGS84')

    def latlon_to_xy(self, lat, lon):
        x, y = self.utm(lon, lat)
        return x, y


# =========================
# 2. ROBOT STATE (ODOMETRÍA)
# =========================

class RobotState:
    def __init__(self):
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def update(self, msg):
        self.pose["x"] = msg["pose"]["pose"]["position"]["x"]
        self.pose["y"] = msg["pose"]["pose"]["position"]["y"]

        q = msg["pose"]["pose"]["orientation"]
        self.pose["yaw"] = self.quaternion_to_yaw(
            q["x"], q["y"], q["z"], q["w"]
        )

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


# =========================
# 3. NAVIGATOR
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
        print("[NAV] Robot detenido")


# =========================
# 4. TRAJECTORY PLANNER
# =========================

class TrajectoryPlanner:
    def __init__(self, navigator):
        self.navigator = navigator

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def move_to_goal(self, current_pose, goal_pose):
        dx = goal_pose["x"] - current_pose["x"]
        dy = goal_pose["y"] - current_pose["y"]

        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.5:
            self.navigator.stop()
            return True

        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - current_pose["yaw"])

        # Control proporcional
        linear_speed = min(0.5, 0.3 * distance)
        angular_speed = 1.5 * angle_error

        # Si está muy desalineado, gira antes de avanzar
        if abs(angle_error) > 0.3:
            linear_speed = 0.0

        self.navigator.send_speed(linear_speed, angular_speed)

        print(f"[CTRL] dist={distance:.2f} angle_error={angle_error:.2f}")
        return False


# =========================
# 5. ROBOT CONTROLLER
# =========================

class RobotController:
    def __init__(self, host, port=9090):
        self.client = roslibpy.Ros(host=host, port=port)

        self.state = RobotState()
        self.nav = Navigator(self.client)
        self.gps = GPSConverter()
        self.planner = TrajectoryPlanner(self.nav)

        self.odom_sub = roslibpy.Topic(
            self.client,
            '/robot/odom',
            'nav_msgs/Odometry'
        )

    def connect(self):
        print("[ROS] Conectando...")
        self.client.run()
        time.sleep(2)

        if not self.client.is_connected:
            raise RuntimeError("No se pudo conectar a ROS bridge")

        self.odom_sub.subscribe(self.state.update)

        print("[ROS] Conectado")

    def send_gps_goal(self, lat, lon):
        goal_x, goal_y = self.gps.latlon_to_xy(lat, lon)

        print(f"[GPS] Meta UTM -> {goal_x:.2f}, {goal_y:.2f}")

        goal = {"x": goal_x, "y": goal_y}

        while True:
            reached = self.planner.move_to_goal(self.state.pose, goal)

            if reached:
                print("[MISSION] Objetivo alcanzado")
                break

            time.sleep(0.1)

    def shutdown(self):
        self.nav.stop()
        self.client.terminate()


# =========================
# 6. MAIN
# =========================

if __name__ == "__main__":

    robot = RobotController(host="100.99.163.44")

    robot.connect()

    # Coordenadas objetivo
    LAT = 41.275929
    LON = 1.987814

    robot.send_gps_goal(LAT, LON)

    robot.shutdown()