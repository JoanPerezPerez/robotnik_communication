import time
import math
import roslibpy
from pyproj import Proj


# =========================
# 1. GPS → UTM CONVERTER
# =========================

class GPSConverter:
    def __init__(self, utm_zone=31):
        self.utm = Proj(proj='utm', zone=utm_zone, ellps='WGS84')

    def latlon_to_xy(self, lat, lon):
        x, y = self.utm(lon, lat)
        return x, y


# =========================
# 2. ROBOT STATE
# =========================

class RobotState:
    def __init__(self):
        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.odom_origin = None

    def update(self, msg):
        #print(msg)
        x = msg["pose"]["pose"]["position"]["x"]
        y = msg["pose"]["pose"]["position"]["y"]

        if self.odom_origin is None:
            self.odom_origin = (x, y)
            print(f"[ODOM] Origen fijado -> {self.odom_origin}")

        # Posición relativa al origen de odometría
        self.pose["x"] = x - self.odom_origin[0]
        self.pose["y"] = y - self.odom_origin[1]

        q = msg["pose"]["pose"]["orientation"]
        self.pose["yaw"] = self._quaternion_to_yaw(q["x"], q["y"], q["z"], q["w"])

    def _quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


# =========================
# 3. GPS STATE
# =========================

class GPSState:
    """
    Suscribe a /robot/gps/fix y captura la posición UTM del robot
    en el momento de arranque, para usarla como origen del frame local.
    """
    def __init__(self, gps_converter):
        self.gps = gps_converter
        self.origin_utm = None   # (x, y) UTM del robot al arrancar

    def update(self, msg):
        lat = msg["latitude"]
        lon = msg["longitude"]
        x, y = self.gps.latlon_to_xy(lat, lon)

        if self.origin_utm is None:
            self.origin_utm = (x, y)
            print(f"[GPS] Origen UTM fijado -> ({x:.2f}, {y:.2f})")

    def goal_to_local(self, goal_utm_x, goal_utm_y):
        """
        Transforma coordenadas UTM absolutas del objetivo
        al frame local del robot (metros desde el origen GPS).
        """
        if self.origin_utm is None:
            raise RuntimeError("GPS origin no disponible aún")
        return {
            "x": goal_utm_x - self.origin_utm[0],
            "y": goal_utm_y - self.origin_utm[1]
        }


# =========================
# 4. NAVIGATOR
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
            'linear':  {'x': linear_x, 'y': 0.0, 'z': 0.0},
            'angular': {'x': 0.0,      'y': 0.0, 'z': angular_z}
        })
        self.cmd_topic.publish(msg)

    def stop(self):
        self.send_speed(0.0, 0.0)
        print("[NAV] Robot detenido")


# =========================
# 5. TRAJECTORY PLANNER
# =========================

class TrajectoryPlanner:
    """
    Lazo cerrado de dos fases:
      Fase 1 - GIRO:    alinear heading al objetivo antes de avanzar.
      Fase 2 - AVANCE:  avanzar con corrección angular continua.
    Detiene el robot cuando distance < goal_tolerance (metros).
    """

    # Parámetros de control
    GOAL_TOLERANCE   = 1.0    # metros — criterio de parada
    ANGLE_THRESHOLD  = 0.15   # rad (~8.5°) — umbral para pasar a fase avance

    # Ganancias
    KP_ANGULAR       = 1.2
    KP_LINEAR        = 0.15

    # Límites de velocidad
    MAX_LINEAR       = 0.5    # m/s
    MIN_LINEAR       = 0.05   # m/s — evita pararse en distancias cortas
    MAX_ANGULAR      = 0.6    # rad/s

    def __init__(self, navigator):
        self.navigator = navigator

    def normalize_angle(self, angle):
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def move_to_goal(self, current_pose, goal_pose):
        """
        Llamar en cada tick del lazo de control.
        Devuelve True cuando el objetivo ha sido alcanzado.
        """
        dx = goal_pose["x"] - current_pose["x"]
        dy = goal_pose["y"] - current_pose["y"]
        distance    = math.sqrt(dx**2 + dy**2)
        target_yaw  = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_yaw - current_pose["yaw"])

        print(f"[CTRL] dist={distance:.2f}m  angle_err={math.degrees(angle_error):.1f}°  "
              f"pos=({current_pose['x']:.2f}, {current_pose['y']:.2f})  "
              f"goal=({goal_pose['x']:.2f}, {goal_pose['y']:.2f})")

        # ── Criterio de llegada ──────────────────────────────────────────
        if distance < self.GOAL_TOLERANCE:
            self.navigator.stop()
            return True

        # ── Fase 1: GIRO puro ────────────────────────────────────────────
        if abs(angle_error) > self.ANGLE_THRESHOLD:
            angular_z = self.KP_ANGULAR * angle_error
            angular_z = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, angular_z))
            self.navigator.send_speed(0.0, angular_z)                                     
            return False

        # ── Fase 2: AVANCE con corrección angular continua ───────────────
        linear_x  = self.KP_LINEAR * distance
        linear_x  = max(self.MIN_LINEAR, min(self.MAX_LINEAR, linear_x))

        # Reducir velocidad lineal si hay error angular residual
        linear_x *= max(0.3, 1.0 - abs(angle_error))

        angular_z = self.KP_ANGULAR * angle_error
        angular_z = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, angular_z))

        self.navigator.send_speed(linear_x, angular_z)
        return False


# =========================
# 6. ROBOT CONTROLLER
# =========================

class RobotController:
    def __init__(self, host, port=9090):
        self.client  = roslibpy.Ros(host=host, port=port)
        self.gps_conv = GPSConverter()

        self.state   = RobotState()
        self.gps_st  = GPSState(self.gps_conv)
        self.nav     = Navigator(self.client)
        self.planner = TrajectoryPlanner(self.nav)

        self.odom_sub = roslibpy.Topic(
            self.client,
            '/robot/odometry/filtered_world',
            'nav_msgs/Odometry'
        )
        self.gps_sub = roslibpy.Topic(
            self.client,
            '/robot/gps/fix',
            'sensor_msgs/NavSatFix'
        )

    def connect(self):
        print("[ROS] Conectando...")
        self.client.run()
        time.sleep(2)

        if not self.client.is_connected:
            raise RuntimeError("No se pudo conectar a ROS bridge")

        self.odom_sub.subscribe(self.state.update)
        self.gps_sub.subscribe(self.gps_st.update)
        print("[ROS] Conectado y suscrito")

    def _wait_for_origins(self):
        """Bloquea hasta tener el primer mensaje de odom y de GPS."""
        print("[WAIT] Esperando origen GPS y odometría...")
        while self.gps_st.origin_utm is None or self.state.odom_origin is None:
            time.sleep(0.1)
        print("[WAIT] Orígenes listos")

    def send_gps_goal(self, lat, lon):
        # 1. Esperar orígenes
        self._wait_for_origins()

        # 2. Convertir objetivo GPS → UTM → frame local
        goal_utm_x, goal_utm_y = self.gps_conv.latlon_to_xy(lat, lon)
        print(f"[GPS] Meta UTM global -> ({goal_utm_x:.2f}, {goal_utm_y:.2f})")

        goal_local = self.gps_st.goal_to_local(goal_utm_x, goal_utm_y)
        print(f"[GPS] Meta en frame local -> ({goal_local['x']:.2f}, {goal_local['y']:.2f})")

        # 3. Lazo cerrado de control
        print("[MISSION] Iniciando navegación...")
        while True:
            reached = self.planner.move_to_goal(self.state.pose, goal_local)
            if reached:
                print("[MISSION] Objetivo alcanzado ✓")
                break
            time.sleep(0.1)

    def shutdown(self):
        self.nav.stop()
        self.client.terminate()
        print("[ROS] Desconectado")


# =========================
# 7. MAIN
# =========================

if __name__ == "__main__":

    robot = RobotController(host="100.99.163.44")

    try:
        robot.connect()

        LAT = 41.275929
        LON = 1.987814

        robot.send_gps_goal(LAT, LON)

    finally:
        robot.shutdown()