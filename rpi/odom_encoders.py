#!/usr/bin/env python3
"""Odometría SOLO con encoders, para comparar con /odom (que gira con el giroscopio).

Lee /ruedas ("fl rl fr rr yaw gz", cuentas acumuladas que publica teleop_ros.py) y publica /odom_enc
(nav_msgs/Odometry, frame odom). Arranca en la pose del primer /odom para que las dos flechas partan juntas.
En un giro en el lugar las ruedas de un skid-steer patinan: la flecha de /odom_enc se pasa o se queda corta.

Uso en la Pi (con el mismo RMW y CYCLONEDDS_URI que mapper.sh):  python3 odom_encoders.py
"""
import math

# Mismos valores que ros/teleop_ros.py (la prueba test_constantes_iguales_a_teleop lo comprueba)
WHEEL_D = 0.065
TRACK = 0.215
TICKS_PER_REV = 994.0
CAL_ESCALA = 1.0773
M_PER_TICK = math.pi * WHEEL_D / TICKS_PER_REV * CAL_ESCALA


def paso(estado, dticks):
    """Integra un paso: estado (x, y, θ), dticks (fl, rl, fr, rr) → nuevo (x, y, θ). Rotación desde los encoders."""
    x, y, th = estado
    dfl, drl, dfr, drr = dticks
    dl = (dfl + drl) / 2.0 * M_PER_TICK
    dr = (dfr + drr) / 2.0 * M_PER_TICK
    d = (dl + dr) / 2.0
    dth = (dr - dl) / TRACK
    medio = th + dth / 2.0
    return x + d * math.cos(medio), y + d * math.sin(medio), th + dth


def main():
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    class OdomEncoders(Node):
        def __init__(self):
            super().__init__('odom_encoders')
            self.estado = None
            self.ultimos = None
            self.pub = self.create_publisher(Odometry, 'odom_enc', 10)
            self.create_subscription(Odometry, 'odom', self.al_odom, 10)
            self.create_subscription(String, 'ruedas', self.al_ruedas, 10)

        def al_odom(self, m):
            if self.estado is None:
                q = m.pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self.estado = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
                self.get_logger().info('pose inicial tomada de /odom')

        def al_ruedas(self, m):
            try:
                ticks = tuple(int(v) for v in m.data.split()[:4])
            except ValueError:
                return
            if self.estado is None:
                return
            if self.ultimos is not None:
                self.estado = paso(self.estado, tuple(a - b for a, b in zip(ticks, self.ultimos)))
            self.ultimos = ticks
            x, y, th = self.estado
            o = Odometry()
            o.header.stamp = self.get_clock().now().to_msg()
            o.header.frame_id = 'odom'
            o.child_frame_id = 'base_enc'
            o.pose.pose.position.x, o.pose.pose.position.y = x, y
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
            self.pub.publish(o)

    rclpy.init()
    rclpy.spin(OdomEncoders())


if __name__ == '__main__':
    main()
