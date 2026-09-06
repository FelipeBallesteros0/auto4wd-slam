# Imprime el punto más cercano del lidar (ángulo en grados, convención ROS: 0=frente, +90=izquierda) y los 5 más cercanos.
import math, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
class S(Node):
    def __init__(s):
        super().__init__("scan_min"); s.done=False
        s.create_subscription(LaserScan,"/scan",s.cb,qos_profile_sensor_data)
    def cb(s,m):
        if s.done: return
        pts=[]
        for i,r in enumerate(m.ranges):
            if r and r>0.05 and r<12 and math.isfinite(r):
                pts.append((r, math.degrees(m.angle_min+i*m.angle_increment)))
        pts.sort()
        print("mas cercano: %.2f m a %.0f grados" % pts[0])
        print("5 mas cercanos:", ", ".join("%.2fm@%.0f" % p for p in pts[:5]))
        # promedio de los que estan a menos de 15 cm del minimo
        cerca=[p for p in pts if p[0]<pts[0][0]+0.15]
        ang=sum(p[1] for p in cerca)/len(cerca)
        print("grupo cercano: %d puntos, angulo medio %.0f grados" % (len(cerca), ang)); s.done=True
rclpy.init(); n=S()
import time; t=time.time()
while rclpy.ok() and not n.done and time.time()-t<10: rclpy.spin_once(n, timeout_sec=0.5)
if not n.done: print("sin scan")
