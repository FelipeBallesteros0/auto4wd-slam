import sys, rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
class G(Node):
    def __init__(s):
        super().__init__("guarda_mapa"); s.ok=False
        qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        s.create_subscription(OccupancyGrid, "/map", s.cb, qos)
    def cb(s,m):
        if s.ok: return
        w,h=m.info.width,m.info.height; d=m.data
        with open(sys.argv[1],"wb") as f:
            f.write(b"P5\n%d %d\n255\n"%(w,h))
            rows=[]
            for y in range(h-1,-1,-1):
                row=bytes(205 if v<0 else (255-int(v*2.55)) for v in d[y*w:(y+1)*w]); rows.append(row)
            f.write(b"".join(rows))
        print("mapa %dx%d res=%.3f"%(w,h,m.info.resolution)); s.ok=True
rclpy.init(); n=G()
import time; t=time.time()
while rclpy.ok() and not n.ok and time.time()-t<15: rclpy.spin_once(n, timeout_sec=0.5)
if not n.ok: print("sin mapa")
