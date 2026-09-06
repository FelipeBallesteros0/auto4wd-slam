# Dibuja el scan actual (negro) sobre el mapa (paredes gris oscuro) usando TF map->laser, y mide
# que fraccion de puntos del scan cae sobre/junto a una celda ocupada del mapa.
import sys, math, time, rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
import tf2_ros
class O(Node):
    def __init__(s):
        super().__init__("overlay"); s.map=None; s.scan=None
        s.buf=tf2_ros.Buffer(); s.tl=tf2_ros.TransformListener(s.buf, s)
        qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        s.create_subscription(OccupancyGrid,"/map",lambda m: setattr(s,"map",m),qos)
        s.create_subscription(LaserScan,"/scan",lambda m: setattr(s,"scan",m),qos_profile_sensor_data)
rclpy.init(); n=O(); t=time.time()
while rclpy.ok() and time.time()-t<12 and not (n.map and n.scan and n.buf.can_transform("map","laser",rclpy.time.Time())):
    rclpy.spin_once(n, timeout_sec=0.2)
if not (n.map and n.scan): print("falta mapa o scan"); sys.exit()
tf=n.buf.lookup_transform("map","laser",rclpy.time.Time()); q=tf.transform.rotation
yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)); tx,ty=tf.transform.translation.x,tf.transform.translation.y
m=n.map; w,h,res=m.info.width,m.info.height,m.info.resolution; ox,oy=m.info.origin.position.x,m.info.origin.position.y
img=[[205 if v<0 else (255 if v<50 else 60) for v in m.data[y*w:(y+1)*w]] for y in range(h)]
occ=set((x,y) for y in range(h) for x in range(w) if m.data[y*w+x]>=50)
hits=0; tot=0; sc=n.scan
for i,r in enumerate(sc.ranges):
    if not (r and math.isfinite(r) and 0.15<r<10): continue
    a=sc.angle_min+i*sc.angle_increment; px=tx+r*math.cos(yaw+a); py=ty+r*math.sin(yaw+a)
    cx=int((px-ox)/res); cy=int((py-oy)/res); tot+=1
    if any((cx+dx,cy+dy) in occ for dx in (-1,0,1) for dy in (-1,0,1)): hits+=1
    if 0<=cx<w and 0<=cy<h: img[cy][cx]=0
rx=int((tx-ox)/res); ry=int((ty-oy)/res)
for dx in (-1,0,1):
    for dy in (-1,0,1):
        if 0<=rx+dx<w and 0<=ry+dy<h: img[ry+dy][rx+dx]=120
with open(sys.argv[1],"wb") as f:
    f.write(b"P5\n%d %d\n255\n"%(w,h))
    for y in range(h-1,-1,-1): f.write(bytes(img[y]))
print("puntos=%d sobre pared=%d (%.0f%%) pose laser en map: x=%.2f y=%.2f yaw=%.0f" % (tot,hits,100.0*hits/max(tot,1),tx,ty,math.degrees(yaw)))
