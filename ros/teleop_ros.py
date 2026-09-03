#!/usr/bin/env python3
# Nodo ROS2: mando Xbox360 -> Due (PWM) + odometría desde encoders -> /odom + TF
# Robot 4WD auto4wd. Convención Due: "V <izq> <der>" (-255..255), telemetría "T fl rl fr rr" (x4, adelante=+).
import os, struct, glob, math, threading, time, fcntl
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.clock import Clock, ClockType
import serial as pyserial

# ---- constantes físicas (calibradas 2026-08-30) ----
WHEEL_D  = 0.065          # m
TRACK    = 0.215          # m entre lados izq-der
TICKS_PER_REV = 994.0     # calibrado 2026-08-30: 1 vuelta a mano = 994 cuentas x4
# Calibración empírica 2026-09-01: 3 corridas de 2,10 m con huincha, decodificador
# por muestreo 20 kHz (cero pérdida de flancos) y trayecto recto (gyro 0,1°):
# los encoders daban -7,2%. Escala medida = 1.0773 (equivale a Ø efectivo 70,0 mm).
CAL_ESCALA = 1.0773
M_PER_TICK = math.pi * WHEEL_D / TICKS_PER_REV * CAL_ESCALA
LASER_XY_YAW = (0.0, 0.0, 0.10, 0.0)   # x, y, z, yaw del lidar respecto a base_link

# ---- mando ----
JS_DEV="/dev/input/js0"; EV_FMT="IhBB"; EV_SIZE=struct.calcsize(EV_FMT)
AX_LX, AX_LT, AX_RT, AX_DPY = 0, 2, 5, 7
BT_B=1; DEADZONE=0.12; MIN_PWM=28

def find_serial():
    c=sorted(glob.glob("/dev/serial/by-id/*Arduino*")) or sorted(glob.glob("/dev/ttyACM*"))
    return c[0] if c else None

class TeleopOdom(Node):
    def __init__(self):
        super().__init__("teleop_odom")
        self.pub=self.create_publisher(Odometry,"odom",10)
        self.pub_ruedas=self.create_publisher(String,"ruedas",10)   # diagnóstico: "fl rl fr rr yaw_mrad gz_mrad"
        self.tfb=TransformBroadcaster(self)
        stb=StaticTransformBroadcaster(self)
        t=TransformStamped()
        t.header.stamp=self.get_clock().now().to_msg()
        t.header.frame_id="base_link"; t.child_frame_id="laser"
        x,y,z,yaw=LASER_XY_YAW
        t.transform.translation.x=x; t.transform.translation.y=y; t.transform.translation.z=z
        t.transform.rotation.z=math.sin(yaw/2); t.transform.rotation.w=math.cos(yaw/2)
        stb.sendTransform(t)
        self.ax={}; self.bt={}; self.js=None; self.ser=None; self.buf=b""
        self.limit=0.5; self.last_dpy=0
        self.x=0.0; self.y=0.0; self.th=0.0
        self.last_ticks=None; self.last_t=None
        self.last_yaw=None; self.imu_seen=False; self.gz=0.0
        self.dbg_ruedas=0; self.dbg_odom=0; self.dbg_dt=0; self.dbg_base=0
        # Bucle en HILO PROPIO: los timers de rclpy en esta Pi dejaban de dispararse
        # esporádicamente (executor idle para siempre, visto con py-spy 2026-08-30,
        # con reloj de pared Y monótono). Un hilo simple no depende del executor.
        self._hilo=threading.Thread(target=self._bucle, daemon=True)
        self._hilo.start()
        self.get_logger().info("teleop_odom listo (ticks/rev=%.1f)" % TICKS_PER_REV)

    def open_devs(self):
        if self.js is None:
            try:
                self.js=os.open(JS_DEV, os.O_RDONLY|os.O_NONBLOCK)
                self.get_logger().info("mando conectado")
            except OSError: pass
        if self.ser is None:
            p=find_serial()
            if p:
                try:
                    self.ser=pyserial.Serial(p,115200,timeout=0,write_timeout=0)
                    # fd NO bloqueante: si el CDC del Due se atasca, write() de pyserial
                    # se pega PARA SIEMPRE (py-spy 2026-08-30) — con O_NONBLOCK devuelve
                    # EAGAIN al instante y el contador declara el puerto caído.
                    fcntl.fcntl(self.ser.fd, fcntl.F_SETFL,
                                fcntl.fcntl(self.ser.fd, fcntl.F_GETFL) | os.O_NONBLOCK)
                    self.eagain=0
                    self.get_logger().info("serial %s" % p)
                    self.last_ticks=None
                except Exception: self.ser=None

    def read_js(self):
        if self.js is None: return
        try:
            while True:
                d=os.read(self.js,EV_SIZE)
                if len(d)<EV_SIZE: break
                _t,val,typ,num=struct.unpack(EV_FMT,d); typ&=~0x80
                if typ==2: self.ax[num]=val
                elif typ==1: self.bt[num]=val
        except BlockingIOError: pass
        except OSError:
            try: os.close(self.js)
            except OSError: pass
            self.js=None; self.ax.clear(); self.bt.clear()
            self.get_logger().warn("mando desconectado")

    def mix(self):
        dpy=self.ax.get(AX_DPY,0)
        if dpy<-16000 and self.last_dpy>=-16000: self.limit=min(1.0,self.limit+0.1)
        if dpy> 16000 and self.last_dpy<= 16000: self.limit=max(0.2,self.limit-0.1)
        self.last_dpy=dpy
        rt=(self.ax.get(AX_RT,-32767)+32767)/65534.0
        lt=(self.ax.get(AX_LT,-32767)+32767)/65534.0
        s=self.ax.get(AX_LX,0)/32767.0
        s=0.0 if abs(s)<DEADZONE else (abs(s)-DEADZONE)/(1-DEADZONE)*(1 if s>0 else -1)
        v=rt-lt
        if self.bt.get(BT_B): v=0.0; s=0.0
        if abs(v)<0.04: v=0.0
        w=s*0.75 if v!=0.0 else s*0.6
        l=max(-1.0,min(1.0,v+w))*self.limit; r=max(-1.0,min(1.0,v-w))*self.limit
        def pwm(x):
            if abs(x)<0.03: return 0
            return (1 if x>0 else -1)*int(MIN_PWM+abs(x)*(255-MIN_PWM))
        return pwm(l),pwm(r)

    def odom_from_line(self,ln):
        try:
            p=ln.split()
            if p and p[0]==b"R":
                self.get_logger().info("Due: %s" % ln.decode("utf-8","replace")); return
            if len(p) not in (5,7) or p[0]!=b"T": return
            fl,rl,fr,rr=(int(v) for v in p[1:5])
            yaw_imu=None
            if len(p)==7:
                yaw_imu=int(p[5])/1000.0; self.gz=int(p[6])/1000.0
                if not self.imu_seen:
                    self.imu_seen=True
                    self.get_logger().info("IMU activo: rotación por giroscopio")
        except ValueError: return
        now=self.get_clock().now()
        m=String(); m.data="%d %d %d %d %s %s" % (fl,rl,fr,rr,
            ("%d"%int(yaw_imu*1000)) if yaw_imu is not None else "x",
            ("%d"%int(self.gz*1000)) if self.imu_seen else "x")
        self.pub_ruedas.publish(m)
        # Log de diagnóstico directo a archivo si existe /tmp/GIRO_ON (sin DDS,
        # evita el problema de descubrimiento de nodos recién llegados).
        if self.dbg_ruedas%3==0:
            try:
                import os
                if os.path.exists("/tmp/GIRO_ON"):
                    with open("/tmp/giro.log","a") as f:
                        f.write("%s %s %s\n" % (time.strftime("%H:%M:%S"), m.data,
                                                "OK" if yaw_imu is not None else "NO"))
            except Exception: pass
        self.dbg_ruedas+=1
        if self.dbg_ruedas%100==0:
            self.get_logger().info("dbg: ruedas=%d odom=%d dt_skip=%d base=%d" %
                (self.dbg_ruedas,self.dbg_odom,self.dbg_dt,self.dbg_base))
        if self.last_ticks is None:
            self.last_ticks=(fl,rl,fr,rr); self.last_t=now; self.dbg_base+=1; return
        dfl=fl-self.last_ticks[0]; drl=rl-self.last_ticks[1]
        dfr=fr-self.last_ticks[2]; drr=rr-self.last_ticks[3]
        self.last_ticks=(fl,rl,fr,rr)
        dt=(now-self.last_t).nanoseconds/1e9; self.last_t=now
        if dt<=0 or dt>1.0: self.dbg_dt+=1; return
        dl=(dfl+drl)/2.0*M_PER_TICK; dr=(dfr+drr)/2.0*M_PER_TICK
        d=(dl+dr)/2.0
        if yaw_imu is not None:                      # gyro: rotación real (skid-steer patina)
            if self.last_yaw is None: self.last_yaw=yaw_imu
            dth=yaw_imu-self.last_yaw; self.last_yaw=yaw_imu
        elif self.imu_seen:
            # IMU visto antes pero caído ahora (jumper flaky): MANTENER el rumbo
            # (dth=0) en vez de caer a encoders-basura del skid-steer. Una micro-caída
            # así no rompe el mapa; al reconectar el yaw absoluto retoma sin salto.
            dth=0.0
        else:
            dth=(dr-dl)/TRACK                        # sin IMU nunca: encoders (peor, pero es lo que hay)
        self.x+=d*math.cos(self.th+dth/2.0); self.y+=d*math.sin(self.th+dth/2.0); self.th+=dth
        o=Odometry(); o.header.stamp=now.to_msg(); o.header.frame_id="odom"; o.child_frame_id="base_link"
        o.pose.pose.position.x=self.x; o.pose.pose.position.y=self.y
        o.pose.pose.orientation.z=math.sin(self.th/2); o.pose.pose.orientation.w=math.cos(self.th/2)
        o.twist.twist.linear.x=d/dt; o.twist.twist.angular.z=(self.gz if self.imu_seen else dth/dt)
        self.pub.publish(o); self.dbg_odom+=1
        t=TransformStamped(); t.header.stamp=now.to_msg()
        t.header.frame_id="odom"; t.child_frame_id="base_link"
        t.transform.translation.x=self.x; t.transform.translation.y=self.y
        t.transform.rotation.z=math.sin(self.th/2); t.transform.rotation.w=math.cos(self.th/2)
        self.tfb.sendTransform(t)

    def _bucle(self):
        while rclpy.ok():
            try: self.loop()
            except Exception as e:
                self.get_logger().error("bucle: %r" % e); time.sleep(0.5)
            time.sleep(0.02)

    def loop(self):
        self.open_devs()
        self.read_js()
        if self.ser is None: return
        L,R=self.mix() if self.js is not None else (0,0)
        try:
            try:
                os.write(self.ser.fd, b"V %d %d\n"%(L,R))
                self.eagain=0
            except BlockingIOError:
                self.eagain+=1
                if self.eagain>50: raise RuntimeError("puerto serial atascado (50x EAGAIN)")
            data=self.ser.read(4096)
            if data:
                self.buf+=data
                while b"\n" in self.buf:
                    ln,self.buf=self.buf.split(b"\n",1)
                    self.odom_from_line(ln.strip())
        except Exception as e:
            self.get_logger().warn("serial caido: %s" % e)
            try: self.ser.close()
            except Exception: pass
            self.ser=None

def main():
    rclpy.init(); n=TeleopOdom()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass

if __name__=="__main__": main()
