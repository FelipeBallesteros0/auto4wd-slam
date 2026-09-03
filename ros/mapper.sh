#!/bin/bash
trap 'kill 0' EXIT
source /opt/ros/jazzy/setup.bash
# Todo el grafo ROS vive en la Pi; Foxglove entra por websocket :8765, no por DDS.
export ROS_LOCALHOST_ONLY=1
# CycloneDDS en vez de FastDDS: el descubrimiento de FastDDS en loopback fallaba
# con los suscriptores que llegan tarde (CLI y foxglove_bridge no recibían /tf;
# 2026-09-03). CycloneDDS descubre en localhost de forma fiable.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" presence_required="false"/></Interfaces><AllowMulticast>true</AllowMulticast></General></Domain></CycloneDDS>'
# Memoria compartida (default de FastDDS): en UNA sola máquina descubre a los que
# llegan tarde MUCHO mejor que UDPv4-loopback (con UDPv4 el foxglove_bridge perdía
# la carrera de arranque y quedaba sin recibir nada -> Foxglove veía canales pero
# cero datos, 2026-09-01). Se limpian los segmentos huérfanos para evitar freezes.
rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* 2>/dev/null

# 1) Publicadores primero (lidar con reintentos, odometría, slam)
( for i in 1 2 3 4 5; do
    ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -p serial_baudrate:=115200 -p frame_id:=laser -p angle_compensate:=true
    echo "[mapper] rplidar salió (intento $i), reintento en 3 s..."; sleep 3
  done ) &
python3 /home/felipe/teleop/teleop_ros.py &
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /home/felipe/teleop/slam_params.yaml &

# 2) Activar slam (transiciones EN LINEA: en background wait -n las contaría como
#    proceso muerto y reiniciaría todo en bucle, bug 2026-08-30)
sleep 14
# Activar slam con reintentos HASTA que quede 'active' (con CycloneDDS el
# descubrimiento en el arranque puede tardar; 3 intentos no bastaban — 2026-09-03)
for i in $(seq 1 15); do
  ros2 lifecycle set /slam_toolbox configure >/dev/null 2>&1
  sleep 2
  ros2 lifecycle set /slam_toolbox activate >/dev/null 2>&1
  sleep 2
  estado=$(ros2 lifecycle get /slam_toolbox 2>/dev/null | head -1)
  echo "[mapper] slam estado: $estado (intento $i)"
  echo "$estado" | grep -q active && break
  sleep 3
done

# 3) foxglove_bridge AL FINAL: con el grafo ya formado y estable descubre todos
#    los publicadores (map, scan, odom, tf) y los reenvía al notebook.
sleep 3
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 &
wait -n
exit 1
