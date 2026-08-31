#!/bin/bash
trap 'kill 0' EXIT
source /opt/ros/jazzy/setup.bash
# Solo UDP: el transporte de memoria compartida de FastDDS queda en estado
# corrupto tras kills bruscos (segmentos borrados con procesos vivos -> nadie
# se descubre; visto 2026-08-30). UDPv4 es robusto y suficiente.
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# DDS confinado a localhost: el descubrimiento por multicast sobre wlan0 pierde
# paquetes (visibilidad aleatoria de nodos, visto 2026-08-30). Todo el grafo ROS
# vive en la Pi; Foxglove entra por websocket :8765, no por DDS.
export ROS_LOCALHOST_ONLY=1
( for i in 1 2 3 4 5; do
    ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -p serial_baudrate:=115200 -p frame_id:=laser -p angle_compensate:=true
    echo "[mapper] rplidar salió (intento $i), reintento en 3 s..."; sleep 3
  done ) &
python3 /home/felipe/teleop/teleop_ros.py &
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /home/felipe/teleop/slam_params.yaml &
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 &
# transiciones lifecycle EN LINEA: si van en background, wait -n las cuenta
# como proceso muerto y reinicia todo en bucle (bug del 2026-08-30)
sleep 12
for i in 1 2 3; do ros2 lifecycle set /slam_toolbox configure && break; sleep 4; done
sleep 2
for i in 1 2 3; do ros2 lifecycle set /slam_toolbox activate && break; sleep 4; done
wait -n
exit 1
