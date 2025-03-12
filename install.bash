#!/bin/bash

source /opt/ros/humble/setup.bash

ROS2_WS=/home/pe65/core-software

if [ -z "$ROS_DISTRO" ]; then
    echo "ROS 2 n'est pas configuré. Assurez-vous que ROS 2 est installé et configuré avant d'exécuter ce script."
    exit 1
fi


echo "Mise à jour des paquets et installation des dépendances..."
sudo apt update
sudo apt upgrade

cd ~/pe65

sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev

echo "Installing libuvc library"

git clone "https://github.com/libuvc/libuvc.git"
cd libuvc
if [ ! -d "build" ]; then
    mkdir "build"
fi
cd build
cmake .. && make -j4
sudo make install
sudo ldconfig # Refreshing the link library

sudo apt install nlohmann-json3-dev

sudo apt upgrade

cd "$ROS2_WS"

# Création du workspace ROS 2 s'il n'existe pas
if [ ! -d "$ROS2_WS/src" ]; then
    echo "Création du workspace ROS 2..."
    mkdir -p "$ROS2_WS/src"
fi

# Clonage du dépôt Git
echo "Clonage du dépôt ros2_astra_camera..."
cd "$ROS2_WS/src"
if [ ! -d "ros2_astra_camera" ]; then
    git clone "https://github.com/orbbec/ros2_astra_camera.git"
else
    echo "Le dépôt est déjà cloné."
fi

cd "$ROS2_WS/src/ros2_astra_camera/astra_camera/scripts"
sudo bash install.sh
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "Project build"
cd "$ROS2_WS"
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release