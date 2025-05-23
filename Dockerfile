FROM ros:humble

# Mise à jour de base
RUN apt update && apt install -y && apt upgrade -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-yaml \
    python3-setuptools \
    ros-humble-rclpy

# Création de l'espace de travail
WORKDIR /core_software

# Copie des sources
COPY ./src ./src
COPY ./config ./config

# Compilation du workspace
RUN . /opt/ros/humble/setup.sh && colcon build

# Activation de l’environnement au démarrage
CMD ["/bin/bash"]
