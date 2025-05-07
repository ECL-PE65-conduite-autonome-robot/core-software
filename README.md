# Core Software

## Introduction

This project takes place in the cursus at Ecole Centrale Lyon in first year and is our study project. We are a team of six members.

*Disclaimer: We had no prior knowledge of ros2 at the outset, so some of the code may contain errors.*

## Objectives

- Development and integration of software solutions on the ROS2 platform.
- Application of navigation techniques and multi-sensor data fusion for the navigation of
navigation of mobile robots.

What should we do ?
Integrate various sensors on the robot: [Rosbot Pro - Orin NX](https://eu.robotshop.com/fr/products/roboworks-rosbot-pro-orin-nx)

## Docker

sudo docker exec core-software-ros2-core-software-1 bash -c "source install/setup.bash && ros2 run core_py lifecycle"

## Roadmap - Sensor Integrations

### Before March 2025

Camera based sensors :
- [ ] AWR1243BOOST radar MMIC + DCA1000EVM
- [ ] Caméra Lucid Vision PHX050S
- [ ] Caméra Orbbec Astra Depth Camera
- [ ] ADS-B  dongle RTL-SDR v3
- [ ] µIMU Litef
- [ ] odomètre SparkFun - PAA5160E1

AHRS :
- [ ] SBG Ellipse-A

INS :
- [ ] SBG Ellipse-E 

GNSS :
- [ ] U-blox EVA-M8 series (EVK-M8)

### Before June 2025

Lidar :
- [ ] Lidar Leishen LSLiDAR m10

AHRS :
- [ ] Xsens MTI-680
- [ ] UAV navigation polar 300
- [ ] Advanced navigation Motus

GNSS :
- [ ] Trimble BX940

### Later

AHRS :
- [ ] Vectornav VN100

INS :
- [ ] Xsens MTI-710
- [ ] Advanced navigation Certus Mini
- [ ] Vectornav VN200

GNSS :
- [ ] Septentrio AsteRx SB3 ProBase
- [ ] Novatel OEM7600

## Install

Clone the repository
```bash
git clone https://github.com/ECL-PE65-conduite-autonome-robot/core-software.git
```

## Build and test

```
source /opt/ros/humble/setup.bash
colcon build
colcon test && colon test-result --verbose
```

## Adding new sensors

Add them in:
- config/parameters/satic_params.yml
- config/parameters/dynamic_params.yml

## Configuration

Note: *The config folder is the default configurations which will be used during build, but during execution you should look at share/core_py/config for seeing live updates*


## Contributing

Please follow our [CONTRIBUTING GUIDE](CONTRIBUTING.md)