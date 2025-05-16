<p align="center"><strong>tita_simulation</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>

## Description

Tita simulation environments, including `Webots` and `Gazebo`.

## Prerequisites

- **Operating System**: Ubuntu 22.04
- **ROS 2**: Humble
- **Webots**: R2023b
- **Gazebo**: classic

## Dependencies
> `Webots` and `Gazebo` can run independently and are not in a mutually dependent relationship.
```bash
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers

# if build webots
sudo apt install ros-humble-webots-ros2

# if build gazebo
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gazebo-ros
```
## Modify Urdf2webots with Hard Joint Limits (optional)
Webots use `urdf2webots` to import urdf/xacro into webots. But builtin `urdf2webots` cannot support hard joint limits. You can modify `urdf2webots` to support hard joint limits.
```
sudo vim /opt/ros/humble/lib/python3.10/site-packages/webots_ros2_importer/urdf2webots/urdf2webots/writeRobot.py
# Comment out line 574-577, like this:
    # if joint.limit.lower != 0.0:
    #     robotFile.write((level + 3) * indent + 'minPosition ' + str(joint.limit.lower) + '\n')
    # if joint.limit.upper != 0.0:
    #     robotFile.write((level + 3) * indent + 'maxPosition ' + str(joint.limit.upper) + '\n')
# Add the following codes to line 527 and line 551 respectively:
        if joint.limit.lower != 0.0:
            robotFile.write((level + 2) * indent + 'minStop ' + str(joint.limit.lower) + '\n')
        if joint.limit.upper != 0.0:
            robotFile.write((level + 2) * indent + 'maxStop ' + str(joint.limit.upper) + '\n') 
```

## Build Package

```bash
# if build webots
colcon build --packages-up-to webots_bridge
source install/setup.bash
ros2 launch webots_bridge webots_bridge.launch.py # webots

# if build gazebo
colcon build --packages-up-to gazebo_bridge 
source install/setup.bash
ros2 launch gazebo_bridge gazebo_bridge.launch.py # gazebo
```
## Docker(webots only)
### Build 
``` bash 
docker build . --file Dockerfile --tag webots_ros2:latest
```
To use the proxy, you need to set the proxy environment variables for the docker container.
```
sudo mkdir -p /etc/systemd/system/docker.service.d
sudo vim /etc/systemd/system/docker.service.d/http-proxy.conf
```
Add the following content to the file, here is local proxy, you can change it to your own proxy.
```
[Service]
Environment="HTTP_PROXY=http://127.0.0.1:8123"
Environment="HTTPS_PROXY=http://127.0.0.1:8123"
```
Restart the docker service:
```
sudo systemctl daemon-reload
sudo systemctl restart docker
```

### Run
``` bash 
xhost +local:root
docker run -it --rm --net=host --privileged --name webots_ros2 -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/workspace -w /workspace -e DISPLAY=$DISPLAY webots_ros2:latest 
```
### Clean the temporary Images
You can run the following command to remove all temporary images:
``` bash
docker system prune
```