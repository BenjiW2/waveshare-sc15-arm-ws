# ROS 2 workspace for a waveshare SC15 based arm i'm working on (`servo_ws`)

* **Hardware** : Raspberry Pi 5 + 5x Waveshare SC-15 TTL bus servos + waveshare gripper-b w/ cf35-12 servo  
* **Software** : ROS 2 Jazzy, custom ros2_control hardware interface  
* **Packages** :
  * `sc15_hw_iface_cpp` – C++ SystemInterface for the SC-15 bus
  * `stservo_sdk_py`   – Python SDK for debugging / config
  * launch & YAML configs in `config/` and `launch/`

To build:

```bash
sudo apt update && sudo apt install ros-jazzy-desktop-full ros-jazzy-ros2-control ros-jazzy-ros2-controllers
mkdir -p ~/servo_ws && cd ~/servo_ws
git clone <this-repo> src
colcon build --symlink-install
source install/setup.bash

ros2 launch sc15_hw_iface_cpp arm_control.launch.py   # coming soon

 ``` 

cat > LICENSE <<'EOF'
MIT License
Copyright (c) 2025 Benji Warburton

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
EOF
