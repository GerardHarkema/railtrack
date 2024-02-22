#!/bin/bash

sudo apt install -y python3-colcon-common-extensions
sudo apt-get install -y python3-rosdep
sudo rosdep init
rosdep update
sudo -H apt-get install -y clang

pip install -r requirements.txt
