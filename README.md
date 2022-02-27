# AutoTune

This repo contains the code associated to our paper "AutoTune: Controller Tuning for High-Speed Flight".

If you use AutoTune in an academic context, please cite the following publication:
```
@ARTICLE{Autotune2022,
  author  = {Loquercio, Antonio and Saviolo, Alessandro and Scaramuzza, Davide},
  title   = {AutoTune: Controller Tuning for High-Speed Flight},
  journal = {IEEE Robotics and Automation Letters}, 
  year    = {2022},
  volume  = {7},
  number  = {2},
  pages   = {4432-4439},
  doi     = {10.1109/LRA.2022.3146897}
}
```

[**Paper**](https://arxiv.org/abs/2103.10698)

[**YouTube-Video**](https://youtu.be/m2q_y7C01So)

[<img src="./youtube-video.gif" title="AutoTune Youtube-Video" width="65%">](https://youtu.be/m2q_y7C01So)

### Abstract

Due to noisy actuation and external disturbances, tuning controllers for high-speed flight is very challenging. In this paper, we ask the following questions:  How sensitive are controllers to tuning when tracking high-speed maneuvers? What algorithms can we use to automatically tune them? To answer the first question, we study the relationship between parameters and performance and find out that the faster the maneuver, the more sensitive a controller becomes to its parameters. To answer the second question, we review existing methods for controller tuning and discover that prior works often perform poorly on the task of high-speed flight. Therefore, we propose AutoTune, a sampling-based tuning algorithm specifically tailored to high-speed flight. In contrast to previous work, our algorithm does not assume any prior knowledge of the drone or its optimization function and can deal with the multi-modal characteristics of the parameters' optimization space. We thoroughly evaluate AutoTune both in simulation and in the physical world. In our experiments, we outperform existing tuning algorithms by up to 90% in trajectory completion. The resulting controllers are tested in the AirSim Game of Drones competition, where we outperform the winner by up to 25% in lap-time. Finally, we show that AutoTune improves tracking error when flying a physical platform with respect to parameters tuned by a human expert.

## Installation

### Requirements

The code was tested with Ubuntu 18.04, ROS Melodic.

### AutoTune in Flightmare

1. Create a catkin workspace with the following commands by replacing <ROS VERSION> with the actual version of ROS you installed:
```
cd
mkdir -p autotune_ws/src
cd autotune_ws
catkin config --init --mkdirs --extend /opt/ros/$ROS_DISTRO --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
```
2. Clone the AutoTune repository:
```
cd ~/autotune_ws/src
git clone https://github.com/uzh-rpg/mh_autotune.git
```
3. Clone the dependencies:
```
vcs-import < mh_autotune/dependencies.yaml
```
***Note 1, if you get error like 'vcs-import: command not found', you need to install the vcstool. For help, visit [here](http://wiki.ros.org/vcstool)***

***Note 2, if you get error like 'Could not determine ref type of version: git@github.com: Permission denied (publickey)', visit [this page](https://docs.github.com/en/github/authenticating-to-github/troubleshooting-ssh/error-permission-denied-publickey) to resolve the issue. You could [add a SSH key](https://docs.github.com/en/github/authenticating-to-github/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) and connect it to your Github account for example to solve this error***                                         
                                          
4. Download the Flightmare Unity Binary RPG_Flightmare.tar.xz for rendering from the [Releases](https://github.com/uzh-rpg/flightmare/releases) and extract it into ```~/autotune_ws/src/flightmare/flightrender```.

5. Build:
```
catkin build
```

***Note 3, you could get build errors such as : "set GLOG_INCLUDE_DIR to directory containing glog/logging.h" for example from building [rotors_simulator](https://github.com/ethz-asl/rotors_simulator) module, which is used inside this project. These errors occur because ros package dependencies for each modules are not installed yet. To do this, visit [here](http://wiki.ros.org/rosdep), or just simply type `rosdep install --from-paths src --ignore-src -r -y` command at the `~/autotune_ws` directory***

***Note 4, you could get build errors like "Could not find a package configuration file provided by "gazebo_plugins"". This can occur for example if you only have installed 'ros-YOUR_DISTRO_NAME-desktop' through apt package manager, and not 'ros-YOUR_DISTRO_NAME-desktop-full' (which includes the gazebo interface & gazebo itself!). So you could try `sudo apt install ros-YOUR_DISTRO_NAME-desktop-full` to avoid this issue***                                       
                                          
                                          
6. Add sourcing of your catkin workspace and AUTOTUNE_PATH environment variable to your ```.bashrc``` file:
```
echo "source ~/autotune_ws/devel/setup.bash" >> ~/.bashrc
echo "export AUTOTUNE_PATH=~/autotune_ws/src/mh_autotune" >> ~/.bashrc
source ~/.bashrc
```

## Basic Usage

Once you have installed the dependencies, you will be able to tune controllers using AutoTune.

### Let's tune a controller

In this example, we show how to use AutoTune to tune the model predictive controller [rpg_mpc](https://github.com/uzh-rpg/rpg_mpc) for flying an high-speed trajectory. We use the [RotorS](https://github.com/ethz-asl/rotors_simulator) for the quadrotor dynamics modelling, and [Flightmare](https://github.com/uzh-rpg/flightmare) for image rendering.

Let's launch the simulation! Open a terminal and type:
```
cd ~/autotune_ws
. devel/setup.bash
roslaunch autotune flightmare.launch
```

### Fly your own high-speed trajectories

We provide a set of time-optimal trajectories. However, you can add your own trajectories and use AutoTune to fly them in Flightmare!

To use AutoTune to tune the model predictive controller and fly your own trajectory, you have to follow these steps:

- Add your own trajectory to ```~/autotune_ws/src/mh_autotune/resources/trajectories```

- Change the ```file_name``` parameter in ```~/autotune_ws/src/mh_autotune/params/autotune.yaml``` to match your trajectory file name

## License

Copyright (C) 2021 Alessandro Saviolo
```
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
```
