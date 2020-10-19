# AutoTune

This repo contains the code associated to our paper AutoTune: Metropolis-Hastings Sampling for Automatic Controller Tuning.

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
git clone https://github.com/uzh-rpg/autotune.git
```
3. Clone the dependencies:
```
vcs-import < autotune/dependencies.yaml
```
4. Download the Flightmare Unity Binary RPG_Flightmare.tar.xz for rendering from the [Releases](https://github.com/uzh-rpg/flightmare/releases) and extract it into ```~/autotune_ws/autotune/flightmare/flightrender```.

5. Build:
```
catkin build
```
6. Add sourcing of your catkin workspace and AUTOTUNE_PATH environment variable to your ```.bashrc``` file:
```
echo "source ~/autotune_ws/devel/setup.bash" >> ~/.bashrc
echo "export AUTOTUNE_PATH=~/autotune_ws/src/autotune" >> ~/.bashrc
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

- Add your own trajectory to ```~/autotune_ws/src/autotune/resources/trajectories```

- Change the ```file_name``` parameter in ```~/autotune_ws/src/autotune/params/autotune.yaml``` to match your trajectory file name

