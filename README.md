# Planning and Decision Making Project
2020-Q2 Group 20

Bram Bornhijm (4692721), Rogier Ketwaru (4698975), Alexander Keijzer (4372697), Wouter Meijer (4691659)

## Quad-rotor Routing Using RRT Sampling Based Path Planners in R3
This repository contains the code written for the Planning and Decision Making project by group 20.
The project implements multiple RRT-based path planners in different environments.

![Gif of Path Planning](https://i.gyazo.com/8712fa148fc2c0b92ac9f33c0b42fb59.gif)

## Usage
This project is written in Python 3 and uses the packages: **numpy**, **pyquaternion** and **matplotlib**. To be able to run the code these packages first have to be installed through your package manager.

**NOTE:** On some Windows machines with pip installations the matplotlib animation toolbox does not work! Please switch to Linux or Anaconda.

Run the pathfinding and simulation on the default environment with `python main.py`.

## Structure
- `main.py` is the entry point and contains the setup and visualization of the simulation
- `drone_class.py` contains the quad-rotor model definition and simulation
- `RRT.py`, `RRT_star.py`, `graph.py` & `A_star.py` contain the path planning code
- `obstacles.py` & `collision_detector.py` contains the obstacle definitions collision detection
- `config.py` contains the list of obstacles for the current envirnoment
- `controller.py` contains the quad-rotor path following control
