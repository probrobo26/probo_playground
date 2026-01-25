# Building A Robotic Simulation Environment

## 0. Assignment Introduction

### 0.1 Welcome to the Simulator Assignment!

This repository is meant to be a starting point for creativity and self-guided learning for the _Probabilistic Robotics_ course at Olin College of Engineering, providing an initial skeleton for a mobile robot simulation environment. Assignments throughout the course will revisit this simulator: building out more of its capabilities, utilizing it to investigate particular algorithms and methods, and providing a start for deep dive projects that push the class materials even further.

### 0.2 How To Use This Guide

As stated before, it's up to you how much you'd like to use the structure of this repository for your own simulator. If you'd like, you can follow this guide closely, and you will end up with a tool that accomplishes everything you'll need it to for the first unit of the course. You are also welcome to extend, modify, and experiment with the ideas you find here to customize the simulator to your own preferences.

As a reminder, your simulator should include the following features:
- A class that maintains and updates the ground truth state of the world and robot (including obstacles, environment features, robot position, etc)
- A class that reads an input file of motor commands and executes them to move the robot around in the environment
- Classes to model at least one proprioceptive sensor (wheel encoders, IMU) and at least one exteroceptive sensor (GPS, beacon-range to landmarks, LIDAR); each sensor must take noisy samples of the ground truth at specific intervals
- Functionality to write an output file listing ground truth data and sensor data by timestep
- An easy-to-read README with details on how to run the simulator, and a requirements.txt file listing all dependencies


### 0.3 Simulators In the State-Action-Sense Framework

You're now familiar with the state-action-sense framework, which is useful for discussing robotic scenarios. In the simplest cases, simulators purely model state. This includes the state of the world the robot is operating in (i.e. temperature, locations of landmarks, etc) as well of the state of the robot itself (i.e. position in the world, velocity, etc). All simulators maintain a description of the "ground truth" state -- meaning what state values are actually, verifiably true. Simulators are useful tools for testing robotic algorithms before real-world deployment because unlike the real world (which we can only observe through fallible sensors), the simulator provides us access to the ground truth. We can use this information as a baseline to assess the quality of our own algorithms. Of course, whether or not our robotic algorithms can discern the simulator's ground truth values is a topic that we will explore at length in this class!

In this simulator assignment, in addition to state, you will also model the "sense" portion of the framework in this simulator. This means that in addition to outputting a sequence of ground truth data over time (what actually happened), the simulator will also output a sequence of sampled observations of the ground truth made by various sensors (what the robot thinks may have happened). This will be useful for the prediction unit, in which we will explore algorithms that attempt to resolve noisy observations into estimates that are closer to the ground truth.

As of now, the simulator does not model the "action" portion of the framework. Instead, the robot in this simulator is remote-controlled by an input file of motor commands. Later in the semester, as we explore planning under uncertainty, we may develop algorithms that generate an input file to the simulator, opposite to how estimation algorithms from the prediction unit will read the simulator's output files to run. In this way, all robotic "thinking" occurs outside of the simulator.

## 1. Designing the Environment Class

### 1.n Unit Testing the Environment Class

## 2. Designing the Robot Class

### 2.n Unit Testing the Robot Class

## 3. Designing Sensor Classes

### 3.n Unit Testing the Environment Class

## 4. Utilities, Visualizations, and Beyond

### 5.1 Data Classes for Common Data Structures

### 5.2 Visualizing the Simulation Environment

### 5.3 Creating a requirements.txt file

## 5. Reading Input Files and Writing Output Files

### 5.1 Useful File Structures for Input Data

### 5.2 Useful File Structures for Output Data
