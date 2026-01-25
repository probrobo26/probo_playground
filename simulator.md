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

### 1.1 Your Robot In The World

We're designing a simulation environment for a 2D mobile robot. This could represent many types of real-world robots: roombas, ground vehicles, autonomous boats, and even unmanned aircraft at a constant altitude! In each of these scenarios, the robot in question has the same kinematic state variables: x position, y position, and heading. Our environment class should keep track each of these values.

Speaking of x and y, the environment probably shouldn't be infinite! Aside from the fact that real-world robots often have limitations on where they can travel, a bounded world will be easier to visualize and plot later on. Therefore, the environment should track minimum and maximum legal x and y values that define how far the robot can move.

One final state variable to track is the simulator's current timestep. Presumably, the starting time will be zero, and time should "step" forward regularly (we'll do this a little later). Critically, the size of the timestep should be constant, so be sure that this is either a settable parameter or an unchanging value in your code.

### 1.2 Static Environment Features

What else is in the environment besides our robot? The real world is full of stuff: rocks, chairs, shelves, walls, and other stationary things that prevent our robot from moving through them. We can abstract all of this stuff as "obstacles". We can also define them, like our environment, by their minimum and maximum x and y values (this does mean that all our obstacles will be rectangles; I leave it to you to define other types of obstacles if you want to do so). Our environment class should maintain a list of all obstacles.

In robotic navigation, "landmarks" describe any unique, identifiable feature in the environment that a robot can detect (either visually with a camera, with radio communication, or some other way). For the purposes of our simulator, we can abstract all landmarks as single points (possessing an x and y position but no width or height) with some unique identifier (like an ID number or a name). Like obstacles, the environment should maintain a list of all landmarks. The simulator should also have a function to calculate the robot's true bearing and range to a given landmark, since the robot will want to sample this value later on!

In the real world, the line between "obstacles" and "landmarks" is extremely fuzzy. I've separated the two ideas in my simulator for computational simplicity, but if you want to explore defining a data structure that represents both, you are welcome to explore this topic!

### 1.3 Moving Your Robot In The World

At this point, our environment is tracking a robot's position, as well as obstacles and landmarks. But what if we'd like to move that robot around? The environment should possess a function that takes in changes to each of the robot's state variables (dx, dy, and d-theta) and uses them to modify the robot's position in the environment. Don't forget that each time the robot state is updated, the environment should increment its current timestep as well!

But are all moves legal? There are constraints to how a real robot can physically move -- it cannot pass through obstacles, nor should it be able to exit the bounds of the environment. We need to make sure this is true in our simulator, too. To do this, the environment should possess a function that takes in a robot position (x and y) and validates that position using the environment's limits and its obstacles list. This function should return a boolean indicating whether or not the given position is valid. Now our state-changing function can validate requested moves before actually executing them!

### 1.4 Logging Ground Truth Data

At this point, we've actually accomplished a very simple simulator. Congratulations! The last thing to do to make this tool useful is to create functions that logs a snapshot of what the environment's states look like at each timestep. All values that change (time, robot pose, and bearing-range from robot to landmarks) should be logged in some standard format. I would recommend either writing the data directly to a CSV file, or [using the pandas library to build the log as a DataFrame.](https://pandas.pydata.org/docs/user_guide/index.html)

In addition to changing data, it may also be useful to export the static environment data (timestep size, obstacle and landmark positions, world bounds). This could also be saved as a CSV or pandas DataFrame, or you could simply use the [Python pickle library](https://docs.python.org/3/library/pickle.html) to save a simple dictionary of static data as a .pkl file.

## 2. Designing the Robot Class

### 2.n Unit Testing the Robot Class

## 3. Designing Sensor Classes

### 3.1 Proprioceptive and Exteroceptive Sensors

### 3.n Unit Testing the Environment Class

## 4. Utilities, Visualizations, and Beyond

### 5.1 Data Classes for Common Data Structures

### 5.2 Visualizing the Simulation Environment

### 5.3 Creating a requirements.txt file

## 5. Reading Input Files and Writing Output Files

### 5.1 Useful File Structures for Input Data

### 5.2 Useful File Structures for Output Data
