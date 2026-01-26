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

> File: `src/environment.py`

### 1.1 Your Robot In The World

We're designing a simulation environment for a 2D mobile robot. This could represent many types of real-world robots: roombas, ground vehicles, autonomous boats, and even unmanned aircraft at a constant altitude! In each of these scenarios, the robot in question has the same kinematic state variables: x position, y position, and heading. Our environment class should keep track each of these values.

Speaking of x and y, the environment probably shouldn't be infinite! Aside from the fact that real-world robots often have limitations on where they can travel, a bounded world will be easier to visualize and plot later on. Therefore, the environment should track minimum and maximum legal x and y values that define how far the robot can move.

One final state variable to track is the simulator's current timestep. Presumably, the starting time will be zero, and time should "step" forward regularly (we'll do this a little later). Critically, the size of the timestep should be constant, so be sure that this is either a settable parameter or an unchanging value in your code.

> Coding tip: All of these variables (robot pose, world dimensions, and time) are initialized in the Environment class for you already. Make sure you know which variable is tracking each value before moving on.

### 1.2 Static Environment Features

What else is in the environment besides our robot? The real world is full of stuff: rocks, chairs, shelves, walls, and other stationary things that prevent our robot from moving through them. We can abstract all of this stuff as "obstacles". We can also define them, like our environment, by their minimum and maximum x and y values (this does mean that all our obstacles will be rectangles; I leave it to you to define other types of obstacles if you want to do so). Our environment class should maintain a list of all obstacles.

In robotic navigation, "landmarks" describe any unique, identifiable feature in the environment that a robot can detect (either visually with a camera, with radio communication, or some other way). For the purposes of our simulator, we can abstract all landmarks as single points (possessing an x and y position but no width or height) with some unique identifier (like an ID number or a name). Like obstacles, the environment should maintain a list of all landmarks. The simulator should also have a function to calculate the robot's true bearing and range to a given landmark, since the robot will want to sample this value later on!

In the real world, the line between "obstacles" and "landmarks" is extremely fuzzy. I've separated the two ideas in my simulator for computational simplicity, but if you want to explore defining a data structure that represents both, you are welcome to explore this topic!

> Coding tip: The obstacle and landmark lists have also been initialized for you already. They reference data classes defined in `src/utils.py`, so make sure to read that file to understand what data they store.

### 1.3 Moving Your Robot In The World

At this point, our environment is tracking a robot's position, as well as obstacles and landmarks. But what if we'd like to move that robot around? The environment should possess a function that takes in changes to each of the robot's state variables (dx, dy, and d-theta) and uses them to modify the robot's position in the environment. Don't forget that each time the robot state is updated, the environment should increment its current timestep as well!

> Coding tip: Fill in the function `robot_step()` with this functionality!

But are all states legal? There are constraints to where a real robot can physically be -- it cannot be inside an obstacle, nor should it ever be outside the bounds of the environment. We need to make sure this is true in our simulator, too. To do this, the environment should possess a function that takes in a robot position (x and y) and validates that position using the environment's limits and its obstacles list. This function should return a boolean indicating whether or not the given position is valid.

> Coding tip: Fill in the function `is_valid_position()` with this functionality! 

If some states are invalid, it follows that we should prevent robotic motion that would cause the robot to enter an invalid state. The environment should possess a function that takes in changes to the robot position (dx and dy) and checks whether the resultant position is valid. The function should then return the actual dx and dy to be executed. Now we have a validating function to use in our movement function!

> Coding tip: Fill in the function `validate_xy_motion()` with this functionality; then call it in `robot_step()`!

### 1.4 Logging Ground Truth Data

At this point, we've actually accomplished a very simple simulator. Congratulations! The last thing to do to make this tool useful is to create functions that logs a snapshot of what the environment's states look like at each timestep. All values that change (time, robot pose, and bearing-range from robot to landmarks) should be logged in some standard format. I would recommend either writing the data directly to a CSV file, or [using the pandas library to build the log as a DataFrame.](https://pandas.pydata.org/docs/user_guide/index.html)

> Coding tip: Fill in the function `take_state_snapshot()` with this functionality. It may be helpful to also fill in the helper functions `get_robot_state()` and `get_proximity_to_landmarks()` and call them in `take_state_snapshot()`.

In addition to changing data, it may also be useful to export the static environment data (timestep size, obstacle and landmark positions, world bounds). This could also be saved as a CSV or pandas DataFrame, or you could simply use the [Python pickle library](https://docs.python.org/3/library/pickle.html) to save a simple dictionary of static data as a .pkl file.

> Coding tip: Fill in the function `get_environment_info()` with this functionality.

## 2. Designing the Robot Class

> File: `src/robot.py`

### 2.1 Think-Act-Sense In A Noisy World

As stated before, our simulated robot isn't going to do much in the way of thinking (for now). However, it will be acting and it will be sensing! Just now, we defined an environment with alterable states and measurable features. The robot we are about to model will take actions that modify the environment, and it will sense/measure/sample features of the environment in an attempt to learn more about it.

Whether are robot is acting or sensing, noise will be a factor in the process. We'll be talking about noise quite a lot in this class, but here we'll learn to model and apply noise programatically. For simplicity, we'll primarily be using [Gaussian noise](https://en.wikipedia.org/wiki/Normal_distribution) in this simulator. Gaussian distributions take in two parameters: mean, which is the value that the distribution is centered around; and standard deviation, which describes the spread of the distribution. When it comes to designing noise for sensors, the mean will generally be set to the ground truth value that is being measured, while the standard deviation will be a property of the sensor itself.

[Python's random library provides a built-in Gaussian function](https://www.geeksforgeeks.org/python/random-gauss-function-in-python/), which we'll use anytime a value needs some noise!

### 2.2 Executing Robotic Actions

Let's handle actions first! The primary action a robot takes is motion. This action alters the robot's state -- particularly, its position and heading. Our environment class already has a function to execute a small movement every timestep. But what kinds of motion instructions will our robots receive? In practice, robots are often executing velocity commands, which we'll need to apprxomately integrate using the environment's timestep to convert it into dx, dy, and d-theta values.

If the robot in question can move omnidirectionally (rotary-wing drones, swerve-drive robots), it directly executes x velocity, y velocity, and angular velocity commands. However, many mobile robots (fixed-wing drones, autonomous boats, and self-driving cars) are only capable of executing linear velocity and angular velocity commands. It's up to you which type of command you'd like to give to your robot, but either way, the robot must a function that transforms those velocities into dx, dy, and d-theta values that can be passed to the environment for execution. Keep in mind that the velocity commands must be approximately integrated using the environment's constant timestep size.

> Coding tip: For an omnidirectional robot, fill in `robot_step_translational()`. For a differential-drive robot, fill in `robot_step_differential()`. Both functions should end by passing their resultant dx, dy, and d-theta to the environment's `robot_step()` function.

Noise isn't just a feature of sensors -- it's present in actuators as well, whether that's wheel slippage, jittery command signals, or environmental disturbances that cause the robot's actual motion to differ from its intended motion. We can model this in our function by immediately setting the input velocities to samples of Gaussians centered on the ground truth input velocity. Choosing the standard deviation value is a design decision that I leave to you!

### 2.3 Sampling the Environment With Sensors

Sensing covers a vast set of capabilities, but there are two broad categories that generalize what sensors measure: proprioceptive sensors and exteroceptive sensors.

Proprioceptive sensors measure a robot's internal state. For example, wheel encoders measure how fast a robot's wheels are turning, IMUs measure a robot's acceleration, and rotary potentiometers measure the angle of a robot's joint. These sensors are most useful for maintaining internal states with controllers. While they can also be used to make estimates of a robot's relationship to the rest of the world, they cannot directly observe that relationship.

Exteroceptive sensors measure the world and relate the robot to it. For example, cameras can measure the robot's distance from other objects, GPS measures the robot's position in the world, and magnetometers measure the robot's heading relative to the magnetic North Pole. These sensors are useful because they provide grounded (if noisy) information about where the robot is.

The robot class possesses a list of sensors. We're going to define our sensor classes in a separate file, and our robot will possess instances of each sensor. There's no code to write here, but take note of the function that will eventually take readings from all sensors.

> Coding tip: The function we're referring to is called `take_sensor_measurements()`. We'll fill it in after making our sensors!

## 3. Designing Sensor Classes

> File: `src/sensors.py`

### 3.1 The Sensor Interface Class

Provided for you is the `SensorInterface` class, which is an Abstract Base Class (ABC). It defines a set of required variables and functions that all other sensor classes must have. Whenever you make a new sensor class, it should inherit the SensorInterface class, which will require that the child class possesses its variables and functions.


According to SensorInterface, all sensors must possess a name, a reference to the robot it belongs to, a sampling interval, and the time of its last measurement. The last two properties are the most interesting, as certain sensors take measurements much more frequently than others!

> Coding tip: Check out `SensorInterface` class! All properties have getters, but only `last_meas_t` has a setter, because it's the only one that changes.

This guide will walk you through the implementation of one proprioceptive sensor and one exteroceptive sensor. However, many other sensors are possible! Some ideas for other sensors: IMU, GPS, LiDAR, visual odometry...

### 3.2 Designing A Wheel Encoder

We'll start by designing a wheel encoder. Wheel encoders measure the speed of the robot (linear/angular or x/y/angular, depending on which type of motor command you chose!). You can see the template for the class in `src/sensors.py`. The class's `__init__()` function is already complete, but you are going to fill in the `sample()` function yourself!

> Coding tip: Check out the `WheelEncoder` class! The `__init__()` function provides some default values for the sensor's sampling interval and noise constants. Feel free to pass in different values when instantiating the class, but keep these defaults as they provide a realistic reference!

> Coding tip: The `WheelEncoder` `__init__()` function is written for a robot with a differential-drive motion style. If your robot is instead doing omnidirectional style, be sure to create noise constants for each command velocity (x, y, angular) to replace the existing ones.

To sample the robot's velocities, access the last executed motor commands using the class's `robot` property. Then, use the Python `random.gauss()` function to sample a distribution with the means centered at the true motor commands and the standard deviations set to the class's noise constants.

> Coding tip: Each sensor should sample a different distribution for each unique value the sensor is measuring. For a differential-drive wheel encoder, this refers to linear and angular velocity. For a omnidirectional wheel encoder, this refers to x velocity, y velocity, and angular velocity. Remember to consider the units of your noise -- an error of 0.5 is much more dramatic in radians than in meters!

Once the sensor has taken noisy measurements, the `sample()` function should return the values. Later on, we'll put all `sample()` function calls in the robot's `take_sensor_measurements()` function, which will utilize the returned values!

### 3.3 Designing a Landmark Pinger

For our exteroceptive sensor, we'll design a landmark pinger. This sensor is abstracting a few different real-life sensors: cameras with feature extraction capabilities, acoustic transducers, and radio beacons all get at the general idea that a robot should be able to sense its bearing and range to a landmark of a known location. Our simulated environment has landmarks of known locations, so this is a good sensor for our robot to have!

Just like the wheel encoder, you can see the template for the class in `src/sensors.py`. The class's `__init__()` function is already complete, but you are going to fill in the `sample()` function yourself!

To sample the robot's bearing and range to all landmarks, access the robot's environment property and call the `get_proximity_to_landmarks()` function. Then, use Python `random.gauss()` function to sample a distribution with the means centered at the true values and the standard deviations set to the class's noise constants. Remember to save the measurements for every landmark!

> Coding tip: Unlike the measurements we've done so far, the landmark pinger's range measurement has a proportional noise component to its standard deviation. This means that range measurements will be noisier the further away the robot is from the landmark! To factor this in, sum the constant noise component with the product of the proportional noise component and the true range value.

Also, note that the landmark pinger has one additional property: maximum range. If the robot is outside the maximum range for a particular landmark, it should log empty or infinite values for that landmark's bearing-range measurement rather than sampling the ground truth value.

Once the sensor has taken noisy measurements from every landmark, the `sample()` function should return the values. Later on, we'll put all `sample()` function calls in the robot's `take_sensor_measurements()` function, which will utilize the returned values!

### 3.4 Aggregate Sampling On Intervals

> File: `src/robot.py`

Now that we've designed a few sensors, let's equip them to our robot. First, instantiate one of each sensor in the robot's `sensors` list. Next, loop through each item of the `sensors` list in `take_sensor_measurements()`. For each sensor, determine if sufficient time has passed for the sensor to resample the environment using its interval, time of last measurement, and the environment's current time. If this is the case, call the sensor's `sample()` function and save the output! Finally, outside the loop, return all samples in a table format similar to the environment class's `take_state_snapshot()` function.

## 4. Putting It All Together In A Main File

## 5. Reading Input Files and Writing Output Files

### 5.1 Useful File Structures for Input Data

### 5.2 Useful File Structures for Output Data
