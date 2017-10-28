# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Overview
---
This is my implementation of project 1 of term 3 of the Udacity - Self-Driving Car NanoDegree . You can find the original repo under [CarND-Path-Planning-Project](https://github.com/udacity/CarND-Path-Planning-Project).

## Code overview
---
The code is divided into 4 classes: Main, Vehicle, SimplePredictionVehicle and Utils. Each on of them has it's own role.
1. Main: Needed main class, provided by Udacity. Handles communication with simulator.
2. SimplePredictionVehicle: Simple model of a vehicle. Can predict future s position and the path, the current lane, the state of the vehicle (see below).
3. Vehicle: Class where the path planning takes place.
4. Utils: Serveral helper methods, which didn't fit into other classes. Most of the provided helper functions (e.g. getFrenet) are here.

## Reflection on how to generate paths
---
The path planning of my solution depends on a vehicle object, which is created on startup (line 67 in main.cpp). It is updates everytime a 'telemetry' event is given from the simulator. Everytime, the x, y, yaw, s, d and speed values are updated (lines 84-93 in main.cpp). With the previous_path_x, previous_path_y, end_path_s, end_path_d and the sensor fusion data the vehicle plan the path to take (line 105 in main.cpp) next depending on it's onw state. Possible states are:
* `KL`: Keep lane
* `LCL`: Lane change left
* `LCR`: Lane change right

The path planning consists of 5 steps (lines 307-324 in vehicle.cpp):
1. Correct s value if there was already a path planned
2. If the current state is not `KL` then check if the vehicle already successfully changed lane depending on it's `d` value
3. Categorize detected vehicles in range into the 3 lanes
4. Calculate the next best state
5. Calculate path depending on previous calculated state

Each of this step is now explained in detail.

### Correct s value if there was already a path planned
If the size of previous_path_x is greater than 0, current s value is end_path_s.

### If the current state is not `KL` then check if the vehicle already successfully changed lane depending on it's `d` value
If the state of the car is not `KL`, the car is performing a lane change. If the car is nearly in the middle of the target lane, the state is turned back to `KL`.

### Categorize detected vehicles in range into the 3 lanes (lines 61-116 in vehicle.cpp)
The by sensor fusion fusion detected vehicles are saved into different vectors, depending on their d value. Car's with a d value < 0 are ignored.
* Lane 1: 0 < d < 4
* Lane 2: 4 <= d < 8
* Lane 3: else (lines 25-34 in simplePredictionVehicle.cpp)

If the `s`value of them is more than 15m less than the current one of the car or if they are more than 90m away of our car.
Note that a SimplePredictionVehicle object is generated out of the sensor fusion data. With this, further calculations and predictions are easier. As the sensor fusion data is handled in this part, it also check's if the car is too close to the one ahead. If yes, it sets the private `_too_close` variable to true.

### Calculate the next best state (lines 231-274 in vehicle.cpp)
Based on the possible next states a simple cost function defines if the car should change lane or not. For each possible state, the vehicle calculates a path of itself (line 241 in vehicle.cpp). Then, for each detected vehicle in the future lane of the state, the car checks if it will collide with it (lines 245-251 in vehicle.cpp and lines 169-188 in utils.cpp). For each possible collision a value of 1000 is added to the costs.

Afterwards, the car predicts it's future speed (line 253 in vehicle.cpp and lines 36-44 in vehicle.h). The difference between the maximum allowed speed and the predicted speed is added as cost with a factor of 2. If the car will perform a lane change, the number of lanes to change is added with a factor of 3 to the cost. The state with the smallest cost is set as current state.

### Calculate path depending on previous calculated state
Afterwards, the car calculates it's path depending on it's state (function `calculate_path_depending_on_state` lines 118-229 in vehicle.cpp). Note that this function was already used in the previous step. This function was shown in the Udacity [Project Walkthrough and Q&A](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d) lecture, but I will briefly explain it.

First, the cars current x, y and yaw are safed as reference points (lines 127-129 in vehicle.cpp). If previous path is empty then we use the currenct location as starting reference (lines 132-141 in vehicle.cpp). Otherwise redefine reference state as previous path and points (lines 143-157 in vehicle.cpp). Next, evenly split waypoints with a distance of 30 m (lines 160-163 in vehicle.cpp). All these points are saved in a vector of spaced (x,y) points and transformed to local car coordinates (lines 172-181). The waypoints are afterwarss interpolated with spline (line 187).

The previously calculated and already send to the simulator waypoints are then added to our next path to avoid jerks and rapid path changes (lines 193-197). Then we calculate how to break up spline points so thate we travel at our desired reference velocity and the rest of our path is filled up (lines 200-228 in vehicle.cpp). Part of this process is to rotate the path point back to global (x,y) variables (line 220, 221). The number of points in our path is set to 30 (line 207, for-loop).


## How to run this project
---
### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
