# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project my goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data will be provided. There is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

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

## Project Instructions and Rubric

### Valid Trajectories

* The car is able to drive at least 4.32 miles without incident.
  I ran the car 13 minutes for two loops without incident.

* The car drives according to the speed limit.
  The car starts at 0 mph in its lane (the middle lane) and accelerates up to 49.5 mph. When it is detected too close to its front car, it slows down to avoid incident or change lanes. 

* Max Acceleration and Jerk are not Exceeded.
  No red sign of exceeding the max acceleration or jerk during the whole driving time.

* Car does not have collisions.
  No collisions in the whole driving time.

* The car stays in its lane, except for the time between changing lanes.
  The car changes back to it lane when it is safe to do so after changing to left or right lane for passing.

* The car is able to change lanes.
  The car is able to change lanes when it is getting too close to its front car, and either left lane or right lane is safe (i.e. not too close) to change into.


### Reflection

My implementation of the path planning is added in the main.cpp file which was provided by [Udacity's seed project](https://github.com/udacity/CarND-Path-Planning-Project). The trajectory solution is based on the project walk through video from the Udacity course material.

In main.cpp file, source code lines 248 - 456 is my implementation of the path planning.

### Prediction - Check other cars positions

Lines 256 - 301 uses telemetry event and sensor fusion data to do calculation and then determine for:
- if the car in front of my car in my lane is too close to my car
- if the car in my left lane is too close to my car
- if the car in my right lane is too close to my car

These boolean results then is used to decide what the driving behavior for my car moving to my next way point.

### Path Planning for Driving Behaviors

Lines 304 - 337 describes the path planning of my car's driving behaviors:
- If my car is getting too close to the car in front of me:
-- If I am not in the most left lane, and no car in that lane is too close to my car, then change to the left lane; 
-- Else if I am not in the most right lane, and no car in that lane is too close to my car, then change to the right lane;
-- Else slow down to avoid collision since it is not safe to change lanes. 
- If I am not driving in my lane (because of changing lanes for passing), I would change back to my lane if it is safe to do so. If my car's speed is lower than the speed limit, I would speed up at the acceleration limit.

### Trajactory Calculation

The spline approach is used here in lines 340 - 456. It does the calculation of the trajectory based on the speed and lane output from the behavior, car coordinates and past path points. Coordinates are converted into car space and sampling to create a smooth trajectory.

