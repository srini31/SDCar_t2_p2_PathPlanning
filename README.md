# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
[//]: # (Image References)
[image1]: ./results/1_initial_screen.png "Initial screen"
[image2]: ./results/2_side_view.png "Side view"
[image3]: ./results/3_max_speed.png "Max speed"
[image4]: ./results/4_slowdown.png "Slow down"
[image5]: ./results/5_slowdown_without_changing_lane_due_to_a_car.png "Slow down avoid lane change"
[image6]: ./results/6_lane_change_lowspeed.png "Lane change low speed"
[image7]: ./results/7_lane_change_fullspeed.png "Lane change full speed"
[image8]: ./results/8_lane_change_right.png "lane change right"
[image9]: ./results/9_final.png "Final"


### Goals
The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data are provided and there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator
["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value
["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details about the simulator

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


## Project implementation details

#### Initial screen of simulator
![Simulator_home][image1]

The simulator sends the cars current position along with the sensor fusion information of all the other cars on the highway. A trajectory was generated using the spline function using points that are spaced 0.02 seconds apart. The car moves once every 0.02 seconds and there are 50 points per second that the car moves along. The distance travelled every 0.02 seconds can be set in the code which determines the speed of the car. For example, setting 0.5 m for each 0.02 second interval means that the car travels at 0.5 x 0.02 = 25 m/s. A websocket communicates with the simulator and the path planning program.

The data from the previous path is used so that all the unused points (the points not consumed by the car yet) are added back to the next_x_vals and next_y_vals vectors. This ensures the trajectory is smooth and the car does not jump around. Three x,y points 30m, 60m, 90m ahead from the current car position are calculated using the getXY function. The explanation from the project Q & A mentions that the x,y points are first rotated to the local coordinate system of the point and then passed as input to the spline function. Once the spline is generated, the target y point is calculated from the spline using 30 for the x value. A line from the current position to this target (x,y) points is split by a factor of (0.02* ref_vel) to get evenly spaced points on the spline. As mentioned in the initial part of the write up, this spacing of points is related to velocity and distance. Later these (x,y) points are rotated back based on the yaw angle and added to the next_x_vals, next_y_vals vectors.


## Valid Trajectories

### The car is able to drive at least 4.32 miles without incident. 

Here is a picture of car after completing the final run 
[Final]: ./results/9_final.png "Final"

### The car drives according to the speed limit.
The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.
Example of driving at max speed (49.5 m/s)
[Max speed]: ./results/3_max_speed.png "Max speed"

Example of the car slowing down
[Slow down]: ./results/4_slowdown.png "Slow down"

### Max Acceleration and Jerk are not Exceeded.
The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
[Side view]: ./results/2_side_view.png "Side view"

### Car does not have collisions.
The car must not come into contact with any of the other cars on the road. Here is an example where the car slows down and does not change lane due to the presence of another car.
[Slow down and avoid lane change due to a car]: ./results/5_slowdown_without_changing_lane_due_to_a_car.png "Slow down avoid lane change"
###The car stays in its lane, except for the time between changing lanes.
The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

###The car is able to change lanes
The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.
Example of lane change left at low speed
[Lane change at low speed]: ./results/6_lane_change_lowspeed.png "Lane change low speed"

Example of lane change left at full speed
[Lane change at full speed]: ./results/7_lane_change_fullspeed.png "Lane change full speed"

Example of lane change right
[Lane change right]: ./results/8_lane_change_right.png "lane change right"

###There is a reflection on how to generate paths.
The code model for generating paths is described in detail. This can be part of the README or a separate doc labeled "Model Documentation".


#code lines 158 - 191 in main.cpp
This code was referenced from the project Q&A and it helps to calculate if the car is too close to the car ahead in the same lane

#code lines 156 - 177 in helpers.h
This code takes action once we know the car is too close to the car ahead. The first step to find a new lane to switch over to from the current lane. The code from helper function helps in this regard (). 
For the left most lane, the cars only choice is to switch to the right side lane
From the center lane, it can either go left or right, a random variable decides to go left or right
From the right lane, it can only go to the left lane

#code lines 181 - 213 in helpers.h
once we have a probable lane to shift to, the next step is to find out if there is no car within 30m of our car. This code in the helper function checks that every times, the socket message is read from the simulator. If we detect a car within 30m , we slow down and stay in lane otherwise we go ahead and switch.

#code lines 187 - 189 and 201 - 204 in main.cpp
Sometimes, the car ahead brakes suddenly and the code lines 1.....1 in the main function handle this by slowing the car 2.26 m/s which is 10 times the normal slow down speed to avoid an accident.

#code lines 208 - 223 in main.cpp
once the lane is switched and the lane variable is updated, the logic for calculating the points incorporates this lane information into the way points and a new spline trajectory is generated.



## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

