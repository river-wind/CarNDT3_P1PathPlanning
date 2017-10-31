# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program [![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

### Video of final result

https://www.youtube.com/watch?v=pdG9bko6TMM

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving within +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data about the locations of other cars is provided by the simulator, mimicking the output of a real self-driving car's sensor array.  There is also a sparse map list of waypoints around the highway, mimicking a map of the road. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible.  Note that other cars will try to change lanes too. The self-driving car should avoid hitting other cars at all cost, drive inside of the marked road lanes at all times, make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 or jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, or distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
5. Launch the simulator to observe the code in action.

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

["sensor_fusion"] is a 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position (in map coordinates), car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position (in frenet coordinates)]. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it receives in the waypoint list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner receives should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this, previous_path_x, and previous_path_y are used as the head of the new trajectory to smooth transition between the prior path and the updated path. 

## Spline

To best create smooth trajectories, http://kluge.in-chemnitz.de/opensource/spline/ is leveraged to map out a smooth path between two points, and minimize jerk.

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
* This project requires the Udacity Self Driving Car Nanodegree Term 3 simulator.  You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

## Project Rubric

For successful completion of this project, certain benchmarks must be met.  Below are the benchmarks for this project, and a short description of how each is addressed in this project.  The original ruberic description can be found here: https://review.udacity.com/#!/rubrics/1020/view

#### The code compiles correctly.

The code does compile with CMake and Make.

#### The car is able to drive at least 4.32 miles without incident..

The car can drive more than a full lap, or above 4.32 miles, in the simulator.  Two full laps can be see here: https://www.youtube.com/watch?v=pdG9bko6TMM

#### The car drives according to the speed limit.

The car's target reference velocity is set to 49.5mph, and does not exceed that target.

#### Max Acceleration and Jerk are not Exceeded.

Keeping acceleration per time segment under 0.224 avoids possible max accelleration exceptions, and the use of splines and previous trajectories avoids jerk from exceeding the set threshold.

#### Car does not have collisions.

The behavior planning code caculates action costs with collisions given very large penalties.  As such, it successfully avoids them as it drives the track.

#### The car stays in its lane, except for the time between changing lanes.

The car follows the existing lane based on the distance from Frenet coordinate 'd', which is 0 at the center of the road (yellow line).  With each lane being 4m wide, the left lane equates to a centerpoint of d=2m, the center lane is d=6m, and the right lane is d=10m.  The car avoid leaving the road by penalizing any movement to a position d<2m or d>10m highly.

#### The car is able to change lanes

When presented with a car in front of it which is going slower than the speed limit, the self driving car checks the right and left lanes for traffic.  If there is no obstruction, the lane with the clearest space in front of it is chosen, and the car changes lanes based on a smooth spline-based trajectory.  If both adjacent lanes contain cars ahead, the speed of the two cars is used as the main component of calculating the cost associated with each lane - that forward car being far away and fast-moving is favored over close and slow.

A finite state machine design is employed implicitly in the decision making process, through behavior selection based on costs associated with a limited set of descrete actions.  A formally structured state machine with clearly defined state transition logic and state labels was not employed at this time.

#### There is a reflection on how to generate paths.

Once a destination point is chosen, the path to that point is plotted by taking the existing future path projection up to that moment, and building a new list point to use in creating a new plan.  Two points from the original path, along with three more points projecting the car's future desired position are added to a new vector.  This short set of points is then fed to the very handy spline.h in local car coordinate space.  This tool builds a smooth curve to get from the current car position, through the existing plan points and to the target future locations.  This spline is then used to calculate, in map x,y coordinate space, a new path of 50 waypoints which combines the previous path with additional new points directing the car smoothly towards its new desired destination.  This list of points is then fed back to the simulator to control the car behavior.

## Additional Thoughts

The code relies heavily on the walkthrough template to manage the creation of the new path, and the behavior planning works well enough.  In its current form, however, it is not easily extensible.  If for example, additional cost factors should be employed, or if planning routes across all three lanes together is desired, significant refactoring would be required.  Redesigning the framework of the code into a set of classes to handle each aspect of the decision-making process would both allow the code to be for accepting of additional features in the future, as well as improve maintainability.  The classroom code structure for the Behavior Planning quiz, with classes for the road, the lanes, and the vehicle may be optimal, and was the initial design targeted for this project.  Due to differences in the data input structure, reworking the active steps employed proved simpler than redesigning the class model.  That said, this did reduce flexibility.

Some additional cost factors which should eventually be included: always favoring the right hand lane and only passing to the left of other cars in order to more carefully follow the law.  Eventually, handling speed limit signs directly to determine the reference velocity, and handling exit and entrance ramp conditions would be needed.  In addition, the current behavior planner handles other cars merging into its lane well, but does not do so explicitly.  Therefor, improved behavior could be added to check changes in Frenet "d" positions of nearby vehicles to actively avoid collisions caused by inattentive lane-changes from other vehicles.  In addition, the lane-change behavior could be altered to actively search of openings in faster-moving traffic in order to better plan passing maneuvers. 
