#### Udacity Self-Driving Car Engineer Nanodegree

# Term 2 / Project 5: PID Controller


##### &nbsp;

## Goal
The goal of this project is to autonomously drive a car around the track in a simulator using a Predictive Control Model. The vehicle must safely navigate the course without leaving the track and there's a secondary goal to maximize its driving speed measured in miles per hour (MPH). To achieve these goals, students must build a Model Predictive Controller (MPC) that adjusts the vehicle's steering angle and throttle.

##### &nbsp;

## Background
This is the third project in which we've developed a model to navigate the simulator track, each of them utilizing different models types and data inputs. The first project was [Behavioral Cloning](https://github.com/tommytracey/udacity/tree/master/self-driving-nano/projects/3-behavioral-cloning), in which we developed a deep learning model based on inputs from three cameras mounted on the front of the car. For the second project, we built a [PID Controller](https://github.com/tommytracey/Udacity-CarND-Term2/tree/master/p4-PID-control) based on the cross-track error (CTE), i.e. the distance from the center of the track. For this third project, we need to build a Model Predictive Controller (MPC) given the vehicle's telemetry data (position, velocity, heading) and a series of waypoints for the stretch of track immediately ahead.

The advantage of a MPC is that it uses third order polynomials to determine the best path for the vehicle. Then, it uses a kinematic model and cost function to find the best actions (steering, throttle) to achieve this path, while accounting for the vehicle's constraints (e.g. its maximum steering angle). This type of model requires more computation, but it results in much smoother steering and throttle controls. However, one of the challenges is to ensure your model is efficient enough to control the vehicle in real-time, plus the 0.1 second delay that's added to the simulation to mimic real-world system latency.

##### &nbsp;

## Approach
For this project we used a global kinematic model, which is a simplification of a dynamic model that ignores physical forces such as gravity, tire friction, and vehicle mass.

The kinematic model consists of a vehicle state and actuators. The vehicle state is represented as a vector `(px, py, psi, v, cte, epsi)`, where `px` is the car's x coordinate, `py` is the car’s y coordinate, `psi` is the car’s orientation or heading direction, `v` is the car's velocity, `cte` is the cross track error, and `epsi` is the error in the car's orientation compared to the reference trajectory.

Here are the kinematic equations used to update the vehicle state:

<img src="results/kinematic-equations.png" width="75%" /></a>

##### &nbsp;

The actuators are the set of controls used to navigate the vehicle. Only two actuators are used in this project: `delta` and `acceleration`. Acceleration (`a`) is the throttle value between -1 and 1 that should be applied to vehicle (negative values are for braking). Meanwhile, `delta` represents the steering angle that should be applied, accounting for the constraints to vehicle's steering radius.

Here are the equations used to calculate the actuator commands:

<img src="results/actuator-equations.png" width="75%" /></a>

##### &nbsp;

[Here](https://github.com/tommytracey/Udacity-CarND-Term2/blob/master/p5-model-predictive-control/src/main.cpp#L120) is the part of my code where this model is implemented, accounting for system latency. And [here](https://github.com/tommytracey/Udacity-CarND-Term2/blob/master/p5-model-predictive-control/src/MPC.cpp#L8) is the final set of parameters that I arrived at mostly via trial and error, plus a few hints from threads in the project's Slack channel.

I ultimately settled on values of `N = 17` (timestep length) and `dt = 0.1` (elapsed duration between timesteps). I experimented with higher values for `N`, but it required more computation and predicting that far into the future seemed unnecessary even at speeds of 70-80 MPH. If the car were traveling at 100 MPH, then perhaps 20 timesteps would be appropriate. But any more than that is probably a waste of computation and could hinder the model's ability to produce actuator commands quickly enough. Conversely, with lower values for N, the model would not take into account enough of the upcoming track, resulting in a set of actuator commands that don't properly plan for sharp turns.

A similar pattern emerged while experimenting with values for `dt`. With smaller values, the model requires more compute resources, which are seem superfluous unless the vehicle is traveling at very high speeds (100+ MPH). At my target speed of 70+ MPH, producing actuator commands more frequently than 0.1 seconds seemed unnecessary based on my initial trial runs. This gave the car enough time to compute and execute the commands and drive smoothly around the track. However, with larger values for `dt` there was too much time elapsing between actuations, and therefore, the car would react too slowly to turns when traveling at higher speeds. 

##### &nbsp;

## Results
Ultimately, I was able to get the car to safely navigate the track at least 2 times with a top speed of 74 MPH.

[Here](https://youtu.be/ATElmSKxF2g) is a video showing the results.

<a href="https://youtu.be/ATElmSKxF2g"><img src="results/video-thumbnail.png" width="60%" /></a>



##### &nbsp;
---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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
