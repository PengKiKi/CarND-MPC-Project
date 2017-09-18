# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

[image1]: ./MPCdemo80mph.gif

![alt text][image1]


## Goal

The Goal of the project is to use Model Predictive Control (MPC) to drive a car in a game simulator. The server sends reference waypoints (yellow line in the demo video) via websocket, we want to develop a nonlinear model predictive controller (NMPC) to control the steer angle as well as the throttle of the car given the current state of the car. Moreover, the solution must be able to handle 100ms latency in actuation as what happens in the real world.

## The Model

The model is a kinematic model which neglects all dynamical effects such as tire forces, gravity, and mass. The model is non-linear because it accounts for heading direction. 

**state**:
- `x,y` the x and y coordinates of the car
- `psi` the heading direction
- `v` velocity
- `cte` the cross-track error
- `epsi` the orientation error
- `Lf` the length from front to CoG that has a similar radius

**actuators**
- `delta` steering angle
- `a` acceleration

The outputs are acceleration and steering angle. As the equations below, the NMPC model calculates the state for the next timestamp based on the state and actuations from the previous timestep.

[image2]: ./VM.png

![alt text][image2]


## Timestep Length and Elapsed Duration (N & dt)
- N = 10
- dt = 0.1

The product of N and dt defines the prediction horizon. In case of driving the car, the length of prediction horizon should be in seconds at most because the environment is always changing thus any data input beyond that time range won't contribute much into the prediction. In general, short prediction horizons means lower latency in response but maybe instable and inaccurate. Longer prediction horizons have smoother controls but introduce more latency.

The value I chose is from experiment. `dt` should take into account the latency of the actuactor, in this case, 100ms.
`N` should at least be 10 when driving in a high speed to smooth out instable steerings.

## Polynomial Fitting and MPC Preprocessing
Preprocessing is needed to convert from map global coordinate to car's coordinate, see the description [here](https://github.com/udacity/CarND-MPC-Project/blob/master/DATA.md)
```
carx =   cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
cary =  -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);
```

main.cpp: 108 has the logic to do the convervion based on the above logic; then a 3rd order polynomial is fitted to waypoints.

## Model Predictive Control with Latency
In this project 100 millisecond latency is introduced to mimic real world scenario where the actutor output will take some time to be actually propagated and executed. To handle this latency, I tried two different approaches:

- modify the MPC equations to account for the 100 ms latency, which is one timestamp later; i.e: shift one step in timestamp to use previous actuation value
- estimate the car's states for after 100ms before sending it to MPC; we can use the same NMPC update rule to calculate the expected state 100 ms later; then solve the control problem starting from that position.

Also, in high speed scenario, I need to adjust the weight in the cost function to punish / minimize cte, epsi, velocity gap, change rate of delta and acceleration to make the driving smooth and stable.

I chose to submit with the 2nd approach as it worked better in high speed with steering and its derivative penalized. The settings of those weights above can be found in MPC.cpp: 40.

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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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

