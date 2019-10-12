# CarND-PID-Controls-Project

This repository contains C++ code for implementation of PID controller, to derive steering angles for a car to drive around a circular track. This task was implemented to partially fulfill Term-II goals of Udacity's self driving car Nano degree program.

---
## Working of PID Controller

The basic principle of working of a PID controller is to satisfy a boundary value problem. Most common examples of such problems are either minimization of the total error in the system or maximization of the total gain in the system. These error or gain problems are represented mathematically and are used to govern the effect to P, I and D components of a PID controller. These components are described below:
1. Proportional (P) component: Mathematically, the P component establishes linear relationship with the problem. The effect of P component is in proportional to the value of problem at the current time step. For e.g.:, if the problem is to minimize the error in the system at current time step, the value of P component is given by the formula:
`-Kp * error`
2. Integral (D) component: Mathematically, the I component establishes linear relationship between the average value of problem over time. The effect of I component is in proportional to the average value of problem from the beginning of time to the current time step. For e.g., if the problem is to minimize the error in the system, the value of I component is given by the formula:
`-Ki * ∑ error`
3. Differential (D) component: Mathematically, the D component establishes linear relationship with the rate of change of problem. The effect of D component is in proportional to the rate of change problem from last time step to current time step. For e.g.:, if the problem is to minimize the uncertainty in the system, the value of D component is given by the formula:
`-Kd * d(error)/dt`

When all components are used, the mathematical equation is given by:

`α = (-Kp * error) + (-Kd * d(error)/dt) + (Ki * ∑ error)`

where, α is the control input to the system, often known as the actuator input.

## Project Goal

In this project, a PID controller was implemented to drive a car around circular track having sharp left and right turns. A good solution would help the car stay in the center portion of the lane and take smooth left and right turns without touching or running over the edges of the lane (considered as risky in case humans were travelling in such a car). 

## Project Implementation

Simulation of a circular track was achieved in the Udacity's self driving car simulator. The simulator measured the cross track error (cte) between the lateral position of car and the center of the lane. This error was communicated to C++ code with the help of uWebSockets library. The cte was then used to calculate P, I and D components of the controller governing the steering angle of the car.

The final implementation consisted of following major steps:

1. Cross track error (cte) recorded by the simulator was used to develop the mathematical equation for calculating steering angle of the car. 

2. The major task in this implementation was to tune the Kp, Ki and Kd gain parameters. This was done using manual tuning in the beginning and followed by fine tuning by using Twiddle or Gradient Descent. This process is listed in the following steps.

3. In the initial step, the I and D components were switched off and only the P component was used to drive the car. This was done by setting the Ki and Kd parameters to 0. The car was allowed to drive along the track. The value of Kp was tuned manually to help the car stay on the track for at least 20% of the total track length, as long as it doesn't cross the edge of the track. 

4. In this step, a PD controller was used. The I component was still switched off by setting the value of Ki to zero. The value of Kp was as tuned from step 3 and the value of Kd was tuned manually to keep the car on track for most of the length of the track.

5. Due to some kind of systemic bias and uncertainty in the system, the car would often drift to the edges of the lane. This undesired behavior was corrected by introducing the I component. The value of Kp and Kd were as tuned from step 3 and step 4 respectively. The value of Ki was tuned manually to restrict the car drifting away from center of the lane.

6. Gain parameters were then fine-tuned by using vanilla Gradient Descent algorithm, also known as Twiddle. In this process, manual tuned values from step 3, 4 and 5 were taken as a starting point. Each of the gain parameters Kp, Ki and Kd were tuned one at a time. For e.g., Ki and Kd were kept constant during the fine tuning of Kp and the overall error in the system was minimized.

## Project Output

PID controller used to derive the steering angles for a car moving on a circular track was implemented successfully. The car could stay close to the center of the lane and take smooth left and right turns along its path.

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 



