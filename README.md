# Project: Model Predictive Control (MPC)
## Overview
This project is about using [Model Predictive Control (MPC)](https://en.wikipedia.org/wiki/Model_predictive_control) for controlling car steering angle & throttle.

![Intro](./images/intro.gif)


## How Does It Work?

The MPC reframes the task of following a trajectory to an optimization problem by finding the best combination of steering angle & throttle values . The MPC is fed the initial state of the car [position, orientation, velocity, cross track error, orientation error]. Using the [Kinematic](https://en.wikipedia.org/wiki/Kinematics) vehicle model, actuator limitations and a cost function, the MPC then predicts the best possible actuator inputs (steering & throttle) by finding the lowest cost trajectory. 

## Rubric Points

### The Model

The MPC is based on the global Kinematic Model that is a simple real-time model easily applicable to different types of vehicle and accuracy wise is as good as its complex counterparts. It uses the current state of the car [position, orientation, velocity, cross track error, orientation error] and the actuator values [steering angle, throttle] for making future state & actuator predictions. Below are the equations that define the model:

![Model equations](./images/model.png)

Following steps illustrate the workings of the MPC:

1. Transform waypoints (provided by the simulator) from global co-ordinate system to vehicle co-ordinate system.
2. Fit a polynomial of order 3 to the transformed waypoints.
3. Calculate initial cross track & orientation errors.
4. Get the state after 100 ms & use the calculated state as current state to account for actuator latency.
5. Call the MPC solver.
6. Get the computed actuator values & the MPC predicted trajectory.  


### Timestep Length and Elapsed Duration (N & dt)

In MPC, the `T` is the *prediction horizon* in seconds, which is a product of *no. of timesteps* `N` and the *duration between each timestep* `dt`. Decreasing *dt* increases the frequency of actuation (the number of times steering & throttle is adjusted), which although results in a smoother trajectory e.g. in case of making a left or right turn, the computational cost increases and vice-versa. In case of *N*, an increase results in a large horizon meaning that the predictions are made for a longer time period. Setting a higher number means that it not only becomes computationally taxing but also somewhat a waste of computational resources as the car might not even maintain that trajectory due to changing environmental factors. Also depending upon *dt*, looking that far ahead might not be helpful if the trajectory is recomputed & gets changed with each timestep.

I tried different combinations of *N* & *dt* and in the end settled for values of `N=10` and `dt=0.1`. The table below shows the results of some of other combination of values:

| N | dt | Description   		| 
|:---:|:---:|:-------------------------------| 			
| 10 | 0.05 |Ok but very sharp turns & steering turning erratically even on straight road.|
| 20 | 0.05 |Crash straightaway.|
| 30 | 0.05 |Crash straightaway.|
| 5 | 0.05 |Worked for a bit then crashed. MPC trajectory pointing in totally different direction.|
| 5 | 0.1 |Worked for a bit then crashed. MPC trajectory pointing in totally different direction.|
| 20 | 0.1 |OK but it seems that on some occasions the MPC trajectory & the reference trajectory are not drawn properly & the car takes more extremes turns. Also this is more computationally taxing as the prediction horizon is large. Also, there’s no point in calculating points that are too much in future as they are simply thrown away with each successive call to the solver.|

Another good reason for using a value of `0.1` for *dt* was that it synchronizes perfectly with the introduced latency of 100 ms (0.1 s).

### Polynomial Fitting and MPC Preprocessing

A polynomial of degree 3 was fitted to the waypoints provided by the simulator in order to get the required *coefficients* for the *solver* & to be able to draw the reference trajectory (yellow line). The *coefficients* are further used to calculate the initial *cross track error* and the *orientation error*. However, before the waypoints can be used as in input for polynomial fitting, they need to be transformed from global (map) co-ordinate system to vehicle co-ordinate system as shown in the following code excerpt:  

```c++

double steer_value = j[1]["steering_angle"]; 
double throttle_value = j[1]["throttle"]; 

/*************************************************************
* Step 1: Transform waypoints (px,py) from global co-ordinate system to vehicle co-ordinate system 
*************************************************************/
// Make x,y co-ordinates 0
for (unsigned int i = 0; i < ptsx.size(); i++) {
double transformed_x = ptsx[i] - px;
double transformed_y = ptsy[i] - py;

// Rotate points
ptsx[i] = transformed_x*cos(-psi) - transformed_y*sin(-psi);
ptsy[i] = transformed_x*sin(-psi) + transformed_y*cos(-psi);
}

double* ptr_x = &ptsx[0]; // pointer at first x position
double* ptr_y = &ptsy[0]; // pointer at first y position
Eigen::Map<Eigen::VectorXd> waypoints_x(ptr_x, 6);
Eigen::Map<Eigen::VectorXd> waypoints_y(ptr_y, 6);

/*************************************************************
* Step 2: Fit a polynomial to the transformed waypoints (order 3 as we have 6 points)
*************************************************************/
auto coeffs = polyfit(waypoints_x, waypoints_y, 3);

/*************************************************************
* Step 3: Calculate initial CTE & Orientation error
*************************************************************/
// Using 0 because co-ordinates have been shifted so car's x,y & psi are all 0.
// Returned value is the y, which is basically our CTE
double cte = polyeval(coeffs, 0); 

// Actual equation --> psi - atan(coeffs[1])
// but as psi is 0, so only left with -atan(coeffs[1])
double epsi = -atan(coeffs[1]); 
```

### Model Predictive Control with Latency

To simulate actuator control latency, a latency of 100 millisecond is added. In order to handle this latency, I used the Kinematic model equations to calculate the future state after `0.1 seconds` (100 milliseconds) and used that state to calculate the input control vector (predicted trajectory). Doing so enables to compensate for the latency as the state that solver is using is in fact the one where the car would be after the latency of 100 ms.

```c++

// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67; 
// 0.1 = 100 ms latency
const double x_future = v * 0.1;
const double y_future = 0.0;
const double psi_future = - v/Lf * steer_value * 0.1;         
const double v_future = v + throttle_value * 0.1;
const double cte_future = cte + v * sin(epsi) * 0.1;
const double epsi_future = epsi - v/Lf * steer_value * 0.1;

Eigen::VectorXd state(6);
state << x_future, y_future, psi_future, v_future, cte_future, epsi_future;
```

### Model Coefficients Fine-tuning 

To choose the best coefficients for the Kinematic model, I further employed a trial-and-error approach. I used different combinations of coefficient values for *CTE, Orientation Error, Velocity, Steering, Throttle, Steering Delta (change rate) & Throttle Delta (change rate)* and drew resulting graphs with the aim of getting a smooth ride with as less erratic actuations as possible. Based on the graphs, I settled for configuration no. 8. 

|Config. No.|CTE|Orientation Error|Velocity|Steering|Throttle|Steering Delta|Throttle Delta|Result| 
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|  			
|1|2000|2000|1|5|5|200|10|OK|

![Config 1](./images/result_0.png)
---
|Config. No.|CTE|Orientation Error|Velocity|Steering|Throttle|Steering Delta|Throttle Delta|Result| 
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|  			
|2|3000|2000|500|100|100|500|100|Crash|

![Config 2](./images/result_1.png)
---
|Config. No.|CTE|Orientation Error|Velocity|Steering|Throttle|Steering Delta|Throttle Delta|Result| 
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|  			
|3|2000|2000|5|20|10|300|20|OK|

![Config 3](./images/result_2.png)
---
|Config. No.|CTE|Orientation Error|Velocity|Steering|Throttle|Steering Delta|Throttle Delta|Result| 
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|  			
|4|2000|2000|1|5|5|500|250|OK|

![Config 4](./images/result_3.png)
---
|Config. No.|CTE|Orientation Error|Velocity|Steering|Throttle|Steering Delta|Throttle Delta|Result| 
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|  			
|5|2000|2000|1|5|5|100|500|OK|

![Config 5](./images/result_4.png)
---
|Config. No.|CTE|Orientation Error|Velocity|Steering|Throttle|Steering Delta|Throttle Delta|Result| 
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|  			
|6|2000|2000|1|5|5|2000|1000|OK|

![Config 6](./images/result_5.png)
---
|Config. No.|CTE|Orientation Error|Velocity|Steering|Throttle|Steering Delta|Throttle Delta|Result| 
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|  			
|7|4000|2000|1|5|5|2000|1000|OK|

![Config 7](./images/result_6.png)
---
|Config. No.|CTE|Orientation Error|Velocity|Steering|Throttle|Steering Delta|Throttle Delta|Result| 
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|  			
|8|2000|2000|1|5|5|3000|2000|OK|

![Config 8](./images/result_7.png)
---
|Config. No.|CTE|Orientation Error|Velocity|Steering|Throttle|Steering Delta|Throttle Delta|Result| 
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|  			
|9|2000|2000|1|5|5|3500|3500|OK|

![Config 9](./images/result_8.png)

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

## Usage

Follow the build instructions above. Once the program is running, start the simulator. You should see a *connected!!!* message upon successful connection between the simulator and the c++ program. Hit the *Start button*. 

## Directory Structure

* **images:** Directory containing writeup images
* **src:** Directory containing c++ source files
* **CMakeLists.txt:** File containing compilation instructions
* **README.md:** Project readme file
* **install-mac.sh:** Script for installing uWebSockets on Macintosh
* **install-ubuntu.sh:** Script for installing uWebSockets on Ubuntu
* **install_Ipopt_CppAD.md:** Installation instructions for Ipopt & CppAD libraries
* **install_ipopt.sh:** Script for installing ipopt


## License

The content of this project is licensed under the [Creative Commons Attribution 3.0 license](https://creativecommons.org/licenses/by/3.0/us/deed.en_US).