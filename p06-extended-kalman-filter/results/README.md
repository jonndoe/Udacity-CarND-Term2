#### Udacity Self-Driving Car Engineer Nanodegree

# Term 2 / Project 1: Extended Kalman Filter

##### &nbsp;

## Goal
The goal of this project is to build a tool that can accurately predict the location of a vehicle based on its lidar and radar sensor measurements. Given that different sensors will return different measurements, and that neither of these measurements may accurately reflect the true position of the car, we need to reconcile or "fuse" the various sensor inputs into one accurate prediction. To do this, we will build an Extended Kalman Filter.

##### &nbsp;

### Initialize, Predict, Update
There are three main steps for programming a Kalman filter:

1. **Initialize** the Kalman filter variables.
1. **Predict** where our object is going to be after a time step Δt
1. **Update** where our object is based on sensor measurements. The 'predict' and 'update' steps then repeat themselves in a loop.

To measure how well our Kalman filter performs, we will then calculate the root mean squared error (RSME) comparing the Kalman filter results with the provided ground truth.

These three steps (initialize, predict, update) plus calculating the RMSE encapsulate the entire Extended Kalman Filter project.

##### &nbsp;

## Results
<a href="https://youtu.be/EJ-2sBC8pq0"><img src="output/video-thumbnail-v2.png" width="60%" /></a>

<img src="output/rmse.png" width="13%" /></a>

##### &nbsp;

## Implementation

### Files in the Github src Folder
The files we worked with are in the /src folder of the github repository.

- **main.cpp** &mdash; communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE

- **FusionEKF.cpp** &mdash; initializes the filter, calls the predict function, calls the update function

- **kalman_filter.cpp** &mdash; defines the predict function, the update function for lidar, and the update function for radar

- **tools.cpp** &mdash; function to calculate RMSE and the Jacobian matrix

##### &nbsp;

### How the Files Relate to Each Other
Here is a brief overview of what happens when you run the code files:

- `Main.cpp` reads in the data and sends a sensor measurement to `FusionEKF.cpp`
- `FusionEKF.cpp` takes the sensor data and initializes variables and updates variables. The Kalman filter equations are not in this file. `FusionEKF.cpp` has a variable called `ekf_`, which is an instance of a KalmanFilter class. The `ekf_` will hold the matrix and vector values. You will also use the `ekf_` instance to call the predict and update equations.
- The KalmanFilter class is defined in `kalman_filter.cpp` and `kalman_filter.h`

##### &nbsp;

### Scope & Steps
1. In `tools.cpp`, fill in the functions that calculate root mean squared error (RMSE) and the Jacobian matrix.
1. Fill in the code in `FusionEKF.cpp`. You'll need to initialize the Kalman Filter, prepare the Q and F matrices for the prediction step, and call the radar and lidar update functions.
1. In `kalman_filter.cpp`, fill out the `Predict()`, `Update()`, and `UpdateEKF()` functions.
1. Initialize the state vector.

   We need to initialize the state vector with the first sensor measurement.

   Although radar gives velocity data in the form of the range rate `ρ˙`, a radar measurement does not contain enough information to determine the state variable velocities `vx` and `vy`. We can, however, use the radar measurements `ρ` and `ϕ` to initialize the state variable locations `px` and `py`.

1. Calculate `y = z - H * x'`.

   For lidar measurements, the error equation is `y = z - H * x'`. For radar measurements, the functions that map the x vector `[px, py, vx, vy]` to polar coordinates are non-linear. Instead of using H to calculate `y = z - H * x'`, for radar measurements you'll have to use the equations that map from cartesian to polar coordinates: `y = z - h(x')`.

1. Normalize angles.

   In C++, `atan2()` returns values between -pi and pi. When calculating phi in `y = z - h(x)` for radar measurements, the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi. The Kalman filter is expecting small angle values between the range -pi and pi. When working in radians, we need to add 2π or subtract 2π until the angle is within the desired range.

1. Avoid divide by zero throughout the implementation.

    Before and while calculating the Jacobian matrix `Hj`, we need to make sure the code avoids dividing by zero. For example, both the `x` and `y` values might be zero or `px*px + py*py` might be close to zero.

1. No need to tune parameters.

    The R matrix values and Q noise values are provided for us. There is no need to tune these parameters for this project. In the next project, Unscented Kalman Filter, we'll discuss how to determine these parameters.

1. Test

   We need to analyze the output data and calculate the root-mean-square error (RMSE).
