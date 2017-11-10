# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---



## Dependencies

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

### Attribution
The infrastructure code in this project is supplied from project repository listed above.  Much of the model code was taken from the lesson quiz [Mind the Line](https://github.com/udacity/CarND-MPC-Quizzes).  As well as from the class [Walkthrough](https://youtu.be/bOQuhpz3YfU)


## Project Instructions and Rubric

### Your code should compile.

Code compiles and runs with the standard instructions.

### Notes on value names

px - position x (of the vehicle)
py - position y (of the vehicle)
psi - heading of the vehicle
v - velocity
cte - cross track error
epsi - heading error

NOTE - In this write up I do not distinguish between map and vehicle coordinates.  Most processing is done in vehicle coordinates.

### The Model

#### Model State

The model state is represented by a vector of {px, py, psi, v, cte, epsi}.  The state is taken from data sent by the simulator.  The data is transformed
into a vehicle centric coordinate space such that {px, py, psi} are 0 after
the transformation.  This is essentially required since the simulator expects
this transformation for the returned plotted center line and vehicle path projection.

The simulator also supplies way points for the center of the road(desired path).  These waypoints are fitted to a 3rd order polynomial.  The polynomial
parameters are used to compute cte and epsi error values.

#### Model Equations

The behavior of the vehicle is modeled with the following kinematic equations:

      x1 = (x0 + v0 * CppAD::cos(psi0) * dt);
      y1 = (y0 + v0 * CppAD::sin(psi0) * dt);
      psi1 = (psi0 + v0 * delta0 / Lf * dt);
      v1 = (v0 + a0 * dt);
      cte1 = ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);
      epsi1 = ((psi0 - psides0) + v0 * delta0 / Lf * dt);

Where f0 is the result of a function that uses the derived coefficients and y0 to provide a distance from the desired path.

In main.cpp in order to provide an estimated state at 100ms into the future, a 
simplified version of the above equations are used.  Simplified due to the fact that x, y and psi are 0 at that point after the transformation.

#### Model Cost

I used the cost model from the MPC quiz and the walkthrough.  It sets
cost values for error states(cte, epsi) and velocity.  With very high 
cost associated with the error functions.  It also sets contraints on the 
actuators for both minimizations of use and sequential state to state changes.

Given the weights on the minimization of use costs, they likely could be 
dropped with little effect.

```
    // The part of the cost bsed on the reference state
    for (int t = 0; t < N; t++) {
      fg[0] += w_cte * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
      fg[0] += w_epsi * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
      fg[0] += w_ref_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators
    for (int t = 0; t < N - 1; t++) {
      fg[0] += w_delta * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += w_a * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations
    for (int t = 0; t < N - 2; t++) {
      fg[0] += w_delta_diff * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += w_a_diff * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```

The values I ended up using are listed below.  Discussion of these values is below.

```
// Goal states for cte, epsi and velocity
double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 120;

// Very important to constrain errors
const int w_cte = 2500;
const int w_epsi = 2500;
const double w_ref_v = 1.0;

// Not important to constrain actuators
const int w_delta = 1;
const int w_a = 1;

// Important to minimize the gap between sequential actuators to reduce jerk
const int w_delta_diff = 200;
const int w_a_diff = 5;
```

Even at fairly low target speeds the cte and epsi costs need to be fairly high
in order to get the model to perform and the car to stay on track.  I also
tried lower weights for cte and epsi with changing the power to the 3rd and 
4th power.   It frequently resulted in an unstable vehicle path.

I chose a fairly high ref_v value the penalizes distance from the target speed
of 120.  At this target speed the model is extremely sensitive to the weight
applied to that cost(w_ref_v=1.0).  At w_rev_v=2.0 the model crashes quite quickly in certain turns, at 1.5 it would probably pass, but occasionally crashes multiple laps in.

At this speed, the model is also very sensitive to w_a_diff.  5 appears to be 
the maximum that it will successfully complete.  This is a somewhat limiting 
factor in getting the car up to its target speed.  It would be nice to be able
to separate the acceleration into positive acceleration costs and negative
acceleration costs.

### Timestep Length and Elsapsed Duration(N & dt)

All of my experiments maintained a ratio of 100:1 of N to dt.  My final values were N = 10 and dt = .1.  I also tried dt values of .8, .12 and .14(with N = dt * 100).  This ratio has no actual meaning and these values likely should not have been changed in concert while testing.

The values of dt=.8 and N=8 did appear to give some instability when tested with the latency included.  No improvement was noted for dt of .12 and .14 so I
defaulted to the .1 value.

### Polynomial Fitting and MPC Preprocessing

A polynomial was fitted to the waypoints.  I generated an additional set of values from the polynomial for return in the next_x and next_y json values to give more points to plot rather than returning transformed ptsx and ptsy values
from the input telemetry.

The waypoints returned are not pre-processed.  However, the coefficients are derived from the pre-processed telemetry waypoints.  The processing is a rotation and translation to bring the points into a vehicle coordinate system
with the first point having x, y and psi values of 0.

###  Model Predictive Control with Latency

The model predictive control is implemented with the kinematic equations listed
above.  The 100ms latency is dealt with by projecting the state of the vehicle 
100ms into the future and using that state as input to the MPC unit so that when the control parameters are used after the 100ms delay the are the correct
actions.

I did not make an estimate on the velocity parameter since the acceleration was
not available from the simulator.  I did try using the throttle position as a proxy for the acceleration as was mentioned in the project walk through.
This generated significant instability in the car in low speed times when coming up to the target speed and during corners.  The model's stability significantly improved when I used the prior velocity.  The prior velocity is a
reasonable proxy since it does not change much in the change in time between steps.

### Simulation

With the submitted parameters the vehicle successfully navigates the track.  At
the current settings I believe it also does not touch the red/white turn markers.



