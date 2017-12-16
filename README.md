# Udacity SDCND Project 10: Model Predictive Control
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)



The goal of this project was to use a advance Model Predictive Control to to drive a car around a track in the [term 2 simulator](https://github.com/udacity/self-driving-car-sim/releases). 

A Model Predictive Controller (MPC) uses a physical model of the vehicle to control it. For every time step the future trajectory over a certain horizon is predicted and current actuations issued. For the next time step the process is repeated.

In this project the simulator is providing waypoints around the track and the current vehicle state (position, direction and velocity). With this I was able to fit a polynomial on the waypoints and use an MPC to control the vehicle along this polynomial.

Project video: (Youtube Link)

[![Project track](https://github.com/stefancyliax/CarND-MPC-Project/raw/master/pic/MPC.gif)](https://www.youtube.com/watch?v=srA5Ejas3QQ)

## Approach

The following block diagram shows my approach. 

![MPC](https://github.com/stefancyliax/CarND-MPC-Project/raw/master/pic/MPC.png)

The simulator provides waypoints in map coordinates and the position, direction and velocity of the vehicle in map coordinates. With this is I was able to transform the waypoints to the vehicle coordinate system. 

Next I fitted a polynomial of degree three to the waypoints that describe the desired trajectory for the vehicle around the track.

Since we are now in the vehicle coordinate system, the vehicle position is x=0 and y=0. Additionally I calculated the cross track error (cte) and error in direction (epsi) and added them to the vehicle state.

Now the MPC was called with the vehicle state and the trajectory over the waypoints. The MPC now optimized the drive around the track and provided steering angle and throttle that were passed back to the simulator. 


## MPC

The MPC uses kinetic equations to predict the reaction of the vehicle to actuations in a certain horizon. I decided to use a horizon of 10 time steps with dt=0.1s. 
```cpp
size_t N = 10;
double dt = 0.1;
```

The state is described using equations that describe the relationship between different time steps.

```cpp 
AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);

// model equations
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```
Additionally constrains were given to limit the actuations. The steering angle is restricted to [-25°,25°] and the throttle value to [-1,1]. 

Since this is a multi-dimentional optimiation problem, we also have to define cost functions. 

```cpp
// The part of the cost based on the reference state.
for (int t = 0; t < N; t++)
{
  fg[0] += 100 * CppAD::pow(vars[cte_start + t], 4);
  fg[0] += 300 * CppAD::pow(vars[epsi_start + t], 4);
  fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
}

// Minimize the use of actuators.
for (int t = 0; t < N - 1; t++)
{
  fg[0] += 20000 * CppAD::pow(vars[delta_start + t], 2);
  fg[0] += CppAD::pow(vars[a_start + t], 2);
}

// Minimize the value gap between sequential actuations.
for (int t = 0; t < N - 2; t++)
{
  fg[0] += 600 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  fg[0] += 10 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```
The cost functions and parameters were chosen to as seen above. I decided to penalize deviations from CTE and direction using the cost function with the power of 4. This enabled me to don't penalize small derivation from the middle of the lane to much but still have a high penalty at the border of the lane. 

I also heavily penalized the use of the steering to enable the vehicle to drive on the inside of the turns and therefore reach a higher velocity around the track. 

To stabilize the vehicle, I also penalized the derivative of the steering usage. 

I set a reference speed of 120 mph and penalized deviations from it.

Finally I lightly penalized the use of the throttle as well as the derivative of the throttle. Since the vehicle has inertia in the simulator, these we not very important.


## Dealing with latency
TO improve the realism of the scenario we introduced a delay of 100ms before the sending of the new actuations to the simulator. Naturally this massively deteriorated the vehicle control. After the delay the predicted actuations just didn't match the current state of the vehicle. To give an example, after 100ms at 100 mph, the vehicle would be 4,5 meters further down the road. 

To handle this I added in another functional block that used the current vehicle state and current steering and throttle state to predict the vehicle state 100ms in the future. 
This was then assumed to be current state and used as described above. 

![MPC_delay](https://github.com/stefancyliax/CarND-MPC-Project/raw/master/pic/MPC_with_delay.png)


Added latency project video: (Youtube Link)

[![Project track](https://github.com/stefancyliax/CarND-MPC-Project/raw/master/pic/MPC_with_delay.gif)](https://www.youtube.com/watch?v=bn1JuwRimEY)
