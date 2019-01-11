# Reflection

## Model

### State

x = x-position of vehicle
y = y-position of vehicle
psi = heading of vehicle
v = velocity of vehicle
cte = cross-track error = distance from reference trajectory
epsi = psi error = distance from reference heading


### Actuators

steer_value = returned steering angle in radians, converted before sent to simulator
throttle_value = returned acceleration of vehicle for next timestep


### Update Equations

```
for (int t = 0; t < N; t++) {
  fg[0] += 100 * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
  fg[0] += 100 * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
  fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
}
```

For the reference cost functions, it was found that a hyperparameter of 100 to be sufficient for penalizing the vehicle for the error calculations. For the velocity, it was found that subtracting the reference velocity to be sufficient. These values were chosen arbitrarily from increments of 25 until the vehicle performed relatively well following the reference trajectory.

```
for (int t = 0; t < N - 1; t++) {
  fg[0] += 25 * CppAD::pow(vars[delta_start + t], 2);
  fg[0] += 25 * CppAD::pow(vars[a_start + t], 2);
}
```

Equations minimizing actuator usage for the next timestep saw a factor of just 25 as sufficient. This again was chosen arbitrarily and seemed to yield speeds close to the reference trajectory than when a larger hyperparameter (i.e. 50) was attached to it.

```
for (int t = 0; t < N - 2; t++) {
  fg[0] += 25000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  fg[0] += 500 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```

For minimizing the gap between different functions, this is where the hyperparameter tuning mattered the most. I found that the simulator would veer right immediately after beginning and by placing a large cost on the delta between current and next position, the vehicle would then have the necessary time to calculate the right values for staying close to the reference trajectory. For the acceleration minimization, there didn't need to be as large of a cost since the minimizing actuator functions above already force the vehicle to run faster/slower based on its value.


## Timestep Length (N) and Elapsed Duration (dt)

```
N = 10
dt = 0.1
```

The combination was chosen to track the vehicle 1 second in the future. The N=10 value is small enough to be computationally efficient and dt=0.1 is enough to duration.


## Polynomial Fitting and MPC Preprocessing

Polynomial fit used for this project was a 3rd degree polynomial. This was used since the turns for the majority of the track follow a 3rd degree polynomial format (particularly around the sharp turns).

Preprocessing done to the vehicle state is to rotate the reference angle by 90 degrees. This is made as a recommendation from the FAQ webinar to set the x and y coordinates to zero to simplify calculations. The rotated values are they calculated to account for the reading delay described in the section below.


## Model Predictive Control with Latency

This MPC model handles a 100 millisecond latency.

The model handles the delay in track readings by taking the `steer_value` and `throttle_value` from the json file and using them to calculate 1 timestep into the future from the origin. So the initial values for time t-1 is

```
double x0 = 0;
double y0 = 0;
double psi0 = 0;
double v = j[1]["speed"];
double cte = polyeval(coeffs, 0);
double epsi = psi - atan(coeffs[1] + (2 * coeffs[2] * px) + (3 * coeffs[3] * pow(px, 2)));
```

and the updated values at time t for the MPC to consume as state is

```
double x_delay = x0 + (v * cos(psi0) * delay);
double y_delay = y0 + (v * sin(psi0) * delay);
double psi_delay = psi0 - (v * throttle_value * delay / Lf);
double v_delay = v + steer_value * delay;
double cte_delay = cte + (v * sin(epsi) * delay);
double epsi_delay = epsi - (v * atan(coeffs[1]) * delay / Lf);
```
