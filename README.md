# MPC Controller
This is an implementation of the MPC Controller using the non-linear CPPAd Solver.

# Setup
## Handling Latency
Latency is handled by assuming a simple kinematic model and simply predicting state 100ms into the future:
```
const double Lf = 2.67;
double latency = 0.1;
px = px + v * cos(psi) * latency;
py = py + v * sin(psi) * latency;
psi += v * steer / Lf * latency;
v += acc * latency;
```

Due to this simplistic model, and since our prediction is being done in car coordinates, we must make sure that acc and steer are as small as possible. Otherwise, the predicted state is too far off from reality during turns and the car goes bye bye.

## Cost function
Nothing spectacular here, except I penalize high steer and acc values due to the above. Infact they are penalized as much as CTE and EPSI: 

```
for (int t = 0; t < N; t++) {
  AD<double> cte = vars[cte_start + t];
  AD<double> epsi = vars[epsi_start + t];
  AD<double> v = vars[v_start + t];
  fg[0] += 500 * CppAD::pow(cte, 2) + 1000 * CppAD::pow(epsi, 2) + CppAD::pow(v - ref_v, 2);
}

for (int t = 0; t < N - 1; t++) {
  fg[0] += 500 * CppAD::pow(vars[delta_start + t], 2);
  fg[0] += 200 * CppAD::pow(vars[a_start + t], 2);
}
```

## Model setup
FG Eval is pretty similar to our simplistic linear model in the quiz, except I had to code in the third order polynomial:

```
AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0,2) + coeffs[3] * CppAD::pow(x0,3);
AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0,2));
```

## Future Thoughts
The MPC controller works though not as well as I would've hoped. There are some issues which I would like to address:

1. The predicted waypoints are in car coordinates. This would be fine, except sometimes (and maybe this is a simulator bug) the points appear in totally weird places on the actual map - usually when the car is turning. Wondering if this is something I can fix. This causes my speed, accel and steering to be kept low.

2. My solver goes nuts sometimes. I haven't been able to pin point this - but sometimes, even with a low cost some of the predicted points output by the solver are quite weird and totaly off. Really weird that this happens.

