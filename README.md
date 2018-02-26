# Model Predictive Control(MPC)

A C++ implementation of Model Predictive Control(MPC)

[Demo video (YouTube)](https://www.youtube.com/watch?v=5syxpdcEfLA)

![](./images/image1.png) 
![](./images/image2.png)

## Overview

Model Predictive Control is a feedback control method to get a appropriate control input by solving optimization problem.

For autonomous vehicle, control input means steering wheel and throttle(and break pedal). Assuming we know reference trajectory, we predicte next N steps' waypoints according to kinematic model(for simplicity) over some pattern, and calculate cost function for each to find most appropriate predicted trajectory. Then we use the first control input of the predicted trajectory and throw away the other trajectory. That's it. We only need to repeat this.

Implementation code is in `MPC.cpp` and `main.cpp`

## Kinematic model equations

Kinematic model handles several status: car's x, y position, orientation, velocity, cross track error(= cte: distance between reference trajectory and actual trajectory), orientation error(= epsi: difference between desired orientation and current orientation). And belows are equations to calculate those at next timestep.

<a href="https://www.codecogs.com/eqnedit.php?latex=x_{t&plus;1}&space;=&space;x_{t}&space;&plus;&space;v_{t}&space;*&space;cos(\psi_{t})&space;*&space;dt" target="_blank"><img src="https://latex.codecogs.com/gif.latex?x_{t&plus;1}&space;=&space;x_{t}&space;&plus;&space;v_{t}&space;*&space;cos(\psi_{t})&space;*&space;dt" title="x_{t+1} = x_{t} + v_{t} * cos(\psi_{t}) * dt" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=y_{t&plus;1}&space;=&space;y_{t}&space;&plus;&space;v_{t}&space;*&space;sin(\psi_{t})&space;*&space;dt" target="_blank"><img src="https://latex.codecogs.com/gif.latex?y_{t&plus;1}&space;=&space;y_{t}&space;&plus;&space;v_{t}&space;*&space;sin(\psi_{t})&space;*&space;dt" title="y_{t+1} = y_{t} + v_{t} * sin(\psi_{t}) * dt" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\psi_{t&plus;1}&space;=&space;\psi_{t}&space;&plus;&space;\frac{v_{t}}{L_{f}}&space;*&space;\delta&space;*&space;dt" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\psi_{t&plus;1}&space;=&space;\psi_{t}&space;&plus;&space;\frac{v_{t}}{L_{f}}&space;*&space;\delta&space;*&space;dt" title="\psi_{t+1} = \psi_{t} + \frac{v_{t}}{L_{f}} * \delta * dt" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=v_{t&plus;1}&space;=&space;v_{t}&space;&plus;&space;a_{t}&space;*&space;dt" target="_blank"><img src="https://latex.codecogs.com/gif.latex?v_{t&plus;1}&space;=&space;v_{t}&space;&plus;&space;a_{t}&space;*&space;dt" title="v_{t+1} = v_{t} + a_{t} * dt" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=cte_{t&plus;1}&space;=&space;cte_{t}&space;&plus;&space;v_{t}&space;*&space;sin(e\psi_{t})&space;*&space;dt" target="_blank"><img src="https://latex.codecogs.com/gif.latex?cte_{t&plus;1}&space;=&space;cte_{t}&space;&plus;&space;v_{t}&space;*&space;sin(e\psi_{t})&space;*&space;dt" title="cte_{t+1} = cte_{t} + v_{t} * sin(e\psi_{t}) * dt" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=e\psi_{t&plus;1}&space;=&space;e\psi_{t}&space;&plus;&space;\frac{v_{t}}{L_{f}}&space;*&space;\delta_{t}&space;*&space;dt" target="_blank"><img src="https://latex.codecogs.com/gif.latex?e\psi_{t&plus;1}&space;=&space;e\psi_{t}&space;&plus;&space;\frac{v_{t}}{L_{f}}&space;*&space;\delta_{t}&space;*&space;dt" title="e\psi_{t+1} = e\psi_{t} + \frac{v_{t}}{L_{f}} * \delta_{t} * dt" /></a>
