# Model Predictive Control (MPC) Project


[//]: # (Image References)

[image1]: ./output_images/simulator_01.jpg "Centre Driving"
[image2]: ./output_images/simulator_02.jpg "Recovery Driving"


## MPC Overview

This project implements Model Predictive Control (MPC) to drive a vehicle around a simulated track.

The following image shows the simulator operating:

* The Yellow line is a projection of a polynomial fitted to the simulator's centre line waypoints.
* The Green line is the output of the MPC.
* The Speed indicator shows 87.5mph (the target is 90mph).

| Centre Lane Driving |
| ------------------- |
| ![alt text][image1] |

The files [main.cpp](src/main.cpp), [MPC.h](src/MPC.h) and [MPC.cpp](src/MPC.cpp) contain the MPC controller logic based on the lessons for Term 2, Lesson 19: Model Predictive Control.


## MPC Model

The Vehicle Model used for the MPC Project is a simplified kinematic model (bycicle model) described by the Vehicle State.


### Vehicle State

The vehicle state is described with the following parameters:

| Parameter | Description          |
| --------- | -------------------- |
| `x`, `y`  | Vehicle position.    |
| `psi`     | Vehicle orientation. |
| `v`       | Vehicle velocity.    |
| `cte`     | Cross-track error (distance from current position to waypoint marked centre line). |
| `epsi`    | Orientation error (compared to waypoint marked centre line heading).               |


### Actuators

The vehicle state is described with the following parameters:

| Parameter | Description                                 |
| --------- | ------------------------------------------- |
| `delta`   | Steering angle (-25 degrees to 25 degrees). |
| `a`       | Vehicle acceleration (-1 to 1).             |


### Update Equation

The vehicle model update equations are included in [MPC.cpp (lines 91-97)](https://github.com/m-matthews/CarND-MPC-Project/blob/master/src/MPC.cpp#L91-L97).


### Cost Function Optimisation

The Cost Function is used to optimize the MPC.  This includes fundamental terms like the cross-track-error, and also others to control abrupt acceleration or steering controls.  The Cost Function is defined in [MPC.cpp (lines 38-55)](https://github.com/m-matthews/CarND-MPC-Project/blob/master/src/MPC.cpp#L38-L55).

The following table shows the order of parameter optimisation taken during the manual tuning.  Note that this was performed on the initial version of the code without latency and the reference velocity set to 50mph.

<table>
  <tbody>
    <tr>
      <th colspan="3">Reference Cost</th>
      <th colspan="2">Actuators</th>
      <th colspan="2">Sequential Actuators</th>
      <th rowspan="2">Comment</th>
    </tr>
    <tr>
      <th>cte</th>
      <th>epsi</th>
      <th>v</th>
      <th>delta</th>
      <th>a</th>
      <th>delta</th>
      <th>a</th>
    </tr>
    <tr>
      <td>1   </td><td>1  </td><td>1  </td><td>1  </td><td>1  </td><td>1  </td><td>1  </td><td>Vehicle reverses before bridge.</td>
    </tr>
    <tr>
      <td>1000</td><td>1  </td><td>1  </td><td>1  </td><td>1  </td><td>1  </td><td>1  </td><td>MPC flickers side to side badly.</td>
    </tr>
    <tr>
      <td>5000</td><td>1  </td><td>1  </td><td>1  </td><td>1  </td><td>1  </td><td>1  </td><td>MPC flickers side to side badly.</td>
    </tr>
    <tr>
      <td>1000</td><td>100</td><td>1  </td><td>1  </td><td>1  </td><td>1  </td><td>1  </td><td>Improved distance curve.</td>
    </tr>
    <tr>
      <td>1000</td><td>200</td><td>1  </td><td>1  </td><td>1  </td><td>1  </td><td>1  </td><td>Improved distance curve.</td>
    </tr>
    <tr>
      <td>1000</td><td>200</td><td>1  </td><td>1  </td><td>1  </td><td>100</td><td>1  </td><td>Attempt to stop ‘flickering tail’ at end of MPC projection.</td>
    </tr>
    <tr>
      <td>1000</td><td>200</td><td>1  </td><td>100</td><td>1  </td><td>100</td><td>1  </td><td>Attempt to stop ‘flickering tail’ at end of MPC projection.</td>
    </tr>
    <tr>
      <td>3000</td><td>200</td><td>1  </td><td>100</td><td>1  </td><td>100</td><td>1  </td><td>Attempt to stop ‘flickering tail’ produces excessive speed in parts.</td>
    </tr>
    <tr>
      <td>3000</td><td>200</td><td>100</td><td>100</td><td>1  </td><td>100</td><td>1  </td><td>Attempt to control speed results in constant brake application and errative MPC.</td>
    </tr>
    <tr>
      <td>3000</td><td>200</td><td>100</td><td>100</td><td>10 </td><td>100</td><td>1  </td><td>Use of brake controlled now.</td>
    </tr>
  </tbody>
</table>


## Timestep Length and Elapsed Duration

The values of `N` and `dt` are used to determine the count and timesteps for the model into the future from the current state.  `N` x `dt` = prediction length in seconds.

Considering the simulator track and the speed of the vehicle it would not make sense to attempt to predict far into the future.  The number of waypoints provided at each step are limited, and also the actions to be taken do not need awareness of more than the approaching or current corner in the track.

The following values were selected:

| Parameter | Description |
| --------- | ----------- |
| `dt`      | A value of 0.1s was selected as this resolution would match the resolution required with the implementation of the `latency` to be introduced in later development. |
| `N`       | The number of predications was subject to experiment.  An initial selection of 8 didn't hold well to the polynomial, 16 was jittery, 12 better than 8 but still a bit jittery.  The final choice was 10 selected which when multiplied by the `dt` value predicts one second into the future. |


## Polynomial Fitting

The polynomial fitting process involved converting the provided waypoints (`ptsx` and `ptsy`) into the Vehicle's Coordinate system.  The conversion code is included in [main.cpp (lines 113-125)](https://github.com/m-matthews/CarND-MPC-Project/blob/master/src/main.cpp#L113-L125).

The resulting polynomial is projected back into the simulator as a Yellow line in [main.cpp (lines 163-175)](https://github.com/m-matthews/CarND-MPC-Project/blob/master/src/main.cpp#L163-L175)

| Fitted Polynomial as Yellow Line |
| -------------------------------- |
| ![alt text][image1]              |


## Latency

The implementation of latency was included in [commit 529d39779b203439069cb381d8c0a6eb5b6662b8](https://github.com/m-matthews/CarND-MPC-Project/commit/529d39779b203439069cb381d8c0a6eb5b6662b8).

The initial code without latency assumed the vehicle state was zero (ie: `x`=0.0, `y`=0.0, `psi`=0.0) in the vehicle coordinate system.

The inclusion of latency involved trying to predict where the vehicle would be after the latency period had elapsed.

The code below from [main.cpp (lines 127-131)](https://github.com/m-matthews/CarND-MPC-Project/blob/master/src/main.cpp#L127-L131) includes the following steps:

- Convert `v` into the same units as other items in the model.
- Determine the new `psi` angle based on current steering angle.
- Update `px` and `py` based on the velocity `v` and angle `psi`.
- Note that the velocity `v` is not updated for acceleration, as the simulator does not provide a suitable value and the change in velocity over 100ms is unlikely to impact the results.

```c++
  // State non-zero as it includes latency values.
  v *= MPC::mph_to_mps; // mph -> m/s
  psi = -v*delta/MPC::Lf*latency_s;
  px = v*cos(psi)*latency_s;
  py = v*sin(psi)*latency_s;
```

Note that once latency is performed, the first cell of the MPC output is no longer a suitable value to use.  Instead the average of 3 predictions are used as shown in [MPC.cpp (lines 223-233)](https://github.com/m-matthews/CarND-MPC-Project/blob/master/src/MPC.cpp#L223-L233),


## Speed Selection

The code was initially developed with a reference velocity of 50mph (without latency).

Once the latency code was developed, the code was trialled at 100mph.  The MPC successfully completed the track, however it would 'touch' the edge of the road on some laps, so the final submission was at 90mph which completes the laps cleanly.

| Centre Lane Driving | High Speed Recovery |
| ------------------- | ------------------- |
| ![alt text][image1] | ![alt text][image2] |

The images above show the vehicle in normal driving conditions, and also in more extreme conditions where it successfully optimises to stay on the road while keeping the steering angle and speed within the limits specified in the cost function.
