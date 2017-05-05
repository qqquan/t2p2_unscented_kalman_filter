# Unscented Kalman Filter Project 
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt`




## Test Result

| Sample Name                                   | RMSE-px   | RMSE-py   | RMSE-vx  | RMSE-vy  |
|-----------------------------------------------|-----------|-----------|----------|----------|
| data/obj_pose-laser-radar-synthetic-input.txt | 0.0644685 | 0.0941484 | 0.289432 | 0.289684 |


Process noise standard deviation longitudinal acceleration is set `0.3` m/s^2.
Process noise standard deviation yaw acceleration is set `0.3` rad/s^2.
The following chart shows that the NIS shows a good fit around the 95% line.

![NIS](data/nis_plot.png)

