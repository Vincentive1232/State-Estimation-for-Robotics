# State-Estimation-for-Robotics

## Iterative Extended Kalman Filter for "Lost in the Woods Dataset"

### Notice
1. folder [code_in_SE(3)](https://github.com/Vincentive1232/State-Estimation-for-Robotics/tree/master/SER_A4_Final/code_in_SE(3)) is the final version and the main file is "Assignment_4_GN_Batch.m"
2. folder [code_Non_Lie_Flavored](https://github.com/Vincentive1232/State-Estimation-for-Robotics/tree/master/SER_A4_Final/code_Non_Lie_Flavored) is the code for inspecting the data and checking our motion and measurement model. The main file is "Assignment_3_starry_night_lsqnonlin.m". This version is not related to our final results.

### Task
- Investigate a nonlinear three-dimensional problem consisting of a vehicle equipped with a stereo camera and inertial measurement unit (IMU) flying above a map of point features. 
- Estimated the position/orientation of this vehicle throughout its ≈ 44.3 m traverse. 

### Dataset Acquisition
![Experiment_Setup 1](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A3_Final/plots/Experiment_Setup1.png)
![Experiment_Setup 2](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A3_Final/plots/Experiment_Setup2.png)

### Pipeline
- Use the the batch Gauss-Newton method to fuse speed measurements from the IMU with point measurements from the stereo camera.

### Results
---
#### Estimation with "good" Initialization (Initialized with groundtruth state)
- Animation of the result trajectory together with an uncertainty ellipse around the robot. ![Animation (lidar_range = 5m)](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A3_Final/Result%20Trajectory.gif)
- The comparison between the groundtruth and the estimated trajectory. ![Comparison (lidar_range = 5m)](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A3_Final/plots/Path_5.jpg)
- The error between the groundtruth and the estimated trajectory in x. ![Error](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A3_Final/plots/x_error_5.jpg)
- The error between the groundtruth and the estimated trajectory in y. ![Error](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A3_Final/plots/y_error_5.jpg)
- The error between the groundtruth and the estimated trajectory in Theta. ![Error](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A3_Final/plots/Theta_error_5.jpg)

---
#### Estimation with "bad" Initialization (Initialized with x0 = [1; 1; 0.1] and P0 = diag([1, 1, 0.1]))
- The error between the groundtruth and the estimated trajectory in x. ![Error](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A3_Final/plots/x_error_bad_5.jpg)
- The error between the groundtruth and the estimated trajectory in y. ![Error](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A3_Final/plots/y_error_bad_5.jpg)
- The error between the groundtruth and the estimated trajectory in Theta. ![Error](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A3_Final/plots/Theta_error_bad_5.jpg)

---
#### Estimation with CRLB (Evaluating the Jacobians at the true state)
CRLB(Cramér-Rao Lower Bound) defines a theoretical lower bound for an unbias estimatior.
- The comparison between the groundtruth and the estimated trajectory. ![Comparison (lidar_range = 5m)](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A3_Final/plots/Trajectory_with_CRLB.png)
- The error between the groundtruth and the estimated trajectory in x(considering the CRLB). ![Error](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A3_Final/plots/x_error_CRLB_5.jpg)
- The error between the groundtruth and the estimated trajectory in y(considering the CRLB). ![Error](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A3_Final/plots/y_error_CRLB_5.jpg)
- The error between the groundtruth and the estimated trajectory in Theta(considering the CRLB). ![Error](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A3_Final/plots/Theta_error_CRLB_5.jpg)


### Reference
> [State_Estimation_for_Robotics](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser24.pdf)
