# State-Estimation-for-Robotics

## Batch Gauss-Newton Estimator for "Starry Night Dataset"

### Notice
1. folder [code_in_SE(3)](https://github.com/Vincentive1232/State-Estimation-for-Robotics/tree/master/SER_A4_Final/code_in_SE(3)) is the final version and the main file is [Assignment_4_GN_Batch](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A4_Final/code_in_SE(3)/Assignment_4_GN_Batch.m).
2. folder [code_Non_Lie_Flavored](https://github.com/Vincentive1232/State-Estimation-for-Robotics/tree/master/SER_A4_Final/code_Non_Lie_Flavored) is the code for inspecting the data and checking our motion and measurement model. The main file is [Assignment_3_starry_night_lsqnonlin](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A4_Final/code_Non_Lie_Flavored/Assignment_3_starry_night_lsqnonlin.m). This version is not related to our final results.

### Task
- Investigate a nonlinear three-dimensional problem consisting of a vehicle equipped with a stereo camera and inertial measurement unit (IMU) flying above a map of point features. 
- Estimated the position/orientation of this vehicle throughout its ≈ 44.3 m traverse.

### Dataset Acquisition
![Experiment_Setup 1](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A4_Final/plots/Experiment_Setup2.png)
![Experiment_Setup 2](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A4_Final/plots/Experiment_Setup1.png)

### Pipeline
- Use the the batch Gauss-Newton method to fuse speed measurements from the IMU with point measurements from the stereo camera.
- As a comparison to batch results, apply different sliding window size, do small batch update and compare the errors.

### Results
---
#### Estimation Error (sliding window size = 10)
![Error Plot](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A4_Final/plots/Sliding_Window_10.png)

#### Estimation Error (sliding window size = 50)
![Error Plot](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A4_Final/plots/Sliding_Window_50.png)


### Reference
> [State_Estimation_for_Robotics](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser24.pdf)
