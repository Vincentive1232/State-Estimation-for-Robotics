# State-Estimation-for-Robotics

## Batch Linear Gaussian Estimator for "Giant Glass of Milk Dataset"
### Task 
- Investigate a linear one-dimensional problem consisting of a robot driving back and forth in a straight line. 
- Estimated the position of the robot along this line throughout its ≈ 280m journey. 

### Dataset Acquisition
![Experiment_Setup](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/Batch_Linear-Gaussian_Estimator/exercise_2_plot/f3bab0bf39901312e9cc858d7acc672.png)

### Pipeline
- Use the batch linear-Gaussian estimator to fuse speed measurements coming from wheel odometry with range measurements coming from a laser rangefinder.
- Matlab was used.

### Results
- The comparison between the groundtruth and the estimated trajectory. ![Comparison](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A2_Final/exercise_2_plot/1.jpg)
- The error between the groundtruth and the estimated trajectory. ![Error](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A2_Final/exercise_2_plot/2.jpg)
- The uncertainty and uncertainty envelope of the results. ![Uncertainty](https://github.com/Vincentive1232/State-Estimation-for-Robotics/blob/master/SER_A2_Final/exercise_2_plot/3.jpg)

### Reference
> [State_Estimation_for_Robotics](http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser24.pdf)