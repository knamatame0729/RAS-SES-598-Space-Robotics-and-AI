# First-Order Boustrophedon Navigator
![image](https://github.com/user-attachments/assets/940fc6bc-fcee-4d11-8bc8-d53a650aaf80)

In this assignment, you will understand the provided code in ROS2 with Turtlesim, and refactor and/or tune the navigator to implement a precise lawnmower survey (a boustrophedon pattern). The current code will do a pattern shown above, which is not a uniform lawnmower survey. 
Explore literature on how lawnmower surveys typically look, and modify the code to meet the requirements for a uniform survey. 

## Background
Boustrophedon patterns (from Greek: "ox-turning", like an ox drawing a plow) are fundamental coverage survey trajectories useful in space exploration and Earth observation. These patterns are useful for:

- **Space Exploration**: Rovers could use boustrophedon patterns to systematically survey areas of interest, ensuring complete coverage when searching for geological samples or mapping terrain. However, due to energy constraints, informative paths are usually optimized, and this results in paths that are sparser than complete coverage sampling, and may still produce high-accuracy reconstructions. 
  
- **Earth Observation**: Aerial vehicles employ these patterns for:
  - Agricultural monitoring and precision farming
  - Search and rescue operations
  - Environmental mapping and monitoring
  - Geological or archaeological surveys
  
- **Ocean Exploration**: Autonomous underwater vehicles (AUVs) use boustrophedon patterns to:
  - Map the ocean floor
  - Search for shipwrecks or aircraft debris
  - Monitor marine ecosystems
  
The efficiency and accuracy of these surveys depend heavily on the robot's ability to follow the prescribed path with minimal deviation (cross-track error). This assignment simulates these real-world challenges in a 2D environment using a first-order dynamical system (the turtlesim robot).

## Objective
Tune a PD controller to make a first-order system execute the most precise boustrophedon pattern possible. The goal is to minimize the cross-track error while maintaining smooth motion.

## Learning Outcomes
- Understanding PD control parameters and their effects on first-order systems
- Practical experience with controller tuning
- Analysis of trajectory tracking performance
- ROS2 visualization and debugging

## Prerequisites

### System Requirements
Choose one of the following combinations:
- Ubuntu 22.04 + ROS2 Humble
- Ubuntu 23.04 + ROS2 Iron
- Ubuntu 23.10 + ROS2 Iron
- Ubuntu 24.04 + ROS2 Jazzy

### Required Packages
```bash
sudo apt install ros-$ROS_DISTRO-turtlesim
sudo apt install ros-$ROS_DISTRO-rqt*
```

### Python Dependencies
```bash
pip3 install numpy matplotlib
```

## The Challenge

### 1. Controller Tuning (60 points)
Use rqt_reconfigure to tune the following PD controller parameters in real-time:
```python
# Controller parameters to tune
self.Kp_linear = 1.0   # Proportional gain for linear velocity
self.Kd_linear = 0.1   # Derivative gain for linear velocity
self.Kp_angular = 1.0  # Proportional gain for angular velocity
self.Kd_angular = 0.1  # Derivative gain for angular velocity
```

Performance Metrics:
- Average cross-track error (25 points)
- Maximum cross-track error (15 points)
- Smoothness of motion (10 points)
- Cornering performance (10 points)

### 2. Pattern Parameters (20 points)
Optimize the boustrophedon pattern parameters:
```python
# Pattern parameters to tune
self.spacing = 1.0     # Spacing between lines
```
- Coverage efficiency (10 points)
- Pattern completeness (10 points)

### 3. Analysis and Documentation (20 points)
Provide a detailed analysis of your tuning process:
- Methodology used for tuning
- Performance plots and metrics
- Challenges encountered and solutions
- Comparison of different parameter sets

## Getting Started

### Repository Setup
1. Fork the course repository:
   - Visit: https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI
   - Click "Fork" in the top-right corner
   - Select your GitHub account as the destination

2. Clone your fork (outside of ros2_ws):
```bash
cd ~/
git clone https://github.com/YOUR_USERNAME/RAS-SES-598-Space-Robotics-and-AI.git
```

3. Create a symlink to the assignment in your ROS2 workspace:
```bash
cd ~/ros2_ws/src
ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/first_order_boustrophedon_navigator .
```

### Building and Running
1. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select first_order_boustrophedon_navigator
source install/setup.bash
```

2. Launch the demo:
```bash
ros2 launch first_order_boustrophedon_navigator boustrophedon.launch.py
```

3. Monitor performance:
```bash
# View cross-track error as a number
ros2 topic echo /cross_track_error

# Or view detailed statistics in the launch terminal
```

4. Visualize trajectory and performance:
```bash
ros2 run rqt_plot rqt_plot
```
Add these topics:
- /turtle1/pose/x
- /turtle1/pose/y
- /turtle1/cmd_vel/linear/x
- /turtle1/cmd_vel/angular/z
- /cross_track_error

## Evaluation Criteria

1. Controller Performance (60%)
   - Average cross-track error < 0.2 units (25%)
   - Maximum cross-track error < 0.5 units (15%)
   - Smooth velocity profiles (10%)
   - Clean cornering behavior (10%)

2. Pattern Quality (20%)
   - Even spacing between lines
   - Complete coverage of target area
   - Efficient use of space

3. Documentation (20%)
   - Clear explanation of tuning process
   - Well-presented performance metrics
   - Thoughtful analysis of results

## Submission Requirements

1. GitHub Repository:
   - Commit messages should be descriptive

2. Documentation in Repository:
   - Update the README.md in your fork with:
     - Final parameter values with justification
     - Performance metrics and analysis
     - Plots showing:
       - Cross-track error over time
       - Trajectory plot
       - Velocity profiles
     - Discussion of tuning methodology
     - Challenges and solutions

3. Submit your work:
   - Submit the URL of your GitHub repository
   - Ensure your repository is public
   - Final commit should be before the deadline

## Tips for Success
- Start with low gains and increase gradually
- Test one parameter at a time
- Pay attention to both straight-line tracking and cornering
- Use rqt_plot to visualize performance in real-time
- Consider the trade-off between speed and accuracy

## Grading Rubric
- Perfect tracking (cross-track error < 0.2 units): 100%
- Good tracking (cross-track error < 0.5 units): 90%
- Acceptable tracking (cross-track error < 0.8 units): 80%
- Poor tracking (cross-track error > 0.8 units): 60% or lower

Note: Final grade will also consider documentation quality and analysis depth.

## Extra Credit (10 points)
Create and implement a custom ROS2 message type to publish detailed performance metrics:
- Define a custom message type with fields for:
  - Cross-track error
  - Current velocity
  - Distance to next waypoint
  - Completion percentage
  - Other relevant metrics
- Implement the message publisher in your node
- Document the message structure and usage

This will demonstrate understanding of:
- ROS2 message definitions
- Custom interface creation
- Message publishing patterns 

# Assignment Discussion
## 1. Tuning Methodology
- Step 1. Set the parameters as initial values and observe performance
- Step 2. Increase the both Kp values to make quick responds
- Step 3. Once the trajectory doesn't have big overshoot, decreaselly adjust both Kp_linear and Kp_angular values to smooth cornering
- Step 4. Adjust both Kd_linear and Kd_angular to make both conering and linear trajectory smoother
- Step 5. After all these steps, slightly adjust both Kp and Kd to optimize the performance under the required performance

### Final Parameter Values
```python
# Tuned parameters
self.Kp_linear = 11.0  # Proportional gain for linear velocity
self.Kd_linear = 0.105  # Derivative gain for linear velocity
self.Kp_angular = 7.9  # Proportional gain for angular velocity
self.Kd_angular = 0.0  # Derivative gain for angular velocity
self.spacing = 0.4    # Spacing between lines
```

## 2. Performance metrics and plots
- Cross-track cross-track error
  - Average : 0.117
  - Maximum : 0.239
- Cross-track-error 
   ![Image](https://github.com/user-attachments/assets/5480de43-bb33-46e4-bbdb-c1fc1a2204cf)

- Trajectory plot

   ![image](https://github.com/user-attachments/assets/cfcbb094-1321-4adf-87f3-5d1d51ee03f6)

- Velocity profiles
   - Linear Velocity x
   ![Image](https://github.com/user-attachments/assets/763a8cca-fdcf-461b-b7cf-87c0233613f2)
   - Angular Velocity z
   ![Image](https://github.com/user-attachments/assets/ad2e0a88-e87a-44df-90d0-5eb205b92f29)

## 3. Challenges and solutions
| Challenges                                       | Solutions                                                            | 
| :-----------------------------------------------:|:--------------------------------------------------------------------:|
|  - Error is not below target values (Overshoot)  |  - Increace the Kp_linear and Kp_angular values to reduce overshoot  |
|  - Vibration during cornering                    |  - Decreace the Kp_linear and Kp_angular to make smoother cornering  | 
|  - Uneven spacing between lines                  |  - Slightly adjust Kd_linear and Kd_angular                          |
|  - Efficiently complete coverage of target area  |  - Make spacing between lines closer but not too close to avoid uneven space between lines and as a result  |

## 4. Comparison of different parameter sets
|Kp_linear|Kd_linear|Kp_angular|Kd_angular|Average Error|Max Error|Smoothness socre |Total score|
|:-------:|:-------:|:--------:|:--------:|:---:|:--------:|:---:|:-:|
|40|0.1|40|0.1|N/A|N/A|0/10|0/10|
|30|0.1|30|0.1|N/A|N/A|0/10|0/10|
|20|0.1|20|0.1|N/A|N/A|0/10|0/10|
|10|0.1|10|0.1|0.066|0.149|4/10|6/10|
|5|0.1|5|0.1|0.271|0.533|9/10|4/10|
|7|0.1|7|0.1|0.159|0.321|10/10|6/10|
|9|0.1|9|0.1|0.074|0.208|7/10|6/10|
|9|0.01|9|0.01|0.083|0.181|9/10|7/10|
|9|0.05|9|0.01|0.072|0.181|8/10|7/10|
|9|0.05|9|0.001|0.073|0.181|8/10|7/10|
|9|0.05|8|0.001|0.103|0.234|8/10|7.5/10|
|9|0.05|7|0.001|0.127|0.300|8.5/10|6.5/10|
|10|0.05|7|0.001|0.127|0.300|8/10|7/10|
|10|0.05|8|0.001|0.110|0.234|8/10|8/10|
|11|0.05|8|0.001|0.114|0.234|9/10|9/10|
|11|0.1|8|0.001|0.114|0.234|9/10|9/10|
|11|0.11|8|0.001|0.114|0.234|9/10|9/10|
|11|0.105|8|0.001|0.101|0.234|8.5/10|8.5/10|
|11|0.105|7.9|0.001|0.117|0.240|9/10|9.5/10|
|11|0.105|7.9|0.0|0.117|0.239|9.5/10|9.5/10|
|11.1|0.105|7.9|0.0|0.110|0.239|8.5/10|9/10|
|11.1|0.1|7.9|0.0|0.106|0.240|8.5/10|8.5/10|
|11|0.1|7.9|0.0|0.101|0.240|8.5/10|8.5/10|
|11|0.05|7.8|0.0|0.109|0.246|8.5/10|8/10|
|11|0.05|7.9|0.0|0.105|0.240|8.5/10|8/10|