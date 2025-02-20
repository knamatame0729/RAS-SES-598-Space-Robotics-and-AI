# Cart-Pole Optimal Control Assignment

[Watch the demo video](https://drive.google.com/file/d/1UEo88tqG-vV_pkRSoBF_-FWAlsZOLoIb/view?usp=sharing)
![image](https://github.com/user-attachments/assets/c8591475-3676-4cdf-8b4a-6539e5a2325f)

## Overview
This assignment challenges students to tune and analyze an LQR controller for a cart-pole system subject to earthquake disturbances. The goal is to maintain the pole's stability while keeping the cart within its physical constraints under external perturbations. The earthquake force generator in this assignment introduces students to simulating and controlling systems under seismic disturbances, which connects to the Virtual Shake Robot covered later in the course. The skills developed here in handling dynamic disturbances and maintaining system stability will be useful for optimal control of space robots, such as Lunar landers or orbital debris removal robots.

## System Description
The assignment is based on the problem formalism here: https://underactuated.mit.edu/acrobot.html#cart_pole
### Physical Setup
- Inverted pendulum mounted on a cart
- Cart traversal range: ±2.5m (total range: 5m)
- Pole length: 1m
- Cart mass: 1.0 kg
- Pole mass: 1.0 kg

### Disturbance Generator
The system includes an earthquake force generator that introduces external disturbances:
- Generates continuous, earthquake-like forces using superposition of sine waves
- Base amplitude: 15.0N (default setting)
- Frequency range: 0.5-4.0 Hz (default setting)
- Random variations in amplitude and phase
- Additional Gaussian noise

## Assignment Objectives

### Core Requirements
1. Analyze and tune the provided LQR controller to:
   - Maintain the pendulum in an upright position
   - Keep the cart within its ±2.5m physical limits
   - Achieve stable operation under earthquake disturbances
2. Document your LQR tuning approach:
   - Analysis of the existing Q and R matrices
   - Justification for any tuning changes made
   - Analysis of performance trade-offs
   - Experimental results and observations
3. Analyze system performance:
   - Duration of stable operation
   - Maximum cart displacement
   - Pendulum angle deviation
   - Control effort analysis

### Learning Outcomes
- Understanding of LQR control parameters and their effects
- Experience with competing control objectives
- Analysis of system behavior under disturbances
- Practical experience with ROS2 and Gazebo simulation

### Extra Credit Options
Students can implement reinforcement learning for extra credit (up to 30 points):

1. Reinforcement Learning Implementation:
   - Implement a basic DQN (Deep Q-Network) controller
   - Train the agent to stabilize the pendulum
   - Compare performance with the LQR controller
   - Document training process and results
   - Create training progress visualizations
   - Analyze and compare performance with LQR

## Implementation

### Controller Description
The package includes a complete LQR controller implementation (`lqr_controller.py`) with the following features:
- State feedback control
- Configurable Q and R matrices
- Real-time force command generation
- State estimation and processing

Current default parameters:
```python
# State cost matrix Q (default values)
Q = np.diag([1.0, 1.0, 10.0, 10.0])  # [x, x_dot, theta, theta_dot]

# Control cost R (default value)
R = np.array([[0.1]])  # Control effort cost
```

### Earthquake Disturbance
The earthquake generator (`earthquake_force_generator.py`) provides realistic disturbances:
- Configurable through ROS2 parameters
- Default settings:
  ```python
  parameters=[{
      'base_amplitude': 15.0,    # Strong force amplitude (N)
      'frequency_range': [0.5, 4.0],  # Wide frequency range (Hz)
      'update_rate': 50.0  # Update rate (Hz)
  }]
  ```

## Getting Started

### Prerequisites
- ROS2 Humble or Jazzy
- Gazebo Garden
- Python 3.8+
- Required Python packages: numpy, scipy

#### Installation Commands
```bash
# Set ROS_DISTRO as per your configuration
export ROS_DISTRO=humble

# Install ROS2 packages
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-interfaces \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-rviz2

# Install Python dependencies
pip3 install numpy scipy control
```

### Repository Setup

#### If you already have a fork of the course repository:
```bash
# Navigate to your local copy of the repository
cd ~/RAS-SES-598-Space-Robotics-and-AI

# Add the original repository as upstream (if not already done)
git remote add upstream https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI.git

# Fetch the latest changes from upstream
git fetch upstream

# Checkout your main branch
git checkout main

# Merge upstream changes
git merge upstream/main

# Push the updates to your fork
git push origin main
```

#### If you don't have a fork yet:
1. Fork the course repository:
   - Visit: https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI
   - Click "Fork" in the top-right corner
   - Select your GitHub account as the destination

2. Clone your fork:
```bash
cd ~/
git clone https://github.com/YOUR_USERNAME/RAS-SES-598-Space-Robotics-and-AI.git
```

### Create Symlink to ROS2 Workspace
```bash
# Create symlink in your ROS2 workspace
cd ~/ros2_ws/src
ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/cart_pole_optimal_control .
```

### Building and Running
```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select cart_pole_optimal_control --symlink-install

# Source the workspace
source install/setup.bash

# Launch the simulation with visualization
ros2 launch cart_pole_optimal_control cart_pole_rviz.launch.py
```

This will start:
- Gazebo simulation (headless mode)
- RViz visualization showing:
  * Cart-pole system
  * Force arrows (control and disturbance forces)
  * TF frames for system state
- LQR controller
- Earthquake force generator
- Force visualizer

### Visualization Features
The RViz view provides a side perspective of the cart-pole system with:

#### Force Arrows
Two types of forces are visualized:
1. Control Forces (at cart level):
   - Red arrows: Positive control force (right)
   - Blue arrows: Negative control force (left)

2. Earthquake Disturbances (above cart):
   - Orange arrows: Positive disturbance (right)
   - Purple arrows: Negative disturbance (left)

Arrow lengths are proportional to force magnitudes.

## Analysis Requirements

### Performance Metrics
Students should analyze:
1. Stability Metrics:
   - Maximum pole angle deviation
   - RMS cart position error
   - Peak control force used
   - Recovery time after disturbances

2. System Constraints:
   - Cart position limit: ±2.5m
   - Control rate: 50Hz
   - Pole angle stability
   - Control effort efficiency

### Analysis Guidelines
1. Baseline Performance:
   - Document system behavior with default parameters
   - Identify key performance bottlenecks
   - Analyze disturbance effects

2. Parameter Effects:
   - Analyze how Q matrix weights affect different states
   - Study R value's impact on control aggressiveness
   - Document trade-offs between objectives

3. Disturbance Response:
   - Characterize system response to different disturbance frequencies
   - Analyze recovery behavior
   - Study control effort distribution

## Evaluation Criteria
### Core Assignment (100 points)
1. Analysis Quality (40 points)
   - Depth of parameter analysis
   - Quality of performance metrics
   - Understanding of system behavior

2. Performance Results (30 points)
   - Stability under disturbances
   - Constraint satisfaction
   - Control efficiency

3. Documentation (30 points)
   - Clear analysis presentation
   - Quality of data and plots
   - Thoroughness of discussion

### Extra Credit (up to 30 points)
- Reinforcement Learning Implementation (30 points)

## Tips for Success
1. Start with understanding the existing controller behavior
2. Document baseline performance thoroughly
3. Make systematic parameter adjustments
4. Keep detailed records of all tests
5. Focus on understanding trade-offs
6. Use visualizations effectively

## Submission Requirements
1. Technical report including:
   - Analysis of controller behavior
   - Performance data and plots
   - Discussion of findings
2. Video demonstration of system performance
3. Any additional analysis tools or visualizations created

## License
This work is licensed under a [Creative Commons Attribution 4.0 International License](http://creativecommons.org/licenses/by/4.0/).
[![Creative Commons License](https://i.creativecommons.org/l/by/4.0/88x31.png)](http://creativecommons.org/licenses/by/4.0/) 

# Assignment 2

## Comparison of different parameters
|#|Q          |R       |Max Cart Displacement|Max Pole Angle|Average Control Effort|Max Control Effort|RMS|Average Recovery time|
|:--:|:-----:|:------:|:-----:|:-----:|:-----:|:-----:|:----:|:-:|
|1|[1 1 10 10]|0.1|N/A|N/A|N/A|N/A|N/A|N/A|N/A|
|2|[1 1 100 10]|0.01|0.107|0.038|2.497|55.314|0.039|0.253|
|3|[1 1 100 50]|0.01|0.101|0.03|5.926|124.772|0.041|0.512|
|4|[100 1 100 10]|0.01|0.14|0.147|39.755|151.507|0.045|0.31|
|5|[100 1 10 100]|0.01|1.681|0.419|998.335|1727.624|0.627|0.627|
|6|[100 1 10 100]|0.1|0.073|0.056|6.208|36.168|0.022|0.251|
|7|[50 1 10 100]|0.01|0.089|0.05|17.104|107.571|0.027|0.232|
|8|[10 1 10 100]|0.01|0.243|0.102|38.565|246.954|0.067|0.32|

## Plots
1. Q[1 1 10 10],R[0.1]  ---Failed---
![Image](https://github.com/user-attachments/assets/9332c152-17f3-4006-8a14-d7b8555020ba)
2. Q[1 1 100 10], R[0.01]
![Image](https://github.com/user-attachments/assets/cb4b829b-a80a-4f7d-9c3a-806f26909712)
3. Q[1 1 100 50], R[0.01]
![Image](https://github.com/user-attachments/assets/6a803254-859a-46a0-afa2-556febf9ce1b)
4. Q[100 1 100 10], R[0.01]
![Image](https://github.com/user-attachments/assets/589bc2c9-b5aa-40ae-a33f-944f1c29de8b)
5. Q[100 1 10 100], R[0.01]
![Image](https://github.com/user-attachments/assets/fb292b28-54b7-4a94-9943-d8ac1b0f29f4)
6. Q[100 1 10 100], R[0.1]
![Image](https://github.com/user-attachments/assets/d30eb9b5-c33e-473c-8637-8eefbf608812)
7. Q[50 1 10 100], R[0.01]
![Image](https://github.com/user-attachments/assets/b0eac831-5f53-40c4-8ca6-59e2fe8646f2)
8. Q[10 1 10 100], R[0.01]
![Image](https://github.com/user-attachments/assets/90659941-6bba-4c2b-a711-56fa1cd05ae3)

## Trade-Offs
1. ### How Q matrix weights affect
|Q      |Increasing Parameters|Decreasing Parameters|
|:-----:|:-------------------:|:-------------------:|
|Q[0, 0] |Reduces the movement of the cart (Less Ocillation)|Prioritizes the pole stability over the cart position(movement).|
|Q[1, 1]|The velocity of the cart will slow down (Less Ocillation)|The velocity of the cart will be fast. (Quick response)|
|Q[2, 2]|Stebilizes the pole in the vertical position quickly|Allows some degree of pole leaning |
|Q[3, 3]|Reduce the pole oscillations for smoothness| Makes faster angular motion for quick response|

2. ### How R parameter affects
|R       |Increasing Parameters|Decreasing Parameters|
|:------:|:-------------------:|:-------------------:|
|R       | Control the system smoother (Slower response)|Control the system more aggressive (Faster stabilization but More ocillation)|

## Discussion
From these results, #6 Q[100 1 10 100], R[0.1] is the best choise for the real world implementation since it is the most stable and efficient conrrol. Both average control efforts and max control effort is small compared to other cases. This means that the energy consumption will be low. 

## Reinforcement Learning
Reinforcement Learning is area fo machine learning that how an agent (Cart-pole in this case) should take actions in a environment to maximize a reward.

1. Markov Decision Process  
- State (S) : Velocity, Accelaration, Angle Velocity and Angle Accelaration
- Model  T(s, a, s') : Take (a) action will lead to the state (s') When the state is (s). 
- Action (A) : Right and Left (in this cart-pole case)
- Reward (R) : If the pole didn't fall, the environment gives a reward. On the other hand, if the pole fell, the environment gives a penalty.
- Policy π(s) => a: A function that decide how an agent should take actions when the state is (s)


In reinforcement learning the agent learn to decide actions that maximize the rewards. While the agent should focuse to maximize teh rewards without any consideration of time, it might take low-risk, low-return actions to risks such as keeping the pole stationary. The policy which learned from those condition might be not the best policy. To optimize the policy, we consider the discount rate which means  
" You have to take actions quickly. If you don't you will be given small rewards although the action is same." .  

Uπ(s) = E[Σr(t)*R(s(t))|π, s0 = s]  
Uπ(s) : Total of the rewards (From the state s, and do the plicy(π))  
Discount rate = r: 0 <= r < 1

The goal in the reinforcement learning is to find a policy that maximizes the reward with discount rate. The optimized policy is represented as π*.  

π* = argmaxΣT(s, a, s')U(s')  

This means that at the state (s), the agent will take the action (a) that maximizes the sum of rewards that consider the future rewards of the next state (s').  

From any given state (s), the agent takes actions that maximize rewards. U(s) can be written as below  

U(s) = R(s) + rmaxΣT(s, a, s')U(s')

This equation is called Bellman equation.



