Localization-and-Navigation-Using- Extended Kalman Filter

---

### Description
This repository demonstrates the implementation of an **Extended Kalman Filter (EKF)** to achieve precise localization and navigation for a **Micro Aerial Vehicle (MAV)**. By leveraging sensor fusion between IMU (Inertial Measurement Unit) data and external Vicon measurements, this project estimates critical state variables such as position, velocity, orientation, and sensor biases. The repository includes MATLAB code, pre-parsed datasets, and detailed visualizations, making it a comprehensive resource for exploring EKF-based sensor fusion in real-world scenarios.

---

### **Project Overview**
Localization and navigation are essential in robotics, particularly for autonomous aerial vehicles. This project addresses these challenges through a robust EKF implementation that integrates noisy and asynchronous IMU and Vicon data. The work is divided into two key parts:
1. **Part 1**: Position and orientation estimation using Vicon pose data.
2. **Part 2**: Velocity estimation using Vicon velocity data.

Each part uses the same process model but incorporates distinct measurement models tailored to the available sensor data.

---

### **Features and Functionality**
1. **Extended Kalman Filter Implementation**:
   - **Process Model**:
     - Utilizes body-frame acceleration and angular velocity from the IMU to predict the system's state.
     - Predicts position, velocity, orientation, and sensor biases.
   - **Measurement Models**:
     - Part 1: Updates state using position and orientation data from Vicon.
     - Part 2: Updates state using velocity data from Vicon.
   - Incorporates prediction and update steps, optimizing the state estimates with Kalman gain.

2. **Robust Sensor Fusion**:
   - Combines high-frequency IMU data and low-latency Vicon measurements.
   - Handles packet loss and noise to maintain estimation accuracy in dynamic conditions.

3. **Comprehensive Datasets**:
   - Includes three `.mat` datasets (`studentdata1.mat`, `studentdata4.mat`, `studentdata9.mat`), each containing synchronized IMU and Vicon data.
   - Dataset structure:
     ```matlab
     [x, y, z, roll, pitch, yaw, vx, vy, vz, ωx, ωy, ωz]
     ```
   - Covers varying motion scenarios to evaluate the EKF under different conditions.

4. **Visualization Tools**:
   - Plotting functions overlap EKF estimates with ground truth data from Vicon.
   - Visualizations include error analysis for position, velocity, and orientation.

---

### **Key Files and Structure**
#### **Part 1: Position and Orientation Estimation**
- `KalmanFilt_Part1.m`: Main script for Part 1, implementing EKF updates based on position and orientation measurements.
- `pred_step.m`: Implements the prediction step of the EKF using IMU data.
- `upd_step.m`: Defines the update step for position and orientation.
- `plotData.m`: Generates plots comparing EKF estimates to Vicon ground truth.

#### **Part 2: Velocity Estimation**
- `KalmanFilt_Part2.m`: Main script for Part 2, implementing EKF updates based on velocity measurements.
- `pred_step.m`: Shared with Part 1 to propagate the process model.
- `upd_step.m`: Update step tailored for velocity data.
- `plotData.m`: Visualizes velocity estimates and compares them to ground truth.

#### **Initialization and Configuration**
- `init.m`: Initializes datasets, parameters, and noise models for both parts.

#### **Datasets**
- **Provided Data Files**:
  - `studentdata1.mat`
  - `studentdata4.mat`
  - `studentdata9.mat`
- Contain synchronized IMU and Vicon data for testing EKF implementations.

---

### **How to Use**
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/Localization-and-Navigation-Using-EKF.git
   cd Localization-and-Navigation-Using-EKF
   ```
2. Open MATLAB 2023b and load the scripts.
3. Run the scripts for the respective parts:
   - **Part 1 (Position and Orientation)**:
     ```matlab
     KalmanFilt_Part1
     ```
   - **Part 2 (Velocity)**:
     ```matlab
     KalmanFilt_Part2
     ```
4. Use `plotData.m` to visualize and analyze the results.

---

### **Core Algorithms**
#### **Prediction Step**:
- Propagates the state using IMU-derived angular velocity and acceleration.
- Updates the state vector and covariance matrix:
  ```matlab
  x_pred = f(x_prev, u); % State propagation using process model
  P_pred = F * P_prev * F' + Q; % Covariance prediction
  ```

#### **Update Step**:
- Refines the predicted state using Kalman gain and measurement data:
  ```matlab
  K = P * H' / (H * P * H' + R); % Kalman gain calculation
  x = x + K * (z - H * x); % State update
  P = (I - K * H) * P; % Covariance update
  ```

#### **Kalman Gain**:
- Balances trust between prediction and measurement:
  ```matlab
  K = P * H' / (H * P * H' + R);
  ```

---

### **Applications**
- **Autonomous Navigation**: Provides robust state estimation for drones, MAVs, and other robots.
- **Aerospace Systems**: Enables precision localization and motion tracking for UAVs.
- **Robotics Research**: Offers a foundational framework for sensor fusion and state estimation.

---

### **Performance Metrics**
1. **Positional Accuracy**:
   - EKF position estimates closely match Vicon ground truth.
   - Plots showcase deviations and error margins.

2. **Velocity Tracking**:
   - Accurate velocity estimates using IMU and Vicon velocity measurements.

3. **Orientation Estimation**:
   - EKF estimates Euler angles (roll, pitch, yaw) with minimal error.

---

### **Future Improvements**
- Incorporate visual odometry data for enhanced robustness.
- Extend to SLAM (Simultaneous Localization and Mapping) with obstacle detection.
- Test additional datasets with varied noise levels.
