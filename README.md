Localization-and-Navigation-Using-EKF

---

### **Repository Description**
This repository showcases the implementation of an **Extended Kalman Filter (EKF)** for accurate localization and navigation of a **Micro Aerial Vehicle (MAV)**. By fusing sensor data from an onboard **IMU (Inertial Measurement Unit)** and external **Vicon system**, the project enables precise estimation of the MAV’s position, velocity, orientation, and sensor biases. The repository contains well-documented code, pre-parsed datasets, and tools for visualization to highlight the performance of the EKF in real-world scenarios.

---

### **Project Overview**
The core objective of this project is to integrate sensor fusion techniques for robust state estimation in dynamic environments. The **EKF** algorithm is applied in two distinct configurations:
1. **Position and Orientation Estimation**: Measurement updates utilize position and orientation data from the Vicon system.
2. **Velocity Estimation**: Measurement updates rely solely on velocity data provided by the Vicon system.

This dual-part implementation demonstrates the flexibility and robustness of EKF-based localization across varying sensor configurations.

---

### **Features and Functionality**
1. **Extended Kalman Filter (EKF)**:
   - **Process Model**: Utilizes acceleration and angular velocity from the IMU to predict the MAV’s state in the body frame.
   - **Measurement Models**:
     - Part 1: Position and orientation updates from Vicon data.
     - Part 2: Velocity updates from Vicon data.
   - Updates are optimized using the **Kalman gain** to minimize estimation errors.

2. **Datasets and Sensor Data**:
   - Pre-parsed `.mat` datasets include synchronized IMU and Vicon data:
     - **Vicon Data**: Pose, velocity, and angular rates (`[x y z roll pitch yaw vx vy vz ωx ωy ωz]`).
     - **IMU Data**: Acceleration and angular velocity in the MAV’s body frame.
   - Data is sampled at different rates and includes wireless transmission artifacts to simulate real-world conditions.

3. **Robust Sensor Fusion**:
   - Integrates complementary sensor data from IMU and Vicon for precise state estimation.
   - Handles delays and packet loss in sensor data streams using EKF’s prediction and correction mechanism.

4. **Visualization and Analysis**:
   - Overlays EKF estimates with ground truth data from Vicon for error analysis.
   - Provides comprehensive plots for positional, velocity, and orientation estimates.

---

### **Key Algorithms**
1. **Prediction Step**:
   - Uses IMU data to predict the MAV’s state vector and covariance matrix.
   - Incorporates body-frame angular velocity and acceleration for state propagation.
   - Example:
     ```matlab
     x_pred = f(x_prev, u); % State propagation using process model
     P_pred = F * P_prev * F' + Q; % Covariance prediction
     ```

2. **Measurement Update**:
   - Corrects the predicted state using measurements from the Vicon system.
   - Computes the Kalman gain and updates state and covariance estimates.
   - Example:
     ```matlab
     K = P * H' / (H * P * H' + R); % Kalman gain calculation
     x = x + K * (z - H * x); % State update
     P = (I - K * H) * P; % Covariance update
     ```

3. **Kalman Gain Optimization**:
   - Balances the trust in prediction and measurement data to minimize estimation errors.
   - Handles varying levels of noise in IMU and Vicon inputs.

---

### **Files and Folder Structure**
1. **KalmanFilt_Part1.m**:
   - Implements EKF with position and orientation measurement updates.
   - Uses Dataset 1 for positional data.

2. **KalmanFilt_Part2.m**:
   - Implements EKF with velocity measurement updates.
   - Uses Dataset 2 for velocity data.

3. **init.m**:
   - Initializes parameters, datasets, and process models for both EKF configurations.

4. **Datasets**:
   - Pre-parsed `.mat` files containing IMU and Vicon data.
   - Three datasets simulating diverse scenarios.

5. **Plotting Utilities**:
   - Functions for visualizing EKF estimates and comparing them with ground truth data.

---

### **How to Use**
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/Localization-and-Navigation-Using-EKF.git
   cd Localization-and-Navigation-Using-EKF
   ```
2. Open MATLAB 2023b and load the scripts.
3. Run `KalmanFilt_Part1.m` to estimate position and orientation or `KalmanFilt_Part2.m` to estimate velocity.
4. Modify parameters in `init.m` to switch between datasets or adjust noise parameters.
5. Generate plots to visualize and analyze performance.

---

### **Performance Evaluation**
- EKF estimates are compared against Vicon ground truth to evaluate accuracy.
- Visualization includes:
  - Position and velocity estimates.
  - Orientation and angular rate comparisons.
  - Error metrics for state estimation.

---

### **Applications**
- **Robotics Navigation**: Enhances localization accuracy for robots in dynamic environments.
- **Autonomous Vehicles**: Provides robust state estimation under sensor noise and data delays.
- **Aerospace Systems**: Enables precise navigation for UAVs and drones.
- **Sensor Fusion Research**: Explores advanced integration techniques for multi-sensor data.

---

### **Repository Highlights**
- **Focus on Code**: Modularized scripts for easy understanding and adaptation.
- **Robust Algorithms**: Handles real-world challenges like sensor noise, delays, and packet loss.
- **Extensive Documentation**: Includes comments and explanations for each step.

---

This detailed description offers a comprehensive overview of the repository, emphasizing the code, algorithms, datasets, and applications. It positions the project as a practical and research-driven implementation of EKF for navigation and localization tasks.
