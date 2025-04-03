"""
Kalman Filter for INS and VO Data Fusion
==============================================================================
Author:        Luca Erviati
Date:          2025-03-15
Version:       1.0
Description:   
    This module provides functionality to fuse INS (Inertial Navigation System)
    and VO (Visual Odometry) data using a Kalman Filter. The
    Kalman Filter estimates the position, velocity, and acceleration of an object
    by combining measurements from both sensors. The module includes classes for
    Kalman Filters that process INS and VO data separately and then fuse them
    to provide more accurate estimates.

Main Features:
    - Kalman Filter for INS data processing.
    - Kalman Filter for VO data processing.
    - Fusion of INS and VO data using Kalman Filter.
    - Speed estimation from fused data.

Dependencies:
    - NumPy: For numerical computations.
    - SciPy: For scientific computing.
    - CSVTool: Utility class for processing CSV files.
    - Speed Estimation: Module for calculating speed from displacement.
==============================================================================
"""

import numpy as np
import os
import sys

sys.path.append('/Users/luca/Downloads/VisualOdometry_Erviati_HITACHI/railfusion')

import speed_estimation as sp_est
from csv_tool import CSVTool as csv_t


dt = 0.1  # Sampling time
estimated_speeds = []

class IntegratedNavigation:
    def __init__(self, dt):
        self.dt = dt
        process_noise_std = 1  
        measurement_noise_std = 0.01 

        # State vector
        self.x = np.zeros((6, 1))  # [[x, vx, ax, y, vy, ay]]
        # State transition matrix
        self.F = np.array([
            [1, dt, 0.5 * dt**2, 0, 0, 0],  # Predicting x-position is equal to x_new = x + vx * dt + 0.5 * ax * dt^2
            [0, 1, dt, 0, 0, 0],            # Predicting x-velocity is equal to vx_new = vx + ax * dt
            [0, 0, 1, 0, 0, 0],             # Predicting x-acceleration is equal to ax_new = ax
            [0, 0, 0, 1, dt, 0.5 * dt**2],  # Predicting y-position is equal to y_new = y + vy * dt + 0.5 * ay * dt^2
            [0, 0, 0, 0, 1, dt],            # Predicting y-velocity is equal to vy_new = vy + ay * dt
            [0, 0, 0, 0, 0, 1]              # Predicting y-acceleration is equal to ay_new = ay
        ])

        # Measurement matrix
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],             # Measure x position
            [0, 0, 0, 1, 0, 0]              # Measure y position
        ])

        # Process noise covariance matrix
        self.Q = np.eye(6) * process_noise_std**2

        # Measurement noise covariance matrix
        self.R = np.eye(2) * measurement_noise_std**2

        # Covariance matrix
        self.P = np.eye(6) * 1000  # Large initial uncertainty
        self.x[1, 0] = 25/3.6  # Initial velocity in x (25 km/h)
        self.x[4, 0] = 0

    def state_derivative(self, state, dt):

        # Compute derivatives if i'm using runge kutta of the state vector based on the uniform acceleration model.

        x, vx, ax, y, vy, ay = state.flatten()
        
        # The state derivatives based on constant acceleration
        dx_dt = vx  # dx/dt = velocity in x
        dvx_dt = ax  # dvx/dt = acceleration in x
        dax_dt = 0   # dax/dt = 0 (assume no jerk)
        
        dy_dt = vy  # dy/dt = velocity in y
        dvy_dt = ay  # dvy/dt = acceleration in y
        day_dt = 0   # day/dt = 0 (assume no jerk)
        
        return np.array([[dx_dt], [dvx_dt], [dax_dt], [dy_dt], [dvy_dt], [day_dt]])
    

    def runge_kutta_4(self):
        # Predicts the next state using Runge-Kutta 4th order.
        k1 = self.dt * self.state_derivative(self.x, self.dt)
        k2 = self.dt * self.state_derivative(self.x + 0.5 * k1, self.dt)
        k3 = self.dt * self.state_derivative(self.x + 0.5 * k2, self.dt)
        k4 = self.dt * self.state_derivative(self.x + k3, self.dt)
        self.x = self.x + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0

    def predict(self, long_acc, lat_acc):
        self.x[2, 0] = long_acc     # Insert longitudinal acceleration
        self.x[5, 0] = lat_acc      # Insert lateral acceleration

        self.runge_kutta_4()
        #self.x = self.F @ self.x                      # The @ operator in Python is used for matrix multiplication
        
        self.P = self.F @ self.P @ self.F.T + self.Q  # Covariance prediction

    def update(self, z):
        # La differenza tra la posizione predetta e quella misurata dal lidar per mezzo del displacement ottenuto (da capire come fare)
        z_t = self.H @ self.x                       # Extract the predicted position from the state vector
        y = z - z_t                                 # Calculate the residual (innovation)
        S = self.H @ self.P @ self.H.T + self.R     # Innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)    # Calculate Kalman gain
        self.x = self.x + K @ y                     # Update the state estimate
        self.P = (np.eye(6) - K @ self.H) @ self.P  # Update covariance

    def update_process_noise(self):
        """
        Dynamically update the process noise covariance matrix (Q).
        """
        # Example: Scale process noise based on acceleration magnitude
        ax = self.x[2, 0]  # Longitudinal acceleration
        ay = self.x[5, 0]  # Lateral acceleration
        acceleration_magnitude = np.sqrt(ax**2 + ay**2)

        # Adjust process noise based on acceleration (higher acceleration -> higher noise)
        process_noise_std = 0.05 + 0.01 * acceleration_magnitude
        self.Q = np.eye(6) * process_noise_std**2

    def update_measurement_noise(self):
        """
        Dynamically update the measurement noise covariance matrix (R).
        """
        # Example: Adjust measurement noise based on velocity magnitude
        vx = self.x[1, 0]  # Longitudinal velocity
        vy = self.x[4, 0]  # Lateral velocity
        velocity_magnitude = np.sqrt(vx**2 + vy**2)

        # Adjust measurement noise based on velocity (higher velocity -> higher noise)
        measurement_noise_std = 0.01 + 0.005 * velocity_magnitude
        self.R = np.eye(2) * measurement_noise_std**2

    def integrate_sensors(self, corrimu, longitudinal_displacement, lateral_displacement):
        long_acc = corrimu["longitudinal_acc"]
        lat_acc = corrimu["lateral_acc"]

        long_acc_m_s2 = long_acc/self.dt
        lat_acc_m_s2 = lat_acc/self.dt

        # Convert displacement to absolute position
        longitudinal_displacement_m = self.x[0, 0] + longitudinal_displacement  # x position + displacement
        lateral_displacement_m = self.x[3, 0] + lateral_displacement            # y position + displacement

        # Predict
        self.predict(long_acc_m_s2, lat_acc_m_s2)

        # Construct measurement vector
        z = np.array([
        [longitudinal_displacement_m], #Add displacement to predicted value.
        [lateral_displacement_m]
        ])

        # Update Kalman Filter
        self.update(z)



def main():

    vo_data = []
    # Example VO data (replace with actual data or run the code)
    # vo_data = [0.7529466514467913, 0.7076726260610116, 0.6482075507324083, 0.7508025232340714, 0.6923270199103655, 0.7038209799950579, 0.7453377512535866, 0.7534564936602663, 0.7088268862301419, 0.7073776973982717, 0.7415715403690939, 0.7055685806657976, 0.7699961650518787, 0.703949830982225, 0.761107756468931, 0.7908630618520505, 0.7064982737495598, 0.7801104706199311, 0.7119002387860576, 0.7601435524436653, 0.7429147541837722, 0.6862436199111812, 0.8712510270034386, 0.7377694501522001, 0.8013103436559987, 0.8034753661612939, 0.7908539000131327, 0.7574955013826923, 0.8552621905975144, 0.7674077251844977, 0.8218610726438271, 0.7821341736424046, 0.8229618805397507, 0.783873237021055, 0.7888082660027607, 0.7818482300564398, 0.714970177846908, 0.7566568120124799, 0.812045487425209, 0.9105274514398047, 0.7081506975778638, 0.7412147327424776, 0.7717181462189444, 0.8580463989788072, 0.8109143712536522, 0.8418684754630732, 0.8683725787299537, 0.8038642398637066, 0.8940226667884872, 0.8756196366053217, 0.81166214043688, 0.8747083010525358, 0.8636801370612517, 0.82544565307537, 0.8627820056276434, 0.8276741046450837, 0.8940176557556754, 0.8446616437270009, 0.8147237960725846, 0.9083494915177042, 0.7895386690111579, 0.8791724276275019, 0.8007483192198421, 0.9243836123758697, 0.8333561853422893, 0.8901889902844786, 0.9041462464693666, 0.8870135378951858, 0.9569161539426858, 0.8456476631006993, 0.8797206153799593, 0.9028009098480112, 0.8578843538055736, 0.9101881961680292, 0.8417336487315339, 0.9720375376093529, 0.8464546676741236, 0.9197034963424997, 0.8589588365598857, 0.8970187108875862, 0.9342151826536949, 0.9610786453477047, 0.9340021547486757, 0.8611935936035984, 0.9231186559872846, 0.972418314876883, 0.8570885018104839, 0.9669692491934683, 0.9242939664783982, 0.9255276381536888, 0.9638722971677289, 0.9969638507619116, 0.9640897240826476, 0.8514271351258209, 0.8842570641025098, 0.8967962955210531, 0.9382815853245177, 0.9052107819665736, 18.403301892477455]

    for displacement in sp_est.displacement_list:
        if displacement > 0:  # Example of additional logic
            vo_data.append({
                "longitudinal_displacement": displacement,
                "lateral_displacement": 0,
                "vertical_displacement": 0
            })
    
    # Create an instance of CSVTool
    csv_tool_instance = csv_t()

    # Process the CSV files to populate the data
    csv_tool_instance.list_csv_files()
    csv_tool_instance.read_csv_files(csv_tool_instance.csv_files_corrimu, csv_tool_instance.csv_rows_corrimu)
    csv_tool_instance.read_csv_files(csv_tool_instance.csv_files_insstdev, csv_tool_instance.csv_rows_insstdev)

    corrimu_data = csv_tool_instance.csv_rows_corrimu
    insstdev_data = csv_tool_instance.csv_rows_insstdev

    nav = IntegratedNavigation(dt)



    for vo, corrimu, insstdev in zip(vo_data, corrimu_data, insstdev_data): #using zip to iterate over both list at the same time.
        corrimu = {
            "longitudinal_acc": float(corrimu[8]),
            "lateral_acc": float(corrimu[7]),
        }
        insstdev = [insstdev[2], insstdev[3], insstdev[5], insstdev[6]]  # Extract only the first two values for latitude and longitude standard deviations
        longitudinal_displacement = vo
        lateral_displacement = 0
        
        nav.integrate_sensors(corrimu, longitudinal_displacement, lateral_displacement)

        # Compute total speed from velocity components
        v_long, v_lat = nav.x[1, 0], nav.x[4, 0]
        total_speed = np.sqrt(v_long**2 + v_lat**2)
        estimated_speeds.append(total_speed * 3.6)


    # CHECK THE DATA
    print(f"CORRIMU DATA...")
    print(corrimu_data[1], corrimu_data[2],corrimu_data[3])
    print(f"INSPVA DATA...")
    print(inspva_data)
    print(f"INSSTDEV DATA...")
    print(insstdev_data)
    print(f"DISPLACEMENT...")
    print(sp_est.displacement_list)
    print(vo_data)
    print(f"GROUND TRUTH SPEED...")
    print(csv_tool_instance.compute_ground_truth_speeds())'
    
    print(f"FILTERED SPEED...")
    print(estimated_speeds)


if __name__ == "__main__":
    main()
