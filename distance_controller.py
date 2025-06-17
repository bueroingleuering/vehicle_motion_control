"""@author: robin leuering"""

import numpy as np
import matplotlib.pyplot as plt

"""Parameters"""

vehicle_mass = 1850.0  # [kg]
coefficient_A = 277.9595403794883  # [N]
coefficient_B = 2.4402320270442033  # [N/(m/s)]
coefficient_C = 0.220029084400295  # [N/(m/s)^2]
F_Max = 12000.0  # max Force [N]

"""Simulation Parameter"""

t_Delta = 0.1  # discrete time step [s]
t_End = 100.0  # simulation time [s]
t_Controller = np.arange(start=0, stop=t_End, step=t_Delta)  # time series
v_Min = 0.0  # m/s
v_Max = 120.0 / 3.6 # km/h -> m/s
v_Ahead = 100.0 / 3.6 * np.ones_like(t_Controller)  # km/h -> m/s
v_Setpoint = v_Ahead * np.ones_like(t_Controller) # km/h -> m/s
v_Controller = 100.0 / 3.6 * np.ones_like(t_Controller)  # km/h -> m/s
d_Min = 100.0 * np.ones_like(t_Controller)  # m
d_Controller = 200.0 * np.ones_like(t_Controller)  # initialize distance with 200 m/s
F_Controller = np.zeros_like(t_Controller)  # empty array
K_Controller_v = 10000.0 # common controller gain
K_Integral_v = coefficient_A  # initial integral gain
K_Controller_d = -0.25
K_Integral_d = v_Ahead[0]

"""Simulation"""
for index in range(1, len(t_Controller)):  # simulation loop
    e_Controller_d = d_Min[index - 1] - d_Controller[index - 1]  # error distance
    K_Proportional_d = K_Controller_d * e_Controller_d  # proportional amplified error distance
    T_Integral_d = K_Controller_v / vehicle_mass
    K_Integral_d = K_Integral_d + K_Proportional_d * t_Delta / T_Integral_d
    K_Integral_d = max(min(K_Integral_d, v_Max), v_Min)  # anti windup
    v_Setpoint[index] = min(K_Proportional_d + K_Integral_d, v_Max)  # v_Min<v_Setpoint<v_Max
    e_Controller_v = v_Setpoint[index] - v_Controller[index - 1]  # error speed
    K_Proportional_v = K_Controller_v * e_Controller_v  # proportional amplified error speed
    T_Integral_v = (1 * vehicle_mass /
                   (2 * coefficient_C * v_Controller[index - 1] + coefficient_B))  # adaptive time constant integral
    K_Integral_v = K_Integral_v + K_Proportional_v * t_Delta / T_Integral_v  # integrated amplified error
    K_Integral_v = max(min(K_Integral_v, F_Max), -F_Max)  # anti windup
    F_Controller[index] = min(K_Proportional_v + K_Integral_v, F_Max)  # limited drag force
    dvdt = (F_Controller[index] -
            coefficient_C * v_Controller[index - 1] ** 2 -
            coefficient_B * v_Controller[index - 1] -
            coefficient_A) / vehicle_mass  # state equation
    v_Controller[index] = v_Controller[index - 1] + dvdt * t_Delta  # state variable x2
    d_Controller[index] = d_Controller[index - 1] + (v_Ahead[index] - v_Controller[index]) * t_Delta # state variable x1

fig_1 = plt.figure(num=1, figsize=(10, 6))
plt.plot(t_Controller, 3.6 * v_Ahead, label='Speed - Ahead', color='#3C6E71', linewidth=3)
plt.plot(t_Controller, 3.6 * v_Setpoint, label='Speed - Setpoint', color='#284b64', linewidth=3)
plt.plot(t_Controller, 3.6 * v_Controller, label='Speed - Closed Loop',color='#893636' , linewidth=3)
plt.xlabel('Time [s]', fontsize=16), plt.ylabel('Speed [km/h]', fontsize=16), plt.ylim([80.0, 140.0])
plt.legend(fontsize=16), plt.grid(), plt.savefig('Speed comparison control.png', format="png", dpi=300)

fig_2 = plt.figure(num=2, figsize=(10, 6))
plt.plot(t_Controller, F_Controller, label='Force - Closed Loop', color='#284b64', linewidth=3)
plt.xlabel('Time [s]', fontsize=16), plt.ylabel('Force [N]', fontsize=16)
plt.legend(fontsize=16), plt.grid(), plt.savefig('Force comparison control.png', format="png", dpi=300)

fig_3 = plt.figure(num=3, figsize=(10, 6))
plt.plot(t_Controller, d_Controller, label='Distance - Closed Loop', color='#284b64', linewidth=3)
plt.xlabel('Time [s]', fontsize=16), plt.ylabel('Distance [m]', fontsize=16), plt.ylim([0, 220.0])
plt.legend(fontsize=16), plt.grid(), plt.savefig('Distance comparison control.png', format="png", dpi=300)
