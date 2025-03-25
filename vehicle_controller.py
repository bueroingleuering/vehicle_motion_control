"""@author: robin leuering"""

import numpy as np
import matplotlib.pyplot as plt

"""Parameters"""

m = 1850.0  # [kg]
A = 277.9595403794883  # [N]
B = 2.4402320270442033  # [N/(m/s)]
C = 0.220029084400295  # [N/(m/s)^2]
F_Max = 12000.0  # max Force [N]

"""Simulation Parameter"""

t_Delta = 0.1  # discrete time step [s]
t_End = 500.0  # simulation time [s]
t_Controller = np.arange(start=0, stop=t_End, step=t_Delta)  # time series
v_Target = 100.0 / 3.6 * np.ones_like(t_Controller)  # km/h -> m/s
for index in range(0, 3):
    v_Target[t_Controller > 100.0 * (index + 1)] = (120.0 - 20 * index) / 3.6  # variable velocity at 100s
F_Disturbance = np.zeros_like(t_Controller)  # disturbance force
F_Disturbance[t_Controller > 400.0] -= 1000.0

v_Controller = np.zeros_like(t_Controller)  # initialize array with 0.0
F_Controller = np.zeros_like(t_Controller)  # empty array
F_Total = np.zeros_like(t_Controller)  # empty array
K_Controller = 1000  # common controller cain
K_Integral = 500.0  # initial integral gain
e_Derivative = 0.0  # error n-1 for derivative
T_Derivative = 0.0  # time constant derivative

"""Simulation"""

for index in range(1, len(t_Controller)):  # simulation loop
    e_Controller = v_Target[index] - v_Controller[index - 1]  # error
    K_Proportional = K_Controller * e_Controller  # proportional amplified error
    T_Integral = 0.1 * m / (2 * C * v_Controller[index - 1] + B)  # adaptive time constant integral
    # T_Integral = 0.1 * m / (2 * C * 0 + B)  # zero velocity time constant integral
    # T_Integral = 0.1 * m / (2 * C * np.max(v_Target) + B)  # max velocity time constant integral
    K_Integral = K_Integral + K_Proportional * t_Delta / T_Integral  # integrated amplified error
    K_Integral = min(K_Integral, F_Max)  # anti windup
    K_Derivative = T_Derivative * (K_Proportional - K_Controller * e_Derivative) / t_Delta  # derivative amplified error
    e_Derivative = e_Controller  # error n-1
    F_Controller[index] = min(K_Proportional + K_Integral + K_Derivative, F_Max)  # limited drag force
    F_Total[index] = F_Controller[index] + F_Disturbance[index]  # total force
    dvdt = (F_Total[index] - C * v_Controller[index - 1] ** 2 - B * v_Controller[index - 1] - A) / m  # state equation
    v_Controller[index] = v_Controller[index - 1] + dvdt * t_Delta  # state variable

P_Controller = np.multiply(F_Controller, v_Controller)  # [W]
E_Controller = np.trapezoid(y=P_Controller, x=t_Controller)  # [Ws]

fig_1 = plt.figure(num=1, figsize=(10, 6))
plt.plot(t_Controller, 3.6 * v_Controller, label='Speed - Closed Loop', color='#284b64', linewidth=3)
plt.plot(t_Controller, 3.6 * v_Target, label='Speed - Target', color='#3C6E71', linewidth=3)
plt.xlabel('Time [s]', fontsize=16), plt.ylabel('Speed [km/h]', fontsize=16), plt.ylim([0, 140.0])
plt.legend(loc='lower right', fontsize=16), plt.grid(), plt.savefig('Speed comparison control.png', format="png", dpi=300)

fig_2 = plt.figure(num=2, figsize=(10, 6))
plt.plot(t_Controller, F_Total, label='Force - Closed Loop', color='#284b64', linewidth=3)
plt.plot(t_Controller, F_Disturbance, label='Force - Disturbance', color='#3C6E71', linewidth=3)
plt.xlabel('Time [s]', fontsize=16), plt.ylabel('Force [N]', fontsize=16), plt.ylim([-6e3, 14e3])
plt.legend(fontsize=16), plt.grid(), plt.savefig('Force comparison control.png', format="png", dpi=300)

fig_3 = plt.figure(num=3, figsize=(10, 6))
plt.plot(t_Controller, 1.36e-3 * P_Controller, label='Power - Closed Loop', color='#284b64', linewidth=3)
plt.fill_between(t_Controller, 1.36e-3 * P_Controller, label='Energy - Closed Loop', color='#3C6E71', alpha=0.25)
plt.xlabel('Time [s]', fontsize=16), plt.ylabel('Power [hp]', fontsize=16)
plt.legend(fontsize=16), plt.grid(), plt.savefig('Power control.png', format="png", dpi=300)