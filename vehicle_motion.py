import numpy as np
import matplotlib.pyplot as plt
import control as ct
plt.ion()


m = 1850  # [kg]
A = 277.9595403794883  # [N]
B = 2.4402320270442033  # [N/(m/s)]
C = 0.220029084400295  # [N/(m/s)^2]
F_Max = 12000
# v_end = -(B / (2 * C)) + np.sqrt((B / (2 * C)) ** 2 + (F - A) / C)
v_end = 100.0 / 3.6
F = min(C * v_end ** 2 + B * v_end + A, F_Max)
t_step = np.array([0.0])
t_delta = 0.1
v_step = np.array([0.0])
index = 0

while v_step[index] < (v_end - 0.01):
    dvdt = (F - C * v_step[index] ** 2 - B * v_step[index] - A) / m
    index += 1
    t_step = np.append(t_step, t_step[index - 1] + t_delta)
    v_step = np.append(v_step, v_step[index - 1] + dvdt * t_delta)

v_kph = 3.6 * v_step

T_System = m / (2 * C * v_end + B)
G_System = (T_System / m) * ct.tf(1, [T_System, 1])
K_Controller = 50
G_Controller = ct.tf([K_Controller * T_System, K_Controller], [T_System, 0.0])
G_Open_Loop = G_Controller * G_System
G_Close_Loop = v_end * G_Open_Loop / (1.0 + G_Open_Loop)  # Close loop
_, v_Close_Loop = ct.step_response(G_Close_Loop, t_step)


fig_1 = plt.figure(num=1, figsize=(10, 6))
plt.plot(t_step, 3.6 * v_step, label="Speed - Constant Force", color='#284b64', linewidth=3)
plt.plot(t_step, 3.6 * v_Close_Loop, label="Speed - Closed Loop", color='#3C6E71', linewidth=3)
plt.xlabel('Time [s]', fontsize=16), plt.ylabel('Speed [km/h]', fontsize=16), plt.legend(fontsize=16), plt.grid()
plt.savefig('Speed comparison step.png', format="png", dpi=300)