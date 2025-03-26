% Parameters
m = 1850.0; % Vehicle mass [kg]
A = 277.9595403794883; % Static drag coefficient [N]
B = 2.4402320270442033; % Rolling resistance coefficient [N/(m/s)]
C = 0.220029084400295; % Air drag coefficient [N/(m/s)^2]
F_Max = 12000.0; % Maximum driving force [N]

% Simulation parameters
t_Delta = 0.1; % Time step [s]
t_End = 500.0; % Total simulation time [s]
t_Sim = 0:t_Delta:t_End; % Time vector

% Target speed [m/s]
v_Target = (100.0 / 3.6) * ones(size(t_Sim));
for i = 1:3
    v_Target(t_Sim > 100.0 * i) = (120.0 - 20 * (i - 1)) / 3.6;
end

% Disturbance force [N]
F_Disturbance = zeros(size(t_Sim));
F_Disturbance(t_Sim > 400.0) = -1000.0;

% Initialization
v_Controller = zeros(size(t_Sim)); % Vehicle speed [m/s]
F_Controller = zeros(size(t_Sim)); % Control force [N]
F_Total = zeros(size(t_Sim)); % Total force [N]

% PID base gain
Kp = 500.0;

% Initialize PID variables
e_Prev = 0;
e_Integral = 0;
T_Derivative = 0; % Derivative time constant

% Power and energy initialization
P = zeros(size(t_Sim));  % Power [W]
E = zeros(size(t_Sim));  % Energy [J]

% Simulation loop
for i = 2:length(t_Sim)
    % Error calculation
    e = v_Target(i) - v_Controller(i-1);

    % Dynamic time constant for the integral part
    T_Integral = 0.1 * m / (2 * C * v_Controller(i-1) + B);

    % Alternative calculation methods for T_Integral (can be activated)
    % T_Integral = 0.1 * m / (2 * C * 0 + B);  % When speed is assumed to be 0
    % T_Integral = 0.1 * m / (C * max(v_Target) + B);  % When max(v_Target) is assumed constant

    % PID coefficients
    Ki = Kp / T_Integral;
    Kd = Kp * T_Derivative;

    % Integral error with Anti-Windup
    e_Integral = e_Integral + e * t_Delta;

    % Anti-Windup
    if F_pid == F_Saturated
        e_Integral = e_Integral - e * t_Delta;
    end

    % Derivative term
    e_Derivative = (e - e_Prev) / t_Delta;

    % PID control force calculation
    F_pid = Kp * e + Ki * e_Integral + Kd * e_Derivative;

    % Saturation of control force & Anti-Windup correction
    F_Saturated = max(min(F_pid, F_Max), -F_Max);
    if F_pid ~= F_Saturated
        e_Integral = e_Integral - e * t_Delta;
    end

    % Set control force
    F_Controller(i) = F_Saturated;

    % Total force
    F_Total(i) = F_Controller(i) - (A + B * v_Controller(i-1) + C * v_Controller(i-1)^2) + F_Disturbance(i);

    % Calculation of velocity change dv/dt
    dvdt = F_Total(i) / m;  % Rate of change of speed [m/s^2]

    % Vehicle dynamics (Euler integration)
    v_Controller(i) = v_Controller(i-1) + dvdt * t_Delta;  % Update speed

    % Power calculation
    P(i) = F_Total(i) * v_Controller(i);  % Power [W]

    % Energy calculation by numerical integration of power
    E(i) = E(i-1) + P(i) * t_Delta;  % Energy [Joule]

    % Update previous error
    e_Prev = e;
end

% Speed plot
figure;
plot(t_Sim, v_Target * 3.6, 'r--', 'DisplayName', 'Target Speed');
hold on;
plot(t_Sim, v_Controller * 3.6, 'b', 'DisplayName', 'Vehicle Speed');
xlabel('Time [s]');
ylabel('Speed [km/h]');
legend;
grid on;

% Control force plot
figure;
plot(t_Sim, F_Controller, 'g', 'DisplayName', 'Control Force');
xlabel('Time [s]');
ylabel('Control Force [N]');
legend;
grid on;

% Power and energy plot - with area under the power curve
figure;
plot(t_Sim, P, 'r', 'DisplayName', 'Power'); % Power curve
hold on;
area(t_Sim, E, 'FaceColor', 'b', 'FaceAlpha', 0.3, 'DisplayName', 'Energy'); % Area under the power curve
xlabel('Time [s]');
ylabel('Power [W] / Energy [J]');
legend;
grid on;