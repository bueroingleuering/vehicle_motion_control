% Parameter
m = 1850.0; % Masse des Fahrzeugs [kg]
A = 277.9595403794883; % Statischer Widerstandskoeffizient [N]
B = 2.4402320270442033; % Rollwiderstandskoeffizient [N/(m/s)]
C = 0.220029084400295; % Luftwiderstandskoeffizient [N/(m/s)^2]
F_Max = 12000.0; % Maximale Antriebskraft [N]

% Simulationsparameter
t_Delta = 0.1; % Zeitschritt [s]
t_End = 500.0; % Gesamte Simulationszeit [s]
t_Sim = 0:t_Delta:t_End; % Zeitvektor

% Zielgeschwindigkeit [m/s]
v_Target = (100.0 / 3.6) * ones(size(t_Sim));
for i = 1:3
    v_Target(t_Sim > 100.0 * i) = (120.0 - 20 * (i - 1)) / 3.6;
end

% Störkraft [N]
F_Disturbance = zeros(size(t_Sim));
F_Disturbance(t_Sim > 400.0) = -1000.0;

% Initialisierung
v_Controller = zeros(size(t_Sim)); % Geschwindigkeit [m/s]
F_Controller = zeros(size(t_Sim)); % Steuerkraft [N]
F_Total = zeros(size(t_Sim)); % Gesamtkraft [N]

% PID-Basisverstärkung
Kp = 500.0;

% Initialisierung der PID-Variablen
e_Prev = 0;
e_Integral = 0;
T_Derivative = 0; % Zeitkonstante D-Anteil

% Leistung und Energie Initialisierung
P = zeros(size(t_Sim));  % Leistung [W]
E = zeros(size(t_Sim));  % Energie [Joule]

% Simulation des Regelkreises
for i = 2:length(t_Sim)
    % Fehlerberechnung
    e = v_Target(i) - v_Controller(i-1);
    
    % Dynamische Zeitkonstante für den Integralanteil
    T_Integral = 0.1 * m / (2 * C * v_Controller(i-1) + B);
    
    % Alternative Berechnungsmethoden für T_Integral (kann aktiviert werden)
    % T_Integral = 0.1 * m / (2 * C * 0 + B);  % Wenn Geschwindigkeit 0 angenommen wird
    % T_Integral = 0.1 * m / (C * max(v_Target) + B);  % Wenn max(v_Target) als konstant angenommen wird
    
    % PID-Koeffizienten
    Ki = Kp / T_Integral;
    Kd = Kp * T_Derivative;

    % Integrierter Fehler mit Anti-Windup
    e_Integral = e_Integral + e * t_Delta;
    
    % Anti-Windup
    if F_pid == F_Saturated
        e_Integral = e_Integral - e * t_Delta;
    end

    % Differenzialanteil
    e_Derivative = (e - e_Prev) / t_Delta;

    % PID-Steuerkraft berechnen
    F_pid = Kp * e + Ki * e_Integral + Kd * e_Derivative;
    
    % Begrenzung der Steuerkraft & Anti-Windup-Korrektur
    F_Saturated = max(min(F_pid, F_Max), -F_Max);
    if F_pid ~= F_Saturated
        e_Integral = e_Integral - e * t_Delta;
    end

    % Steuerkraft setzen
    F_Controller(i) = F_Saturated;
    
    % Gesamtkraft
    F_Total(i) = F_Controller(i) - (A + B * v_Controller(i-1) + C * v_Controller(i-1)^2) + F_Disturbance(i);
    
    % Berechnung der Geschwindigkeitänderung dv/dt
    dvdt = F_Total(i) / m;  % Änderungsrate der Geschwindigkeit [m/s^2]
    
    % Fahrzeugdynamik (Euler-Integration)
    v_Controller(i) = v_Controller(i-1) + dvdt * t_Delta;  % Geschwindigkeit aktualisieren
    
    % Berechnung der Leistung
    P(i) = F_Total(i) * v_Controller(i);  % Leistung [W]
    
    % Berechnung der Energie durch numerische Integration der Leistung
    E(i) = E(i-1) + P(i) * t_Delta;  % Energie [Joule]
    
    % Aktualisierung des vorherigen Fehlers
    e_Prev = e;
end

% Plot der Geschwindigkeit
figure;
plot(t_Sim, v_Target * 3.6, 'r--', 'DisplayName', 'Zielgeschwindigkeit');
hold on;
plot(t_Sim, v_Controller * 3.6, 'b', 'DisplayName', 'Fahrzeuggeschwindigkeit');
xlabel('Zeit [s]');
ylabel('Geschwindigkeit [km/h]');
legend;
grid on;

% Plot der Steuerkraft
figure;
plot(t_Sim, F_Controller, 'g', 'DisplayName', 'Steuerkraft');
xlabel('Zeit [s]');
ylabel('Steuerkraft [N]');
legend;
grid on;

% Plot der Leistung und Energie - mit Fläche unter der Leistungskurve
figure;
plot(t_Sim, P, 'r', 'DisplayName', 'Leistung'); % Leistungskurve
hold on;
area(t_Sim, E, 'FaceColor', 'b', 'FaceAlpha', 0.3, 'DisplayName', 'Energie'); % Fläche unter der Leistungskurve
xlabel('Zeit [s]');
ylabel('Leistung [W] / Energie [J]');
legend;
grid on;