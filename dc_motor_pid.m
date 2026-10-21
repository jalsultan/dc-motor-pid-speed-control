% DC Motor Speed Control with PID
% Script version of dc_motor_pid_speed_control.slx
% Builds the motor model from first principles, compares P, PI and PID,
% and shows the effect of input saturation (+-450 V and +-24 V).

clear; clc; close all;

%% Motor parameters
R = 2;      % armature resistance (ohm)
L = 0.5;    % armature inductance (H)
J = 0.02;   % rotor inertia (kg*m^2)
b = 0.2;    % viscous friction (N*m*s/rad)
K = 0.1;    % motor constant (V*s/rad = N*m/A)

%% Transfer function from first principles
% V = R*i + L*di/dt + K*w   and   J*dw/dt + b*w = K*i
s = tf('s');
G = K / ((L*s + R)*(J*s + b) + K^2);    % = 0.1 / (0.01s^2 + 0.14s + 0.41)
disp('Motor transfer function:'); G
disp('Open-loop poles:'); disp(pole(G))

%% Simulation settings
r      = 100;    % speed reference (rad/s)
d      = 0.5;    % output disturbance step (rad/s)
t_dist = 20;     % disturbance time (s)
t      = (0:1e-3:30)';
ref    = r * ones(size(t));
dist   = d * (t >= t_dist);

%% Controllers (derivative filter N = 100, as in the Simulink PID block)
N  = 100;
Cs = {pid(50, 0, 0), pid(50, 100, 0), pid(15, 40, 1.5, 1/N)};
names = {'P', 'PI', 'PID'};

%% Compare P, PI and PID with no saturation
fprintf('\n%-5s %8s %8s %8s %16s %14s %10s\n', 'Ctrl', 'Kp', 'Ki', 'Kd', 'Rise time (s)', 'Overshoot (%)', 'SS error');
figure('Name', 'P vs PI vs PID'); hold on;
for k = 1:3
    C = Cs{k};
    T  = feedback(C*G, 1);    % reference -> speed
    S  = feedback(1, C*G);    % output disturbance -> speed
    y  = lsim(T, ref, t) - lsim(S, dist, t);

    info = stepinfo(r*T);     % rise time is 10-90% by default
    ess  = r - r*dcgain(T);
    fprintf('%-5s %8.1f %8.1f %8.1f %16.3f %14.2f %9.2f%%\n', names{k}, C.Kp, C.Ki, C.Kd, ...
            info.RiseTime, info.Overshoot, ess);
    plot(t, y, 'LineWidth', 1.5);
end
yline(r, '--k'); yline(1.15*r, ':r');
xlim([0 3]); grid on;
xlabel('Time (s)'); ylabel('Speed (rad/s)');
legend('P', 'PI', 'PID', 'Reference', '15% overshoot limit', 'Location', 'southeast');
title('P vs PI vs PID, no saturation');

%% PID with input saturation (same structure as the Simulink model)
% Simple fixed-step simulation so the saturation block can be included.
figure('Name', 'PID with saturation'); hold on;
for Vmax = [450 24]
    y = sim_pid_sat(15, 40, 1.5, N, Vmax, r, d, t_dist, t);
    plot(t, y, 'LineWidth', 1.5);
    fprintf('PID with +-%d V limit: final speed = %.2f rad/s\n', Vmax, y(end));
end
yline(r, '--k'); grid on;
xlabel('Time (s)'); ylabel('Speed (rad/s)');
legend('\pm450 V limit', '\pm24 V limit', 'Reference', 'Location', 'east');
title('PID with input saturation');

%% Voltage needed to hold 100 rad/s
V_ss = r * (R*b + K^2) / K;
fprintf('Steady-state voltage needed for %d rad/s: %.0f V\n', r, V_ss);
fprintf('Max speed with 24 V: %.2f rad/s\n', 24 * dcgain(G));

%% Local function
function y = sim_pid_sat(Kp, Ki, Kd, N, Vmax, r, d, t_dist, t)
    dt = t(2) - t(1);
    % plant in state-space form: x1'' + 14 x1' + 41 x1 = 10 u
    x = [0; 0]; integ = 0; filt = 0;
    y = zeros(size(t));
    for k = 1:numel(t)
        yk = x(1) - d*(t(k) >= t_dist);
        e  = r - yk;
        u  = Kp*e + Ki*integ + Kd*N*(e - filt);
        u  = min(max(u, -Vmax), Vmax);        % saturation block
        integ = integ + e*dt;
        filt  = filt + N*(e - filt)*dt;
        x = x + dt*[x(2); 10*u - 14*x(2) - 41*x(1)];
        y(k) = yk;
    end
end
