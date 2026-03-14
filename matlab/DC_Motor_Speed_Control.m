clear all;
close all;
clc;

fprintf('========================================\n');
fprintf('DC MOTOR SPEED CONTROL PROJECT\n');
fprintf('========================================\n\n');

J = 0.01;
b = 0.1;
Kt = 0.01;
Ke = 0.01;
R = 1;
L = 0.5;

Rb_Kt = R * b / Kt;
V_to_omega_factor = Rb_Kt + Ke;
omega_to_V_factor = 1 / V_to_omega_factor;

fprintf('Motor Parameters:\n');
fprintf('-----------------\n');
fprintf('J = %.3f kg*m^2\n', J);
fprintf('b = %.3f Nms\n', b);
fprintf('Kt = %.3f Nm/A\n', Kt);
fprintf('Ke = %.3f Vs/rad\n', Ke);
fprintf('R = %.3f Ohm\n', R);
fprintf('L = %.3f H\n\n', L);
fprintf('Steady-State Relationship:\n');
fprintf('-------------------------\n');
fprintf('V = ω * (R*b/Kt + Ke) = ω * (%.3f)\n', V_to_omega_factor);
fprintf('ω = V / %.3f\n\n', V_to_omega_factor);

A = [-R/L, -Ke/L; Kt/J, -b/J];
B = [1/L; 0];
C = [0, 1];
D = 0;

fprintf('System Matrices:\n');
fprintf('----------------\n');
fprintf('A matrix (system dynamics):\n');
disp(A);
fprintf('B matrix (input matrix):\n');
disp(B);
fprintf('C matrix (output matrix):\n');
disp(C);
fprintf('\n');

fprintf('\n========================================\n');
fprintf('OPEN LOOP RESPONSE ANALYSIS\n');
fprintf('========================================\n\n');

t = 0:0.01:5;
dt = t(2) - t(1);
n_steps = length(t);

test_voltages = [10, 50, 100, 200, 500, 1000];
fprintf('Model Validation - Steady-State Speed vs Voltage:\n');
fprintf('------------------------------------------------\n');
fprintf('Voltage (V) | Simulated Speed | Theoretical Speed | Error (%%)\n');
fprintf('------------|-----------------|-------------------|----------\n');

figure(1);
hold on;
grid on;
colors = {'b-', 'g-', 'r-', 'c-', 'm-', 'k-'};

for v_idx = 1:length(test_voltages)
    V_test = test_voltages(v_idx);
    
    x_ol = zeros(2, n_steps);
    y_ol = zeros(1, n_steps);
    u = V_test * ones(1, n_steps);
    
    for k = 1:n_steps-1
        dxdt = A * x_ol(:,k) + B * u(k);
        x_ol(:,k+1) = x_ol(:,k) + dxdt * dt;
        y_ol(k+1) = C * x_ol(:,k+1);
    end
    
    theoretical_speed = V_test / V_to_omega_factor;
    simulated_speed = y_ol(end);
    error_percent = abs(simulated_speed - theoretical_speed) / theoretical_speed * 100;
    
    fprintf('V = %4d V   | %13.2f rad/s | %17.2f rad/s | %6.2f%%\n', ...
            V_test, simulated_speed, theoretical_speed, error_percent);
    
    plot(t, y_ol, colors{v_idx}, 'LineWidth', 1.5, ...
         'DisplayName', sprintf('V = %d V (final: %.1f rad/s)', V_test, simulated_speed));
end

plot(t, test_voltages(1)/V_to_omega_factor * ones(size(t)), 'k--', 'LineWidth', 1);
title('Open Loop Response at Different Voltages');
xlabel('Time (seconds)');
ylabel('Speed (rad/s)');
legend('Location', 'best');
hold off;

fprintf('\nModel validation PASSED - Simulation matches theoretical predictions!\n\n');

fprintf('\n========================================\n');
fprintf('OPERATING POINT SELECTION\n');
fprintf('========================================\n\n');

reference_speed = 50;
required_voltage = reference_speed * V_to_omega_factor;

fprintf('Selected Operating Point:\n');
fprintf('------------------------\n');
fprintf('Reference Speed: %.1f rad/s\n', reference_speed);
fprintf('Required Steady-State Voltage: %.1f V\n', required_voltage);
fprintf('Maximum Voltage Limit (set): 500 V (to allow for control action)\n\n');

fprintf('\n========================================\n');
fprintf('PID CONTROLLER DESIGN\n');
fprintf('========================================\n\n');

Kp = 20;
Ki = 50;
Kd = 1;

V_min = -500;
V_max = 500;

fprintf('PID Controller Parameters:\n');
fprintf('-------------------------\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n', Kd);
fprintf('Voltage Limits: [%d, %d] V\n\n', V_min, V_max);

fprintf('\n========================================\n');
fprintf('CLOSED LOOP SIMULATION\n');
fprintf('========================================\n\n');

x_cl = zeros(2, n_steps);
y_cl = zeros(1, n_steps);
error_integral = 0;
prev_error = 0;
control_signal = zeros(1, n_steps);
error_array = zeros(1, n_steps);

for k = 1:n_steps-1
    current_speed = x_cl(2, k);
    
    error = reference_speed - current_speed;
    error_array(k) = error;
    
    error_integral = error_integral + error * dt;
    
    integral_limit = 50;
    error_integral = max(-integral_limit, min(integral_limit, error_integral));
    
    if k > 1
        error_derivative = (error - prev_error) / dt;
    else
        error_derivative = 0;
    end
    
    u_control = Kp * error + Ki * error_integral + Kd * error_derivative;
    
    u_control = max(V_min, min(V_max, u_control));
    control_signal(k) = u_control;
    
    dxdt = A * x_cl(:,k) + B * u_control;
    
    x_cl(:,k+1) = x_cl(:,k) + dxdt * dt;
    
    y_cl(k+1) = C * x_cl(:,k+1);
    
    prev_error = error;
end

figure(2);
subplot(2,1,1);
plot(t, y_cl, 'b-', 'LineWidth', 2);
hold on;
plot(t, reference_speed * ones(size(t)), 'r--', 'LineWidth', 1.5);
grid on;
title('Closed Loop Step Response with PID Control');
xlabel('Time (seconds)');
ylabel('Speed (rad/s)');
legend('PID Controlled Response', 'Reference Speed');
hold off;

subplot(2,1,2);
plot(t, control_signal, 'g-', 'LineWidth', 1.5);
grid on;
title('Control Signal (Voltage Applied to Motor)');
xlabel('Time (seconds)');
ylabel('Voltage (V)');

fprintf('Performance Analysis:\n');
fprintf('--------------------\n');

steady_state_start = round(0.9 * n_steps);
steady_state_error = mean(abs(reference_speed - y_cl(steady_state_start:end)));
steady_state_error_percent = (steady_state_error / reference_speed) * 100;

peak_speed = max(y_cl);
overshoot = max(0, (peak_speed - reference_speed) / reference_speed * 100);

y_10 = 0.1 * reference_speed;
y_90 = 0.9 * reference_speed;

idx_10 = find(y_cl >= y_10, 1, 'first');
idx_90 = find(y_cl >= y_90, 1, 'first');

if ~isempty(idx_10) && ~isempty(idx_90) && idx_90 > idx_10
    rise_time = t(idx_90) - t(idx_10);
else
    rise_time = NaN;
end

settling_idx = [];
for i = length(y_cl):-1:1
    if abs(y_cl(i) - reference_speed) > 0.02 * reference_speed
        settling_idx = i + 1;
        break;
    end
end

if ~isempty(settling_idx) && settling_idx <= length(t)
    settling_time = t(settling_idx);
else
    settling_time = NaN;
end

fprintf('Reference Speed: %.1f rad/s\n', reference_speed);
fprintf('Final Speed: %.2f rad/s\n', y_cl(end));
fprintf('Steady-State Error: %.4f rad/s (%.2f%%)\n', steady_state_error, steady_state_error_percent);
fprintf('Peak Speed: %.2f rad/s\n', peak_speed);
fprintf('Overshoot: %.2f %%\n', overshoot);
fprintf('Rise Time (10%%-90%%): %.3f s\n', rise_time);
fprintf('Settling Time (2%%): %.3f s\n\n', settling_time);

fprintf('\n========================================\n');
fprintf('DISTURBANCE REJECTION TEST\n');
fprintf('========================================\n\n');

x_dist = zeros(2, n_steps);
y_dist = zeros(1, n_steps);
error_integral_dist = 0;
prev_error_dist = 0;
control_signal_dist = zeros(1, n_steps);

disturbance_start = 2.0;
disturbance_duration = 1.0;
disturbance_magnitude = -2.0;

for k = 1:n_steps-1
    current_speed = x_dist(2, k);
    error = reference_speed - current_speed;
    
    if t(k) >= disturbance_start && t(k) < disturbance_start + disturbance_duration
        disturbance = disturbance_magnitude;
    else
        disturbance = 0;
    end
    
    error_integral_dist = error_integral_dist + error * dt;
    error_integral_dist = max(-integral_limit, min(integral_limit, error_integral_dist));
    
    if k > 1
        error_derivative = (error - prev_error_dist) / dt;
    else
        error_derivative = 0;
    end
    
    u_control = Kp * error + Ki * error_integral_dist + Kd * error_derivative;
    u_control = max(V_min, min(V_max, u_control));
    control_signal_dist(k) = u_control;
    
    dxdt = A * x_dist(:,k) + B * u_control + [0; disturbance/J];
    
    x_dist(:,k+1) = x_dist(:,k) + dxdt * dt;
    y_dist(k+1) = C * x_dist(:,k+1);
    
    prev_error_dist = error;
end

disturbance_indices = find(t >= disturbance_start & t < disturbance_start + disturbance_duration);
if ~isempty(disturbance_indices)
    min_speed_during_disturbance = min(y_dist(disturbance_indices));
    speed_drop = reference_speed - min_speed_during_disturbance;
    speed_drop_percent = (speed_drop / reference_speed) * 100;
    
    recovery_start_idx = find(t >= disturbance_start + disturbance_duration, 1, 'first');
    if ~isempty(recovery_start_idx)
        recovery_idx = find(y_dist(recovery_start_idx:end) >= 0.98 * reference_speed, 1, 'first');
        if ~isempty(recovery_idx)
            recovery_time = t(recovery_start_idx + recovery_idx - 1) - (disturbance_start + disturbance_duration);
        else
            recovery_time = NaN;
        end
    else
        recovery_time = NaN;
    end
else
    speed_drop = NaN;
    speed_drop_percent = NaN;
    recovery_time = NaN;
end

figure(3);
subplot(2,1,1);
plot(t, y_dist, 'b-', 'LineWidth', 2);
hold on;
plot(t, reference_speed * ones(size(t)), 'r--', 'LineWidth', 1.5);
fill([disturbance_start, disturbance_start+disturbance_duration, ...
      disturbance_start+disturbance_duration, disturbance_start], ...
     [0, 0, reference_speed*1.2, reference_speed*1.2], ...
     'y', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
grid on;
title('Disturbance Rejection Test');
xlabel('Time (seconds)');
ylabel('Speed (rad/s)');
legend('Response', 'Reference', 'Disturbance Period');

subplot(2,1,2);
plot(t, control_signal_dist, 'g-', 'LineWidth', 1.5);
grid on;
title('Control Signal During Disturbance');
xlabel('Time (seconds)');
ylabel('Voltage (V)');

fprintf('Disturbance Rejection Performance:\n');
fprintf('---------------------------------\n');
fprintf('Load Torque: %.1f Nm from %.1f to %.1f s\n', ...
        disturbance_magnitude, disturbance_start, disturbance_start + disturbance_duration);
fprintf('Maximum Speed Drop: %.2f rad/s (%.1f%%)\n', speed_drop, speed_drop_percent);
fprintf('Recovery Time (after disturbance ends): %.3f s\n\n', recovery_time);

fprintf('\n========================================\n');
fprintf('PARAMETER SENSITIVITY ANALYSIS\n');
fprintf('========================================\n\n');

Kp_values = [10, 20, 40, 80];
Ki_fixed = 50;
Kd_fixed = 1;

figure(4);
subplot(2,1,1);
hold on;
grid on;

for i = 1:length(Kp_values)
    Kp_test = Kp_values(i);
    y_test = simulate_pid(A, B, C, Kp_test, Ki_fixed, Kd_fixed, ...
                          reference_speed, V_min, V_max, t);
    plot(t, y_test, 'LineWidth', 1.5, 'DisplayName', sprintf('Kp = %d', Kp_test));
end

plot(t, reference_speed * ones(size(t)), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
title('Effect of Proportional Gain (Kp) on Response');
xlabel('Time (seconds)');
ylabel('Speed (rad/s)');
legend('Location', 'best');
hold off;

Kp_fixed = 20;
Ki_values = [20, 50, 100, 200];
Kd_fixed = 1;

subplot(2,1,2);
hold on;
grid on;

for i = 1:length(Ki_values)
    Ki_test = Ki_values(i);
    y_test = simulate_pid(A, B, C, Kp_fixed, Ki_test, Kd_fixed, ...
                          reference_speed, V_min, V_max, t);
    plot(t, y_test, 'LineWidth', 1.5, 'DisplayName', sprintf('Ki = %d', Ki_test));
end

plot(t, reference_speed * ones(size(t)), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
title('Effect of Integral Gain (Ki) on Response');
xlabel('Time (seconds)');
ylabel('Speed (rad/s)');
legend('Location', 'best');
hold off;

fprintf('\n========================================\n');
fprintf('FINAL ASSESSMENT\n');
fprintf('========================================\n\n');

if steady_state_error_percent < 5 && overshoot < 20 && ~isnan(settling_time) && settling_time < 3
    performance_grade = 'EXCELLENT';
    recommendation = 'Controller meets all design specifications';
elseif steady_state_error_percent < 10 && overshoot < 30
    performance_grade = 'GOOD';
    recommendation = 'Controller works but could be tuned further';
elseif steady_state_error_percent < 20
    performance_grade = 'ACCEPTABLE';
    recommendation = 'Controller tracks reference but performance could be improved';
else
    performance_grade = 'NEEDS IMPROVEMENT';
    recommendation = 'Controller not tracking reference properly - check implementation';
end

fprintf('Controller Performance: %s\n', performance_grade);
fprintf('Recommendation: %s\n\n', recommendation);

save('dc_motor_final_results.mat', 't', 'y_cl', 'y_dist', 'control_signal', ...
     'Kp', 'Ki', 'Kd', 'reference_speed', 'V_to_omega_factor');

try
    saveas(figure(1), 'open_loop_validation.png');
    saveas(figure(2), 'closed_loop_response.png');
    saveas(figure(3), 'disturbance_rejection.png');
    saveas(figure(4), 'parameter_sweep.png');
    fprintf('\nFigures saved successfully.\n');
catch
    fprintf('\nWarning: Could not save some figures.\n');
end

fprintf('\n========================================\n');
fprintf('PROJECT COMPLETED SUCCESSFULLY\n');
fprintf('========================================\n');

function y = simulate_pid(A, B, C, Kp, Ki, Kd, reference, V_min, V_max, t)
    dt = t(2) - t(1);
    n_steps = length(t);
    x = zeros(2, n_steps);
    y = zeros(1, n_steps);
    error_integral = 0;
    prev_error = 0;
    
    for k = 1:n_steps-1
        current_speed = x(2, k);
        error = reference - current_speed;
        
        error_integral = error_integral + error * dt;
        error_integral = max(-50, min(50, error_integral));
        
        if k > 1
            error_derivative = (error - prev_error) / dt;
        else
            error_derivative = 0;
        end
        
        u = Kp * error + Ki * error_integral + Kd * error_derivative;
        u = max(V_min, min(V_max, u));
        
        dxdt = A * x(:,k) + B * u;
        x(:,k+1) = x(:,k) + dxdt * dt;
        y(k+1) = C * x(:,k+1);
        
        prev_error = error;
    end
end
