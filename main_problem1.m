% % function main_problem1()
% % clc; close all;
% % 
% % % ------------------------
% % % Assumptions and constants
% % % ------------------------
% % g = 9.81;                 % m/s^2
% % dt = 0.001;               % s, controller timestep
% % T  = 12;                  % s, total sim time
% % t  = (0:dt:T)';           % time vector
% % 
% % % Roll plant parameters (simple second order)
% % plant.Iphi = 900;         % kg*m^2  (tuneable)
% % plant.cphi = 1800;        % N*m*s/rad
% % plant.kphi = 12000;       % N*m/rad
% % 
% % % Actuator limits (torque about roll axis)
% % act.u_max = 6000;         % N*m (saturation)
% % act.du_max = 80000;       % N*m/s (rate limit), optional
% % act.du_max_step = act.du_max * dt;   % N*m per timestep
% % 
% % 
% % % Controller gains (start conservative, then tune)
% % % ctrl.Kp = 14000;          % N*m/rad
% % % ctrl.Kd = 3000;           % N*m*s/rad
% % ctrl.Kp = 4000;
% % ctrl.Kd = 800;
% % ctrl.use_rate_limit = true;
% % 
% % % Sensor noise settings (for ay measurement)
% % noise.enable = true;
% % noise.sigma_ay = 0.25;    % m/s^2 (tune)
% % 
% % % ------------------------
% % % Create test inputs
% % % ------------------------
% % tests = make_test_inputs(t);
% % 
% % % ------------------------
% % % Run all tests
% % % ------------------------
% % for k = 1:numel(tests)
% %     in = tests(k);
% % 
% %     % True lateral acceleration profile
% %     ay_true = in.ay_profile;
% % 
% %     % Measured acceleration with noise
% %     if noise.enable
% %         ay_meas = ay_true + noise.sigma_ay * randn(size(ay_true));
% %     else
% %         ay_meas = ay_true;
% %     end
% % 
% %     % Initialize states
% %     x.phi = 0;            % rad
% %     x.phi_dot = 0;        % rad/s
% %     u_prev = 0;
% % 
% %     % Log arrays
% %     phi     = zeros(size(t));
% %     phi_dot = zeros(size(t));
% %     u_log   = zeros(size(t));
% %     u_cmd_log = zeros(size(t));
% %     phi_des = zeros(size(t));
% % 
% %     % Optional: profile controller + plant
% %     % profile on
% % 
% %     for i = 1:numel(t)
% %         % Desired lean from measured ay 
% %         phi_des(i) = atan2(ay_meas(i), g);   % This is the core physical equation being tracked
% % 
% %         % Low-pass filter parameters
% %         fc = 5;                       % cutoff frequency (Hz)
% %         alpha = exp(-2*pi*fc*dt);
% % 
% %         if i == 1
% %             phi_des_dot_filt = 0;
% %         else
% %             phi_des_dot_raw = (phi_des(i) - phi_des(i-1)) / dt;
% %             phi_des_dot_filt = alpha * phi_des_dot_filt + (1-alpha) * phi_des_dot_raw;
% %         end
% % 
% % 
% %         % Approx desired lean rate (finite difference)
% %         if i == 1
% %             phi_des_dot = 0;
% %         else
% %             phi_des_dot = (phi_des(i) - phi_des(i-1)) / dt;
% %         end
% % 
% %         % Controller computes actuator torque
% %         % [u_cmd, u_limited] = lean_controller( ...
% %         %     phi_des(i), phi_des_dot, x.phi, x.phi_dot, ...
% %         %     u_prev, ctrl, act);
% %         [u_cmd, u_limited] = lean_controller( ...
% %             phi_des(i), phi_des_dot, x.phi, x.phi_dot, ...
% %             u_prev, ctrl, act, plant, dt);
% % 
% % 
% %         % Step plant forward
% %         x = roll_plant_step(x, u_limited, plant, dt);
% % 
% %         % Save logs
% %         phi(i)     = x.phi;
% %         phi_dot(i) = x.phi_dot;
% %         u_log(i)   = u_limited;
% %         u_cmd_log(i) = u_cmd;
% %         u_prev = u_limited;
% %     end
% % 
% %     % profile off
% %     % p = profile('info'); profsave(p, fullfile(pwd, ['profile_test_' num2str(k)]));
% % 
% %     % ------------------------
% %     % Plot results
% %     % ------------------------
% %     figure('Name', ['Test ' num2str(k) ': ' in.name], 'Color', 'w');
% % 
% %     subplot(3,1,1);
% %     plot(t, ay_true, 'LineWidth', 1.5); hold on;
% %     plot(t, ay_meas, 'LineWidth', 1.0);
% %     grid on;
% %     ylabel('a_y (m/s^2)');
% %     legend('true', 'measured');
% % 
% %     subplot(3,1,2);
% %     plot(t, phi_des, 'LineWidth', 1.5); hold on;
% %     plot(t, phi, 'LineWidth', 1.5);
% %     grid on;
% %     ylabel('\phi (rad)');
% %     legend('\phi_{des}', '\phi');
% % 
% %     subplot(3,1,3);
% %     plot(t, u_cmd_log, 'LineWidth', 1.2); hold on;
% %     plot(t, u_log, 'LineWidth', 1.5);
% %     grid on;
% %     ylabel('Torque (N*m)');
% %     xlabel('time (s)');
% %     legend('u_{cmd} (raw)', 'u (saturated)');
% % 
% % 
% %     % Simple metrics
% %     rmse = sqrt(mean((phi - phi_des).^2));
% %     fprintf('Test %d (%s): phi RMSE = %.4f rad (%.2f deg)\n', ...
% %         k, in.name, rmse, rmse * 180/pi);
% % end
% % end
% % 
% % 
% 
% 
% function main_problem1()
% clc; close all;
% 
% % ------------------------
% % Assumptions and constants
% % ------------------------
% g  = 9.81;          % m/s^2
% dt = 0.001;         % s
% T  = 12;            % s
% t  = (0:dt:T)';     % time vector
% 
% % Roll plant parameters (2nd order)
% plant.Iphi = 900;         % kg*m^2
% plant.cphi = 1800;        % N*m*s/rad
% plant.kphi = 12000;       % N*m/rad
% 
% % Actuator limits
% act.u_max = 6000;         % N*m
% act.du_max = 80000;       % N*m/s
% act.du_max_step = act.du_max * dt;   % N*m per timestep
% 
% % Controller gains
% ctrl.Kp = 4000;
% ctrl.Kd = 800;
% ctrl.use_rate_limit = true;
% 
% % Sensor noise
% noise.enable = true;
% noise.sigma_ay = 0.25;    % m/s^2
% 
% % Low-pass filter for reference derivative (critical!)
% fc = 2;                          % Hz (use low cutoff to suppress noise)
% alpha = exp(-2*pi*fc*dt);        % filter coefficient
% 
% % ------------------------
% % Create test inputs
% % ------------------------
% tests = make_test_inputs(t);
% 
% % Optional: make noise repeatable
% rng(1);
% 
% % ------------------------
% % Run all tests
% % ------------------------
% 
% for k = 1:numel(tests)
%     in = tests(k);
% 
%     % True lateral acceleration profile
%     ay_true = in.ay_profile;
% 
%     % Measured acceleration with noise
%     if noise.enable
%         ay_meas = ay_true + noise.sigma_ay * randn(size(ay_true));
%     else
%         ay_meas = ay_true;
%     end
% 
%     % Initialize roll states
%     x.phi = 0;         % rad
%     x.phi_dot = 0;     % rad/s
%     u_prev = 0;
% 
%     % Logs
%     phi         = zeros(size(t));
%     phi_dot     = zeros(size(t));
%     phi_des     = zeros(size(t));
%     u_log       = zeros(size(t));
%     u_cmd_log   = zeros(size(t));
%     phi_des_dot_log = zeros(size(t));
% 
%     % Initialize filter state (per test)
%     phi_des_dot_filt = 0;
% 
%     % ------------------------
%     % Main simulation loop
%     % ------------------------
%     for i = 1:numel(t)
% 
%         % Desired lean from measured ay (core physics)
%         phi_des(i) = atan2(ay_meas(i), g);
% 
%         % Desired lean rate (raw finite difference)
%         if i == 1
%             phi_des_dot_raw = 0;
%         else
%             phi_des_dot_raw = (phi_des(i) - phi_des(i-1)) / dt;
%         end
% 
%         % Filter desired lean rate (THIS is what we should use)
%         phi_des_dot_filt = alpha * phi_des_dot_filt + (1 - alpha) * phi_des_dot_raw;
%         phi_des_dot_log(i) = phi_des_dot_filt;
% 
%         % Controller (use filtered reference derivative)
%         [u_cmd, u_limited] = lean_controller( ...
%             phi_des(i), phi_des_dot_filt, x.phi, x.phi_dot, ...
%             u_prev, ctrl, act, plant, dt);
% 
%         % Plant update
%         x = roll_plant_step(x, u_limited, plant, dt);
% 
%         % Log
%         phi(i)       = x.phi;
%         phi_dot(i)   = x.phi_dot;
%         u_log(i)     = u_limited;
%         u_cmd_log(i) = u_cmd;
% 
%         u_prev = u_limited;
%     end
% 
%     % ------------------------
%     % Plot results
%     % ------------------------
%     figure('Name', ['Test ' num2str(k) ': ' in.name], 'Color', 'w');
% 
%     subplot(3,1,1);
%     plot(t, ay_true, 'LineWidth', 1.5); hold on;
%     plot(t, ay_meas, 'LineWidth', 1.0);
%     grid on;
%     ylabel('a_y (m/s^2)');
%     legend('true', 'measured', 'Location', 'best');
% 
%     subplot(3,1,2);
%     plot(t, phi_des, 'LineWidth', 1.5); hold on;
%     plot(t, phi, 'LineWidth', 1.5);
%     grid on;
%     ylabel('\phi (rad)');
%     legend('\phi_{des}', '\phi', 'Location', 'best');
% 
%     subplot(3,1,3);
%     plot(t, u_cmd_log, 'LineWidth', 1.0); hold on;
%     plot(t, u_log, 'LineWidth', 1.5);
%     grid on;
%     ylabel('Torque (N*m)');
%     xlabel('time (s)');
%     legend('u_{cmd} (raw)', 'u (saturated)', 'Location', 'best');
% 
%     % Print metric
%     rmse = sqrt(mean((phi - phi_des).^2));
%     fprintf('Test %d (%s): phi RMSE = %.4f rad (%.2f deg)\n', ...
%         k, in.name, rmse, rmse * 180/pi);
% 
%     peak_torque = max(abs(u_log));
%     fprintf('   Peak applied torque = %.1f N*m\n', peak_torque);
% 
% 
%     % Optional: sanity print for saturation percentage
%     sat_frac = mean(abs(u_log) >= (0.999 * act.u_max)) * 100;
%     fprintf('   Saturation fraction: %.1f%% of timesteps\n', sat_frac);
% end
% end



function main_problem1()
clc; close all;

% ------------------------
% Assumptions and constants
% ------------------------
g  = 9.81;          % m/s^2
dt = 0.001;         % s
T  = 12;            % s
t  = (0:dt:T)';     % time vector

% Roll plant parameters (2nd order)
plant.Iphi = 900;         % kg*m^2
plant.cphi = 1800;        % N*m*s/rad
plant.kphi = 12000;       % N*m/rad

% Actuator limits
act.u_max = 6000;         % N*m
act.du_max = 80000;       % N*m/s
act.du_max_step = act.du_max * dt;   % N*m per timestep

% Controller gains
ctrl.Kp = 4000;
ctrl.Kd = 800;
ctrl.use_rate_limit = true;

% Sensor noise
noise.enable = true;
noise.sigma_ay = 0.25;    % m/s^2

% Low-pass filter for reference derivative (critical!)
fc = 2;                          % Hz
alpha = exp(-2*pi*fc*dt);        % filter coefficient

% ------------------------
% Create test inputs
% ------------------------
tests = make_test_inputs(t);

% Optional: make noise repeatable
rng(1);

% ------------------------
% Profiling settings
% ------------------------
profile_test_idx = 2;  % profile only Test 2 (smooth turn). Change if you want.
show_profile_viewer = true;

% ------------------------
% Run all tests
% ------------------------
for k = 1:numel(tests)
    in = tests(k);

    % True lateral acceleration profile
    ay_true = in.ay_profile;

    % Measured acceleration with noise
    if noise.enable
        ay_meas = ay_true + noise.sigma_ay * randn(size(ay_true));
    else
        ay_meas = ay_true;
    end

    % Initialize roll states
    x.phi = 0;         % rad
    x.phi_dot = 0;     % rad/s
    u_prev = 0;

    % Logs
    phi         = zeros(size(t));
    phi_dot     = zeros(size(t));
    phi_des     = zeros(size(t));
    u_log       = zeros(size(t));
    u_cmd_log   = zeros(size(t));
    phi_des_dot_log = zeros(size(t));

    % Initialize filter state (per test)
    phi_des_dot_filt = 0;

    % ------------------------
    % Main simulation loop (PROFILE THIS SECTION)
    % ------------------------
    do_profile = (k == profile_test_idx);

    if do_profile
        profile clear
        profile on
        tic; % wall-clock timing for the loop section
    end

    for i = 1:numel(t)

        % Desired lean from measured ay (core physics)
        phi_des(i) = atan2(ay_meas(i), g);

        % Desired lean rate (raw finite difference)
        if i == 1
            phi_des_dot_raw = 0;
        else
            phi_des_dot_raw = (phi_des(i) - phi_des(i-1)) / dt;
        end

        % Filter desired lean rate
        phi_des_dot_filt = alpha * phi_des_dot_filt + (1 - alpha) * phi_des_dot_raw;
        phi_des_dot_log(i) = phi_des_dot_filt;

        % Controller (use filtered reference derivative)
        [u_cmd, u_limited] = lean_controller( ...
            phi_des(i), phi_des_dot_filt, x.phi, x.phi_dot, ...
            u_prev, ctrl, act, plant, dt);

        % Plant update
        x = roll_plant_step(x, u_limited, plant, dt);

        % Log
        phi(i)       = x.phi;
        phi_dot(i)   = x.phi_dot;
        u_log(i)     = u_limited;
        u_cmd_log(i) = u_cmd;

        u_prev = u_limited;
    end

    if do_profile
        loop_time = toc;
        profile off
        pinfo = profile('info');

        fprintf('\n================ PROFILING SUMMARY ================\n');
        fprintf('Profiled Test %d (%s)\n', k, in.name);
        fprintf('Loop wall-clock time: %.6f s for %d steps (dt=%.4g s)\n', ...
            loop_time, numel(t), dt);
        fprintf('Average time per step: %.3e s (%.2f kHz max rate)\n', ...
            loop_time/numel(t), (numel(t)/loop_time)/1000);

        % Print top hotspots by total time
        ft = pinfo.FunctionTable;
        [~, idx] = sort([ft.TotalTime], 'descend');
        topN = min(8, numel(idx));

        fprintf('\nTop %d hotspots (by total time):\n', topN);
        fprintf('%-40s %-12s %-12s\n', 'Function', 'TotalTime(s)', 'NumCalls');
        for j = 1:topN
            f = ft(idx(j));
            fprintf('%-40s %-12.6f %-12d\n', f.FunctionName, f.TotalTime, f.NumCalls);
        end
        fprintf('===================================================\n\n');

        if show_profile_viewer
            profile viewer
        end
    end

    % ------------------------
    % Plot results (NOT profiled)
    % ------------------------
    figure('Name', ['Test ' num2str(k) ': ' in.name], 'Color', 'w');

    subplot(3,1,1);
    plot(t, ay_true, 'LineWidth', 1.5); hold on;
    plot(t, ay_meas, 'LineWidth', 1.0);
    grid on;
    ylabel('a_y (m/s^2)');
    legend('true', 'measured', 'Location', 'best');

    subplot(3,1,2);
    plot(t, phi_des, 'LineWidth', 1.5); hold on;
    plot(t, phi, 'LineWidth', 1.5);
    grid on;
    ylabel('\phi (rad)');
    legend('\phi_{des}', '\phi', 'Location', 'best');

    subplot(3,1,3);
    plot(t, u_cmd_log, 'LineWidth', 1.0); hold on;
    plot(t, u_log, 'LineWidth', 1.5);
    grid on;
    ylabel('Torque (N*m)');
    xlabel('time (s)');
    legend('u_{cmd} (raw)', 'u (saturated)', 'Location', 'best');

    % Print metric
    rmse = sqrt(mean((phi - phi_des).^2));
    fprintf('Test %d (%s): phi RMSE = %.4f rad (%.2f deg)\n', ...
        k, in.name, rmse, rmse * 180/pi);

    peak_torque = max(abs(u_log));
    fprintf('   Peak applied torque = %.1f N*m\n', peak_torque);

    sat_frac = mean(abs(u_log) >= (0.999 * act.u_max)) * 100;
    fprintf('   Saturation fraction: %.1f%% of timesteps\n', sat_frac);
end
end
