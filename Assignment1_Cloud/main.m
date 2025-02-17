%%% Initialization
clear all;
close all;
% clc;

%rng(1);

% Parameters
p = parameters(); % Load scenario parameters
Pk0 = diag([250,40,250,40]); % Initial state covariance
Sensor_Parameter = p.sensor;
dt = p.target(1).sampletime; % Sampling time
Target_Process_Noise = p.target(1).process_noise;

% State Transition Matrix
F = [1 dt 0 0;
     0 1  0 0;
     0 0  1 dt;
     0 0  0 1];

% Initialize Error Tracking
errors_per_time = zeros(1, p.scenario.num_of_time_steps); % Accumulated squared errors per time step
counts_per_time = zeros(1, p.scenario.monte_runs); % Valid estimates count per time step

%% Main Monte Carlo Loop
for r = 1:p.scenario.monte_runs
    % Initialize for each Monte Carlo run
    xk_1 = p.target(1).start_state;
    real_Target_Movement = xk_1; % Initialize target movement
    Measurement = [];
    Sensor_Measurement = [];
    counts = 0;
    rmse_per_Monte = 0;
    
    % Open Figure for Visualization
    figure;
    hold on;
    grid on;
    xlabel('X Position');
    ylabel('Y Position');
    title(['Monte Carlo Run ', num2str(r)]);
    plot(Sensor_Parameter.Xpos, Sensor_Parameter.Ypos, 'k*', 'MarkerSize', 25); % Sensor position

    % Time Step Loop
    for k = 1:p.scenario.num_of_time_steps

        % Plot real target movement
        plot(real_Target_Movement(1, k), real_Target_Movement(3, k), 'b.', 'MarkerSize', 15);

        % Generate measurement
        Measurement = generateMeasurements(Sensor_Parameter, real_Target_Movement(:, k));

        % Target movement and process noise
        [xk, Q] = moveTarget(Target_Process_Noise, xk_1, dt);

        % Predict step
        if k == 1
            x_pred = mvnrnd(p.target(1).start_state, Pk0)'; % Add some initial noise
            P_pred = Pk0;
        else
            x_pred = F * xk_hat;
            P_pred = F * Pk_hat * F' + Q;
        end


        % Update real target movement
        xk_1 = xk;
        real_Target_Movement = [real_Target_Movement, xk];

        % Skip if no measurement
        if isempty(Measurement)
            continue;
        end

        % Convert measurement to expected format and calculate R
        [MeasurementConvert, R] = convertMeasurement(Measurement, Sensor_Parameter);

        % Data association
        [asso_meas_ind, R_Update] = dataAssociation(p.tracker.gate_size, x_pred, P_pred, MeasurementConvert, R);

        % Kalman filter update
        if asso_meas_ind ~= -1 % If a measurement was associated we do filtering
            z = MeasurementConvert(asso_meas_ind, :); % Associated Measurement
            Sensor_Measurement = [Sensor_Measurement; z];
            plot(z(1), z(2), 'm.', 'MarkerSize', 15); % Plot measurement
            [xk_hat, Pk_hat] = kalmanFilter(x_pred, P_pred, z, R_Update);
        else
            Sensor_Measurement = [Sensor_Measurement; nan(1, 2)]; % Mark as unassociated
            xk_hat = x_pred;
            Pk_hat = P_pred;
        end

        % Plot tracker
        plot(xk_hat(1), xk_hat(3), 's', 'color', 'c', 'MarkerSize', 7);

        % Calculate error if associated
        if asso_meas_ind ~= -1
            estimated_position_x = xk_hat(1); % Estimated X position
            true_position = real_Target_Movement(1, k); % True position
            
            error = estimated_position_x - true_position; % Euclidean distance
            errors_per_time(r, k) = error.^2; 
            counts_per_time(1, r) = counts_per_time(1,r) + 1;

        end
        
    end

    % Finalize visualization
    hold off;
    legend({'Sensor Position', 'Real Target', 'Measurement', 'Tracker'}, 'Location', 'best');
end

%% Calculate RMSE
result = zeros(1, size(errors_per_time, 2));
for row = 1:size(errors_per_time, 2)

    col_sum = sum(errors_per_time(:, row));

    col_mean = col_sum / size(errors_per_time, 1);

    result(row) = sqrt(col_mean);
end
figure
plot(result)


