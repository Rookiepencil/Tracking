%%% Initialization
clear all;
close all;

% rng(1);

% Parameters
p = parameters(); % Load scenario parameters
Pk0 = diag([30^2,10^2,30^2,10^2]); % Initial state covariance
Sensor_Parameter = p.sensor;
dt = p.target(1).sampletime; % Sampling time
Target_Process_Noise = p.target(1).process_noise;
plots_Wants_to_See = 1;
total_Valid_Track = zeros(p.scenario.monte_runs,p.scenario.num_of_time_steps);
% Initialize Error Tracking
errors_per_time = zeros(1, p.scenario.num_of_time_steps); % Accumulated squared errors for position per time step
errors_per_time_Velocity = zeros(1, p.scenario.num_of_time_steps); %Accumulated squared errors for Velocity per time step
counts_per_time = zeros(1, p.scenario.monte_runs); % Valid estimates count per time step
RMSE_result = zeros(1, size(errors_per_time, 2));
RMSE_Velocity_result = zeros(1, size(errors_per_time, 2));
time_vector = (0:p.scenario.num_of_time_steps - 1) * dt;

% State Transition Matrix
F = [1 dt 0 0;
     0 1  0 0;
     0 0  1 dt;
     0 0  0 1];

%% Main Monte Carlo Loop
for r = 1:p.scenario.monte_runs
    % Initialize for each Monte Carlo run
    xk_1 = p.target(1).start_state;
    real_Target_Movement = xk_1; % Initialize target movement
    Measurement = [];
    all_Measurement_WithFalse =[];
    Sensor_Measurement = [];
    counts = 0;
    rmse_per_Monte = 0;
    valid_track = zeros(1,p.scenario.num_of_time_steps);
    
    % Open Figure for Visualization
    if p.scenario.monte_runs - r < plots_Wants_to_See
        figure;
        hold on;
        grid on;
        xlabel('X Position');
        ylabel('Y Position');
        title('Truth & Measurement with KF');
        plot(Sensor_Parameter.Xpos, Sensor_Parameter.Ypos, 'k*', 'MarkerSize', 25); % Plot Sensor Plot
    end

    % Time Step Loop
    for k = 1:p.scenario.num_of_time_steps

        % Plot real target movement
      if p.scenario.monte_runs -r < plots_Wants_to_See
        plot(real_Target_Movement(1, k), real_Target_Movement(3, k), 'b.', 'MarkerSize', 15);
      end

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
        all_Measurement_WithFalse = [all_Measurement_WithFalse;MeasurementConvert];

        % Data association
        [asso_meas_ind, R_Update] = dataAssociation(p.tracker.gate_size, x_pred, P_pred, MeasurementConvert, R);

        % Kalman filter update
        if asso_meas_ind ~= -1 % If a measurement was associated we do filtering
            z = MeasurementConvert(asso_meas_ind, :); % Associated Measurement
            Sensor_Measurement = [Sensor_Measurement; z];

            if p.scenario.monte_runs -r < plots_Wants_to_See
               % for M = 1:size(all_Measurement_WithFalse, 1)
               %   plot(all_Measurement_WithFalse(:,1), all_Measurement_WithFalse(:,2), 'm.', 'MarkerSize', 15);
               % end
               plot(z(1), z(2), 'm.', 'MarkerSize', 15); % Plot measurement
          
            % elseif p.scenario.monte_runs -r < plots_Wants_to_See
            %     plot(z(1), z(2), 'm.', 'MarkerSize', 15); % Plot measurement
            end

            [xk_hat, Pk_hat] = kalmanFilter(x_pred, P_pred, z, R_Update);
        else
            Sensor_Measurement = [Sensor_Measurement; nan(1, 2)]; % Mark as unassociated
            xk_hat = x_pred;
            Pk_hat = P_pred;
            continue;
        end

        % Plot tracker
        if p.scenario.monte_runs -r < plots_Wants_to_See
            plot(xk_hat(1), xk_hat(3), 's', 'color', 'c', 'MarkerSize', 7);
        end

        %Actual Error Calculation
            estimated_velocity_xy = [xk_hat(2);xk_hat(4)]; %Estimated XY Velocity
            estimated_position_xy = [xk_hat(1);xk_hat(3)]; %Estimated XY position
            
            true_velocity_xy = [real_Target_Movement(2, k);real_Target_Movement(4, k)];% True XY Velocity
            true_position_xy = [real_Target_Movement(1, k);real_Target_Movement(3, k)];% True XY position
            
            error_xy_velocity = vecnorm(true_velocity_xy - estimated_velocity_xy, 2, 2);
            error_xy = vecnorm(true_position_xy - estimated_position_xy, 2, 2);
            error_xy = norm(error_xy);
            error_xy_velocity = norm(error_xy_velocity);


        % Calculate error if associated
        if  error_xy < p.perf_eval.gate_size
            errors_per_time_Velocity(r,k) = error_xy_velocity.^2; 
            errors_per_time(r, k) = error_xy.^2;
            valid_track(k) = 1;
        else
            errors_per_time(r, k) = 0;
            errors_per_time_Velocity(r,k) = 0;
            valid_track(k) = 0;
        end
        
    end

    total_Valid_Track(r,:) = total_Valid_Track(r,:) + valid_track;
    % Finalize visualization
    if p.scenario.monte_runs -r < plots_Wants_to_See
        hold off;
        legend({'Sensor Position', 'Real Target', 'Measurement', 'Tracker'}, 'Location', 'best');
    end
end

%% Calculate overall Distance RMSE
total_Valid_Track = sum(total_Valid_Track,1);
for k = 1:size(errors_per_time, 2)

    col_sum = sum(errors_per_time(:, k));
    col_sum_Velocity = sum(errors_per_time_Velocity(:, k));
    %col_mean = col_sum / size(errors_per_time, 1);
    col_mean = col_sum / total_Valid_Track(k);
    col_mean_Velocity = col_sum_Velocity / size(errors_per_time_Velocity, 1);
    % col_mean = col_sum / p.total_valid_run(k);

    RMSE_result(k) = sqrt(col_mean);
    RMSE_Velocity_result(k) = sqrt(col_mean_Velocity);
end
%% Plot Position XY RMSE
figure
plot(time_vector,RMSE_result,'b-',LineWidth=1.5);
xlabel('Time (s)');
ylabel('RMSE of Distance');
title('RMSE of Distance vs. Time');
grid on;
%% Plot Velocity XY RMSE
figure
plot(time_vector,RMSE_Velocity_result,'r-',LineWidth=1.5);
xlabel('Time (s)');
ylabel('RMSE of Velocity');
title('RMSE of Velocity vs. Time');
grid on;
%% Plot Valid Run
figure
plot(time_vector,total_Valid_Track,'k-',LineWidth=1.5);
xlabel('Time (s)');
ylabel('Number of Valid Run');
title('Total Valid Track vs. Time');
grid on;