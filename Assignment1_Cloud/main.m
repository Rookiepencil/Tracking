%%% Initialization
clear all
close all
clc
p = parameters();
Sensor_Parameter = p.sensor;
dt = p.target(1).sampletime; % Sampling time
xk_1 = p.target(1).start_state; % Initial state of the target
Target_Process_Noise = p.target(1).process_noise;
Pk0= eye(4); % P stay the same
F = [1 dt 0 0;
     0 1  0 0;
     0 0  1 dt;
     0 0  0 1];
errors_per_time = zeros(1, p.scenario.num_of_time_steps); % Sum of squared errors per time step
counts_per_time = zeros(1, p.scenario.num_of_time_steps); % Count of valid estimates per time step
%% Main Loop
for r=1:p.scenario.monte_runs
     figure; % open the figure and make sure it can always update on that graph
     real_Target_Movement =[];
     Measurement = [];
     MeasurementConvert = [];
     R = [];
     grid on;
     hold on; 
     xlabel('X Position');
     ylabel('Y Position');
     title('Assignment 1 ECE 767');
     plot(Sensor_Parameter.Xpos, Sensor_Parameter.Ypos, 'k*', 'MarkerSize', 25); % Plot target Movement

     % xhat_k; % you will need mvmrnd to generate X update initial state
     %xk_1 = mvnrnd(zeros(4,1), eye(4))';
     real_Target_Movement =[real_Target_Movement, xk_1];
    for k=1:p.scenario.num_of_time_steps % This is the simulation step not the Actual time!!!!

        plot(real_Target_Movement(1,k), real_Target_Movement(3,k), 'b.', 'MarkerSize', 15); % Plot target Movement
        Measurement = generateMeasurements(Sensor_Parameter, real_Target_Movement(:,k));

        [xk, Q] = moveTarget(Target_Process_Noise, xk_1, dt);

        if k == 1
            x_pred = mvnrnd(zeros(4,1), eye(4))';  
            P_pred = Pk0;
        else
            x_pred = F * xk_hat;
            P_pred = F * Pk_hat * F' + Q;
        end
        
        xk_1 = xk;
        real_Target_Movement = [real_Target_Movement,xk];
        
        if isempty(Measurement)
            continue;
        end

        [MeasurementConvert, R] = convertMeasurement(Measurement, Sensor_Parameter);

        [asso_meas_ind,R_Update] = dataAssociation(p.tracker.gate_size, x_pred, P_pred, MeasurementConvert, R);
        disp(asso_meas_ind);

        if(asso_meas_ind ~= -1)
            z = MeasurementConvert(asso_meas_ind,:);
            plot(MeasurementConvert(asso_meas_ind,1), MeasurementConvert(asso_meas_ind,2), 'm.', 'MarkerSize', 15);
            [xk_hat, Pk_hat] = kalmanFilter(x_pred, P_pred, z, R_Update);
        else
            xk_hat = x_pred;
            Pk_hat = P_pred;
        end
        plot(xk_hat(1), xk_hat(3), 's','color', 'c', 'MarkerSize', 7); %Tracker
        
        estimated_position = [xk_hat(1);xk_hat(3)]; % Only [x, y] position
        true_position = [real_Target_Movement(1, k);real_Target_Movement(3, k)]; % True [x, y] position
        % error = norm(estimated_position - true_position); % Euclidean distance
        % errors_per_time(k) = errors_per_time(k) + error^2; % Accumulate squared error
        % counts_per_time(k) = counts_per_time(k) + 1; % Count valid estimates
    end

    hold off
    legend({'Sensor Position', 'Real', 'Measurement','Tracker'}, 'Location', 'best');
   
end
%% calculate the RMSE
% rmse_per_time = sqrt(errors_per_time ./ counts_per_time); % RMSE per time step
% overall_rmse = sqrt(mean(rmse_per_time .^ 2)); % Overall RMSE
% 
% % Display RMSE Results
% disp(['Overall RMSE for the scenario: ', num2str(overall_rmse)]);
% 
% %% Plot RMSE Results
% figure;
% plot(1:p.scenario.num_of_time_steps, rmse_per_time, '-o', 'LineWidth', 1.5);
% grid on;
% xlabel('Time Step');
% ylabel('RMSE');
% title('RMSE Over Time');
% legend('RMSE');

%% plot RMSE results



