%%% Initialization
clear all;
close all;

% rng(1);

% Parameters
p = parameters(); % Load scenario parameters
Pk0 = diag([30^2,10^2,30^2,10^2]); % Initial state covariance
Sensor_Parameter = p.sensor;
dt = p.target(1).sampletime; % Sampling time
Target1_Process_Noise = p.target(1).process_noise;
Target2_Process_Noise = p.target(2).process_noise;
plots_Wants_to_See = 1;
%total_Valid_Track = zeros(p.scenario.monte_runs,p.scenario.num_of_time_steps);

% % Initialize Error Tracking
% errors_per_time = zeros(1, p.scenario.num_of_time_steps); % Accumulated squared errors for position per time step
% errors_per_time_Velocity = zeros(1, p.scenario.num_of_time_steps); %Accumulated squared errors for Velocity per time step
% counts_per_time = zeros(1, p.scenario.monte_runs); % Valid estimates count per time step
% RMSE_result = zeros(1, size(errors_per_time, 2));
% RMSE_Velocity_result = zeros(1, size(errors_per_time, 2));
% %time_vector = (0:p.scenario.num_of_time_steps - 1) * dt;

% State Transition Matrix
F = [1 dt 0 0;
     0 1  0 0;
     0 0  1 dt;
     0 0  0 1];

%% Main Monte Carlo Loop
for r = 1:p.scenario.monte_runs
    % Initialize for each Monte Carlo run
    Target_1_Start_State = p.target(1).start_state;
    Target_2_Start_State = p.target(2).start_state;
    real_Target1_Movement = Target_1_Start_State; % Initialize target1 movement
    real_Target2_Movement = Target_2_Start_State; % Initialize target2 movement
    Measurement = [];
    all_Measurement_WithFalse =[];
    Sensor_Measurement = [];
    counts = 0;
    rmse_per_Monte = 0;
    %valid_track = zeros(1,p.scenario.num_of_time_steps);

    % Time Step Loop
    for k = 1:dt:p.scenario.num_of_time
       if k >= 5 && k <= 30
          [Target_1_Next_State, Target1_Q] = moveTarget(Target1_Process_Noise, Target_1_Start_State, dt);
          real_Target1_Movement = [real_Target1_Movement,Target_1_Next_State];
          Target_1_Start_State = Target_1_Next_State;
       end
    

       if k >=15 && k <= 40
         [Target_2_Next_State, Target2_Q] = moveTarget(Target2_Process_Noise, Target_2_Start_State, dt);
         real_Target2_Movement = [real_Target2_Movement,Target_2_Next_State];
         Target_2_Start_State = Target_2_Next_State;
       end
        

       
       
       
       
     
       
    end

    
end
%% Plotting Section
figure;
hold on
grid on;
xlabel('X Position');
ylabel('Y Position');
title('Assignment 2');
plot(Sensor_Parameter.Xpos, Sensor_Parameter.Ypos, 'g*', 'MarkerSize', 25); % Plot Sensor Plot
plot(real_Target1_Movement(1, k), real_Target1_Movement(3, k), 'b.', 'MarkerSize', 15);
plot(real_Target2_Movement(1, k), real_Target2_Movement(3, k), 'b.', 'MarkerSize', 15);

%% RMSE



% %% Calculate overall Distance RMSE
% total_Valid_Track = sum(total_Valid_Track,1);
% for k = 1:size(errors_per_time, 2)
% 
%     col_sum = sum(errors_per_time(:, k));
%     col_sum_Velocity = sum(errors_per_time_Velocity(:, k));
%     %col_mean = col_sum / size(errors_per_time, 1);
%     col_mean = col_sum / total_Valid_Track(k);
%     col_mean_Velocity = col_sum_Velocity / size(errors_per_time_Velocity, 1);
%     % col_mean = col_sum / p.total_valid_run(k);
% 
%     RMSE_result(k) = sqrt(col_mean);
%     RMSE_Velocity_result(k) = sqrt(col_mean_Velocity);
% end
% %% Plot Position XY RMSE
% figure
% plot(time_vector,RMSE_result,'b-',LineWidth=1.5);
% xlabel('Time (s)');
% ylabel('RMSE of Distance');
% title('RMSE of Distance vs. Time');
% grid on;
% %% Plot Velocity XY RMSE
% figure
% plot(time_vector,RMSE_Velocity_result,'r-',LineWidth=1.5);
% xlabel('Time (s)');
% ylabel('RMSE of Velocity');
% title('RMSE of Velocity vs. Time');
% grid on;
% %% Plot Valid Run
% figure
% plot(time_vector,total_Valid_Track,'k-',LineWidth=1.5);
% xlabel('Time (s)');
% ylabel('Number of Valid Run');
% title('Total Valid Track vs. Time');
% grid on;