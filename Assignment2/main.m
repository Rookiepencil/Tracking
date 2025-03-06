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
    % real_Target1_Movement = Target_1_Start_State; % Initialize target1 movement
    real_Target1_Movement =[];
    real_Target2_Movement =[];
    % real_Target2_Movement = Target_2_Start_State; % Initialize target2 movement
    Measurement = [];
    all_Measurement_T1 =[];
    all_Measurement_T2 =[];
    Sensor_Measurement = [];
    TargetPred =[];
    TargetP_pred =[];
    counts = 0;
    rmse_per_Monte = 0;
    %valid_track = zeros(1,p.scenario.num_of_time_steps);

    % Time Step Loop
    for k = 1:dt:p.scenario.num_of_time
    %% Target Generation
        %time = (k-1)*dt
       if k >= 5 && k <= 30
          if k > 5
            [Target_1_Next_State, Target1_Q] = moveTarget(Target1_Process_Noise, Target_1_Start_State, dt);
            real_Target1_Movement = [real_Target1_Movement,Target_1_Next_State];
            Target_1_Start_State = Target_1_Next_State;
          else
            real_Target1_Movement = [real_Target1_Movement,Target_1_Start_State];
          end
       else
           real_Target1_Movement = [real_Target1_Movement,nan(4,1)];
       end
    
       if k >= 15 && k <= 40
           if k > 15
             [Target_2_Next_State, Target2_Q] = moveTarget(Target2_Process_Noise, Target_2_Start_State, dt);
             real_Target2_Movement = [real_Target2_Movement,Target_2_Next_State];
             Target_2_Start_State = Target_2_Next_State;
           else
              real_Target2_Movement = [real_Target2_Movement,Target_2_Start_State];
           end
       else
         real_Target2_Movement = [real_Target2_Movement,nan(4,1)];
       end
     %% Measurement Generation
     [MeasurementT1, MeasurementT2] = generateMeasurements(Sensor_Parameter, real_Target1_Movement(:, k), real_Target2_Movement(:, k));
     all_Measurement_T1 = [all_Measurement_T1; MeasurementT1];  
     all_Measurement_T2 = [all_Measurement_T2; MeasurementT2];  
     %% Track Initialize
            if k == 5
              p.tracker(1).Status = "Tentative";
              p.tracker(1).Score = 0;
              p.tracker(1).P = eye(4)*300;
              p.tracker(1).State = mvnrnd(p.target(1).start_state, p.tracker(1).P)';
            end

            if k == 15
              p.tracker(2).Status = "Tentative";
              p.tracker(2).Score = 0;
              p.tracker(2).P = eye(4)*300;
              p.tracker(2).State = mvnrnd(p.target(2).start_state, p.tracker(2).P)';
            end
      %% Prediction & PDA
            if k <= 5 || k <= 15 %track initialization I can not do any prediction
                
            else %I can do prediction and PDA
                for numTrack = 1:length(p.tracker)
                    %[x_pred, P_pred] = PredictEKF(p.tracker(numTrack).State, p.tracker(numTrack).P, F, Q);
                    %p.tracker(numTrack).State = x_pred;
                    %p.tracker(numTrack).P = P_pred;
                   % [x_update, P_update] = PDAEKF(p.tracker(numTrack),Sensor_Parameter);
                end

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
for plotstep = 1:size(real_Target1_Movement,2)
    plot(real_Target1_Movement(1, plotstep), real_Target1_Movement(3, plotstep), 'b.', 'MarkerSize', 15);
    plot(real_Target2_Movement(1, plotstep), real_Target2_Movement(3, plotstep), 'b.', 'MarkerSize', 15);
end

%% Performance Evalution