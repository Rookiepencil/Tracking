%%% Initialization
clear all;
close all;

rng(1);

% Parameters
p = parameters(); % Load scenario parameters
Pk0 = diag([30^2,10^2,30^2,10^2]); % Initial state covariance
Sensor_Parameter = p.sensor;
dt = p.target(1).sampletime; % Sampling time
Target1_Process_Noise = p.target(1).process_noise;
Target2_Process_Noise = p.target(2).process_noise;
TrackerGate = p.tracker.gate_size;
N_tent = p.tracker.Ntent;
M_tent = p.tracker.Mtent;
N_conf = p.tracker.Nconf;
M_conf = p.tracker.Mconf;

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
    Measurement_Index_Total = [];
    counts = 0;
    rmse_per_Monte = 0;
    Tracks = [];
    selected_Idx = [];
    %valid_track = zeros(1,p.scenario.num_of_time_steps);

    % Time Step Loop
    for k = 1:dt:p.scenario.num_of_time
      Measurement_Index_Total = [];
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
     Measurement_Total = [MeasurementT1;MeasurementT2];
    
      %% Prediction & PDA
      if  ~isempty(Tracks)
          for numTrack = 1:length(Tracks)
              [x_pred_EKF, P_pred_EKF] = PredictEKF(Tracks(numTrack).x, Tracks(numTrack).P, F, p.other.Q);
              Tracks(numTrack).x = x_pred_EKF;
              Tracks(numTrack).P = P_pred_EKF;
              [x_update, P_update,selected_Idx] = PDAEKF(Tracks(numTrack), Measurement_Total, Sensor_Parameter, TrackerGate, p.tracker(1).Pg);
              Tracks(numTrack).x = x_update;
              Tracks(numTrack).P = P_update;
          end
      end
            
      %% Track Initialization
      for num = 1:length(Measurement_Total)
          Measurement_Index_Total = [Measurement_Index_Total;num];
      end
      unselected_Idx = setdiff(Measurement_Index_Total, selected_Idx);
      Vmax = 30;
      for i = 1:length(unselected_Idx)
           Z_Init = Measurement_Total(i, :);
           [MeasurementConvert,Rp11,Rp22,Rp12] = convertMeasurement(Z_Init, Sensor_Parameter);
           newTrack.ID = i;
           newTrack.x = [MeasurementConvert(1); 0; MeasurementConvert(2); 0];
           newTrack.P = [Rp11   0      Rp12 0;
                         0  (Vmax/2)^2  0   0;
                         Rp12    0     Rp11 0;
                         0       0      0 (Vmax/2)^2];
            newTrack.TentaCount = 0;
            newTrack.ConfirmCount = 0;
            newTrack.Status= "Tentative";
            newTrack.assodata_Tentative = [];
            newTrack.assodata_Confirm = [];
            Tracks = [Tracks, newTrack];
      end      


     %% Track Management
     Tracks = TrackManagement(Tracks, selected_Idx ,M_tent, N_conf, M_conf);     
    end
end
%% Plotting Section
figure;
hold on
grid on;
xlabel('X Position');
ylabel('Y Position');
title('Assignment 2');
plot(Sensor_Parameter.Xpos, Sensor_Parameter.Ypos, 'k*', 'MarkerSize', 25); % Plot Sensor Plot
for plotstep = 1:size(real_Target1_Movement,2)
    plot(real_Target1_Movement(1, plotstep), real_Target1_Movement(3, plotstep), 'b.', 'MarkerSize', 15);
    plot(real_Target2_Movement(1, plotstep), real_Target2_Movement(3, plotstep), 'r.', 'MarkerSize', 15);
end
for i = 1:length(Tracks)
    if strcmp(Tracks(i).Status, 'Confirmed')
        plot(Tracks(i).x(1), Tracks(i).x(3), 's', 'color', 'g', 'MarkerSize', 7);
    elseif strcmp(Tracks(i).Status, 'Tentative')
        plot(Tracks(i).x(1), Tracks(i).x(3), 's', 'color', 'r', 'MarkerSize', 8);
    end
end

%% Performance Evalution
