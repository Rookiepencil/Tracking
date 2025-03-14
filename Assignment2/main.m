%%% Initialization
clear all;
close all;

%rng(1);

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
    figure;
    hold on;
    grid on;
    xlabel('X Position');
    ylabel('Y Position');
    
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
    deletedTracks = [];
    selected_Idx = [];
    p.IDcounter = 1;
    %valid_track = zeros(1,p.scenario.num_of_time_steps);

    % Time Step Loop
    for k = 1:dt:p.scenario.num_of_time
      Measurement_Index_Total = [];
      title(sprintf('EKF + PDA, Time step: %d', k));
    %% Target Generation
        %time = (k-1)*dt
        if k >= p.target(1).start_time && k <= p.target(1).end_time
          if k > p.target(1).start_time
            [Target_1_Next_State, Target1_Q] = moveTarget(Target1_Process_Noise, Target_1_Start_State, dt);
            real_Target1_Movement = [real_Target1_Movement,Target_1_Next_State];
            Target_1_Start_State = Target_1_Next_State;
          else
            real_Target1_Movement = [real_Target1_Movement,Target_1_Start_State];
          end
       else
           real_Target1_Movement = [real_Target1_Movement,nan(4,1)]; 
       end

       if k >= p.target(2).start_time && k <= p.target(2).end_time
           if k > p.target(2).start_time
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
        Measurement_Total = [MeasurementT1; MeasurementT2];
        
        %% Prediction & PDA Update 
        usedIdx_total = [];
        if ~isempty(Tracks)
            for numTrack = 1:length(Tracks)
                if Tracks(numTrack).Status == "Tentative" || Tracks(numTrack).Status == "Confirmed"
                    % EKF Prediction
                    [x_pred, P_pred] = PredictEKF(Tracks(numTrack).x, Tracks(numTrack).P, F, p.other.Q);
                    Tracks(numTrack).x = x_pred;
                    Tracks(numTrack).P = P_pred;
                    
                    % PDA Update
                    [x_upd, P_upd, selected_Idx] = PDAEKF(Tracks(numTrack), Measurement_Total, Sensor_Parameter, TrackerGate, p.tracker(1).Pg);
                    Tracks(numTrack).x = x_upd;
                    Tracks(numTrack).P = P_upd;
                    usedIdx_total = union(usedIdx_total, selected_Idx);
                    Tracks(numTrack).TrackHistory_x = [Tracks(numTrack).TrackHistory_x; x_upd'];
                    Tracks(numTrack).TrackHistory_p = [Tracks(numTrack).TrackHistory_p;P_upd];
                    
                    % Tell us whether assocation is success or not
                    if ~isempty(selected_Idx)
                        Tracks(numTrack).assocResultCurrent = 1;
                    else
                        Tracks(numTrack).assocResultCurrent = 0;
                    end
                end
            end
        end
        
        %% Track Initialization for Unassociated
      for num = 1:size(Measurement_Total, 1)
          Measurement_Index_Total = [Measurement_Index_Total;num];
      end
      unselected_Idx = setdiff(Measurement_Index_Total, usedIdx_total);
      Vmax = 30;

      for i = 1:length(unselected_Idx)
           actual_unasso_Measu = unselected_Idx(i);
           Z_Init = Measurement_Total(actual_unasso_Measu, :);
           [MeasurementConvert,Rp11,Rp22,Rp12] = convertMeasurement(Z_Init, Sensor_Parameter);
           newTrack.ID = p.IDcounter;
           newTrack.x = [MeasurementConvert(1); 0; MeasurementConvert(2); 0];
           newTrack.P = [Rp11   0      Rp12 0;
                         0  (Vmax/2)^2  0   0;
                         Rp12    0     Rp22 0;
                         0       0      0 (Vmax/2)^2];
            %newTrack.P = nearSPD(newTrack.P);
            newTrack.TentaCount = 0;
            newTrack.ConfirmCount = 0;
            newTrack.Status= "Tentative";
            newTrack.assodata_Tentative = [];
            %newTrack.assodata_Confirm = [];
            newTrack.TrackHistory_x = []; 
            newTrack.TrackHistory_p = [];
            newTrack.assocResultCurrent = 1;
            newTrack.assodata_Tentative = [];
            Tracks = [Tracks, newTrack];
            p.IDcounter = p.IDcounter + 1;
      end      
        
        %% Track Management
  
        [Tracks, deletedTracks] = TrackManagement(Tracks, deletedTracks, N_tent, M_tent, N_conf, M_conf);

        
        %% Plotting Section

        % Plot Sensor Position
        plot(Sensor_Parameter.Xpos, Sensor_Parameter.Ypos, 'k*', 'MarkerSize', 27);

        % Target Movement
        if ~isempty(real_Target1_Movement)
            plot(real_Target1_Movement(1,1:k), real_Target1_Movement(3,1:k), 'b.', 'MarkerSize', 15);
        end
        if ~isempty(real_Target2_Movement)
            plot(real_Target2_Movement(1,1:k), real_Target2_Movement(3,1:k), 'b.', 'MarkerSize', 15);
        end

        % Current Measurement
        if ~isempty(Measurement_Total)
            meas_cart = [];
            for m = 1:size(Measurement_Total,1)
                meas_cart = [meas_cart; convertMeasurement(Measurement_Total(m,:),Sensor_Parameter)];
                if ~isempty(meas_cart)
                    plot(meas_cart(m,1), meas_cart(m,2), 'm.', 'MarkerSize', 15);
                end
            end
            
        end
        % Plot Trackers
        for t = 1:length(Tracks)
            if Tracks(t).Status == "Confirmed"
                plot(Tracks(t).x(1), Tracks(t).x(3), 's', 'color', 'g', 'MarkerSize', 7);
            elseif Tracks(t).Status == "Tentative"
                plot(Tracks(t).x(1), Tracks(t).x(3), 's', 'color', 'r', 'MarkerSize', 8);
            elseif Tracks(t).Status == "Dead"
                plot(Tracks(t).x(1), Tracks(t).x(3), 's', 'color', 'k', 'MarkerSize', 8);
            end
        end
       
        pause(1)
      
        
        
        
    end  % End of time step loop
    hold off 
end
 
%% Performance Evalution
