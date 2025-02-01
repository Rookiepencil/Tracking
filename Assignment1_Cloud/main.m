%%% Initialization
clear all
close all
clc
p = parameters();
Sensor_Parameter = p.sensor;
dt = p.target(1).sampletime; % Sampling time
xk_1 = p.target(1).start_state; % Initial state of the target
Target_Process_Noise = p.target(1).process_noise;
Ideal_Movement = []; %just for testing
real_Target_Movement =[]; 
Ideal_Movement = [Ideal_Movement,xk_1]; %just for testing
real_Target_Movement =[real_Target_Movement, xk_1];
% track = [0;0;0;0];
% plot(track(1,:), track(3,:), 's','color', 'c', 'MarkerSize', 8); % plot initial track state
Pk0= eye(4); % P stay the same
F = [1 dt 0 0;
     0 1  0 0;
     0 0  1 dt;
     0 0  0 1];
%% Main Loop
for r=1:p.scenario.monte_runs
     figure; % open the figure and make sure it can always update on that graph
     hold on; 
     xlabel('X Position');
     ylabel('Y Position');
     title('Assignment 1 ECE 767');
     plot(Sensor_Parameter.Xpos, Sensor_Parameter.Ypos, 'k*', 'MarkerSize', 25); % Plot target Movement

     % xhat_k; % you will need mvmrnd to generate X update initial state
     x_est = xk_1 + mvnrnd(zeros(4,1), eye(4))';
    
    for k=1:p.scenario.num_of_time_steps % This is the simulation step not the Actual time!!!!

        plot(real_Target_Movement(1,k), real_Target_Movement(3,k), 'b.', 'MarkerSize', 15); % Plot target Movement

        plot(x_est(1), x_est(3), 's','color', 'c', 'MarkerSize', 7); %Tracker
      
        Measurement= generateMeasurements(Sensor_Parameter, real_Target_Movement(:,k));

        [xk, Q] = moveTarget(Target_Process_Noise, xk_1, dt);

        xk_1 = xk;
        real_Target_Movement = [real_Target_Movement,xk];
        
        if isempty(Measurement)
            continue;
        end

        [MeasurementConvert, R] = convertMeasurement(Measurement, Sensor_Parameter);

        %Kalman Filter Predict Part, We really need it for data Association
        x_pred = F * x_est;
        P_pred = (F * Pk0 * F')+ Q;

        asso_meas_ind = dataAssociation(p.tracker.gate_size, x_pred, P_pred, MeasurementConvert, R);

        % for y = 1:size(MeasurementConvert, 1)
        %     plot(MeasurementConvert(y,1), MeasurementConvert(y,2), 'm.', 'MarkerSize', 15);
        % end

        if(asso_meas_ind ~= -1)
            z = MeasurementConvert(asso_meas_ind,:);
            plot(MeasurementConvert(asso_meas_ind,1), MeasurementConvert(asso_meas_ind,2), 'm.', 'MarkerSize', 15);
            [xk_hat, Pk_hat] = kalmanFilter(x_pred, P_pred, z, R);
            x_est = xk_hat;
            Pk0 = Pk_hat;
        else
            x_est = x_pred;
            Pk0 = P_pred;
        end
        
         %pause(1)    
    end
    
    hold off
    %legend('Sensor Position','real Movement','Ideal Movement','Measurement','Location','best');
    legend({'Sensor Position','Real','Tracker'}, 'Location', 'best');
    grid on;
end
%% calculate the RMSE


%% plot RMSE results



