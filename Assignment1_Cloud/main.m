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
%% Main Loop
for r=1:p.scenario.monte_runs
     figure; % open the figure and make sure it can always update on that graph
     hold on; 
     xlabel('X Position');
     ylabel('Y Position');
     title('Assignment 1 ECE 767');
     plot(Sensor_Parameter.Xpos, Sensor_Parameter.Ypos, 'k*', 'MarkerSize', 25); % Plot target Movement

     xhat_k; % you will need mvmrnd to generate X update initial state
     P; %P stay the same
    
    for k=1:p.scenario.num_of_time_steps % This is the simulation step not the Actual time!!!!

        %if k == 1 I should not do anything -- Prof Suggestion

        plot(real_Target_Movement(1,k), real_Target_Movement(3,k), 'b.', 'MarkerSize', 15); % Plot target Movement
        %plot(Ideal_Movement(1,k), Ideal_Movement(3,k), 'r.', 'MarkerSize', 15); % Plot true target Movement

        [Measurement]= generateMeasurements(Sensor_Parameter, real_Target_Movement(:,k));

        [xk, Q,Truevalue] = moveTarget(Target_Process_Noise, xk_1, dt, Ideal_Movement(:,k)); %just for testing

         xk_1 = xk;
        Ideal_Movement = [Ideal_Movement,Truevalue];
        real_Target_Movement = [real_Target_Movement,xk];
        
        if isempty(Measurement)
            continue;
        end

        [MeasurementConvert, R] = convertMeasurement(Measurement, Sensor_Parameter);

        for y = 1:size(MeasurementConvert, 1)
            plot(MeasurementConvert(y,1), MeasurementConvert(y,2), 'm.', 'MarkerSize', 15);
        end
         
        % plot(FalseAlarmConvert(1), FalseAlarmConvert(2), 'rx', 'MarkerSize', 10); 
        % 
        % .. = dataAssociation(....);
        % 
        % .. = kalmanFilter(....)
        %pause(1)    
    end
    
    hold off
    %legend('Sensor Position','real Movement','Ideal Movement','Measurement','Location','best');
    legend('Sensor Position','real Movement','Measurement','Location','best');
    grid on;
end
%% calculate the RMSE


%% plot RMSE results



