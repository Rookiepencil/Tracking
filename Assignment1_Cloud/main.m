%%% Initialization
clear all
close all

p = parameters();
Sensor_Parameter = p.sensor;
figure; % open the figure and make sure it can always update on that graph
hold on; 
xlabel('X Position');
ylabel('Y Position');
title('Assignment 1');
grid on;

%% Main Loop
for r=1:p.scenario.monte_runs
    %%% perform any initialization this run

    xk_1 = p.target(1).start_state; % Initial state of the target
    plot(xk_1(1), xk_1(3), 'b.', 'MarkerSize', 15); % Plot target Movement
    plot(Sensor_Parameter.Xpos, Sensor_Parameter.Ypos, 'g.', 'MarkerSize', 25); % Plot target Movement
    
    [Measurement, FalseAlarm]= generateMeasurements(Sensor_Parameter, xk_1);
    if ~isempty(Measurement)
        [MeasurementConvert, FalseAlarmConvert, R] = convertMeasurement(Measurement, FalseAlarm, Sensor_Parameter);
        plot(MeasurementConvert(1), MeasurementConvert(2), 'm.', 'MarkerSize', 15); 
    end
    
    dt = p.target(1).sampletime; % Sampling time
    Parameter = p.target(1).process_noise;

    for k=1:dt:p.scenario.num_of_time_steps 
        
        [xk, Q] = moveTarget(Parameter, xk_1, k);
        plot(xk(1), xk(3), 'b.', 'MarkerSize', 15); % Plot target Movement
        [Measurement, FalseAlarm]= generateMeasurements(Sensor_Parameter, xk);
        if isempty(Measurement)
            continue;
        end
        
        [MeasurementConvert, FalseAlarmConvert, R] = convertMeasurement(Measurement, FalseAlarm, Sensor_Parameter);
        plot(MeasurementConvert(1), MeasurementConvert(2), 'm.', 'MarkerSize', 15); 
        plot(FalseAlarmConvert(1), FalseAlarmConvert(2), 'rx', 'MarkerSize', 10); 
        % 
        % .. = dataAssociation(....);
        % 
        % .. = kalmanFilter(....)
        
    end
    
    hold off
    
end

%% calculate the RMSE


%% plot RMSE results



