function [x_update, P_update] = PDAEKF(tracker, Measurements, R, Sensor_Parameter, TrackerGate)
    
    x_pred = tracker.State;
    P_pred = tracker.P;
    selected_Idx = [];
    selected_Error = [];
    % H = [1 0 0 0;
    %      0 0 1 0];
    %R_Update = [];
    
    range_Jo = sqrt((x_pred(1) - Sensor_Parameter.Xpos)^2 + (x_pred(3) - Sensor_Parameter.Ypos)^2);
    %azi_Jo = atan2(x_pred(3)- Sensor_Parameter.Ypos , x_pred(1)- Sensor_Parameter.Xpos);
    dr_dx = (x_pred(1) - Sensor_Parameter.Xpos) / range_Jo;
    dr_dy = (x_pred(3) - Sensor_Parameter.Ypos) / range_Jo;
    darc_dx = -(x_pred(3)- Sensor_Parameter.Ypos) / range_Jo^2;
    darc_dy = (x_pred(1) - Sensor_Parameter.Xpos) / range_Jo^2;

    H = [dr_dx 0 dr_dy 0;
         darc_dx 0 darc_dy 0];

    for i = 1:size(Measurements, 1)
        S = (H * P_pred * H') + R(i:i+1, :);
        S_Inv = inv(S);

        z = Measurements(i, :)';  
        Error = z - (H * x_pred); 

        Distance = Error' * S_Inv * Error;
        

        if abs(Distance) < TrackerGate^2 %Initial Gating
            selected_Idx = [selected_Idx; i];
            selected_Error = [selected_Error;Error];
        end
    end
    
    if isempty(selected_Idx) % if no data in the gate direct out
        x_update = x_pred;
        P_update = P_pred;
        return;
    else % do PDA
        Measurements = Measurements(selected_Idx,:);
        %LLR Calculation
        
    end








end

