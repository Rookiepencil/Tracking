function [x_update, P_update,selected_Idx] = PDAEKF(tracker, Measurements, Sensor_Parameter, TrackerGate, Pg)
    % need shuchu unassoicated idx
    x_pred = tracker.x;
    P_pred = tracker.P;
    selected_Idx = [];
    unselected_Idx = [];
    selected_Error = [];
    V = zeros(2,1);
    Term1 = zeros(2,2);
    LLR_Matrix = zeros(1, size(Measurements,1));
    % H = [1 0 0 0;
    %      0 0 1 0];
    R  = [Sensor_Parameter.rangeSigma 0;
          0 Sensor_Parameter.azimuthSigma];

    Lamda = Sensor_Parameter.FalseDensity * (Sensor_Parameter.rangeUb  - Sensor_Parameter.rangeLb) * (Sensor_Parameter.AziUb  - Sensor_Parameter.AziLb);
   
    
    range_Jo = sqrt((x_pred(1) - Sensor_Parameter.Xpos)^2 + (x_pred(3) - Sensor_Parameter.Ypos)^2);
    %azi_Jo = atan2(x_pred(3)- Sensor_Parameter.Ypos , x_pred(1)- Sensor_Parameter.Xpos);
    dr_dx = (x_pred(1) - Sensor_Parameter.Xpos) / range_Jo;
    dr_dy = (x_pred(3) - Sensor_Parameter.Ypos) / range_Jo;
    darc_dx = -(x_pred(3)- Sensor_Parameter.Ypos) / range_Jo^2;
    darc_dy = (x_pred(1) - Sensor_Parameter.Xpos) / range_Jo^2;

    H = [dr_dx 0 dr_dy 0;
         darc_dx 0 darc_dy 0];

    S = (H * P_pred * H') + R;
    S_Inv = inv(S);

    for i = 1:size(Measurements, 1)
       
        z = Measurements(i, :)';
        z_hat = (H * x_pred);
        Error = z - z_hat; 
        Error(2) = wrapToPi(Error(2)); %Important From  ZHILI HUANG Last presentation
   
        Distance = Error' * S_Inv * Error;
        
        if abs(Distance) < TrackerGate^2 %Initial Gating
            selected_Idx = [selected_Idx; i];
            selected_Error = [selected_Error,Error];
        else
            %unasso idx
            unselected_Idx = [unselected_Idx; i];
        end
    end
    
    if isempty(selected_Idx) % if no data in the gate direct out
        x_update = x_pred;
        P_update = P_pred;
        return;
    else % do PDA
        Measurements = Measurements(selected_Idx,:);
        %LLR Calculation
        for s = 1:length(Measurements)
            selected_Error = selected_Error(:,s);
            % ECE 712 Formula
            LLR = (1/(2*pi)) * (1 / sqrt(det(S))) * exp(-(1/2) * (selected_Error' * S_Inv * selected_Error));
            LLR = (LLR * Sensor_Parameter.Pd) / Lamda;
            LLR_Matrix(s) = LLR;
        end
        BETADenom = (1 - (Sensor_Parameter.Pd * Pg)) + sum(LLR_Matrix);
        BETA = LLR_Matrix / BETADenom;
        BETA0 = (1 - (Sensor_Parameter.Pd * Pg)) / BETADenom;

        %Kalman Gain
        K = (P_pred * H') / S;
        %Updating
          for m = 1:length(Measurements)
              V = V + (BETA(m) * selected_Error(:,m));
              Term1 = Term1 + (BETA(m) * selected_Error(:,m) * selected_Error(:,m)');
          end
        P_Tilt = K * (Term1 - (V * V'))* K';
        P_C = P_pred - (K * S * K');
        x_update = x_pred + K * V;
        P_update = (BETA0 * P_pred) + ((1-BETA0) * P_C) + P_Tilt;

    end

end

