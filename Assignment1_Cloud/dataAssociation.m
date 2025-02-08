function [asso_meas_ind,R_Update] = dataAssociation(TrackGate, x_pred, P_pred, meas, R)

    minimumDis = inf;
    asso_meas_ind = -1;
    H = [1 0 0 0;
         0 0 1 0];
    R_Update = [];

    
    for i = 1:size(meas, 1)
        S = (H * P_pred * H') + R(i:i+1, :);
        S_Inv = inv(S);

        z = meas(i, :)';  
        Error = z - (H * x_pred); 

        Distance = Error' * S_Inv * Error;

        if abs(Distance) < TrackGate && abs(Distance) < minimumDis
            asso_meas_ind = i;      
            minimumDis = Distance;
            R_Update =  R(i:i+1, :);
        end
    end
end