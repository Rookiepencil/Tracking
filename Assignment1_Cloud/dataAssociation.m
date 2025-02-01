function asso_meas_ind = dataAssociation(TrackGate, x_pred, P_pred, meas, R)

    minimumDis = inf;
    asso_meas_ind = -1;
    H = [1 0 0 0;
         0 0 1 0];

    S = (H * P_pred * H') + R;
    S_Inv = inv(S);

    for i = 1:size(meas, 1)
        z = meas(i, :)';  
        Error = z - (H * x_pred); 

        Distance = Error' * S_Inv * Error;

        if Distance < TrackGate && Distance < minimumDis
            asso_meas_ind = i;      
            minimumDis = Distance;
        end
    end
end