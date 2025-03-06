function [x_pred, P_pred] = PredictEKF(x, P, F, Q)
    x_pred = F * x;
    P_pred = F * P * F' + Q;
end

