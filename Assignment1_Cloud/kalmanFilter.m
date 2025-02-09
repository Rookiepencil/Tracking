function [xk_hat, Pk_hat] = kalmanFilter(x_Pred, P_pred, z, R) % add additional parameters, if necessary

    H = [1 0 0 0;
         0 0 1 0];

    % Innovation
    Inno = z' - H * x_Pred;
     
    % S matrix
    S = H * P_pred * H' + R;

    % **Kalman Gain
    K = (P_pred * H') / S;

    % **Update**
    xk_hat = x_Pred + K * Inno;

    Pk_hat = P_pred - (K * S * K');
end