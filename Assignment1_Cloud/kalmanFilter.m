function [xk_hat, Pk_hat] = kalmanFilter(xk_1_hat, Pk_1_hat, z, R) % add additional parameters, if necessary

    H = [1 0 0 0;
         0 0 1 0];

    % Innovation
    Inno = z' - H * xk_1_hat;
     
    % S matrix
    S = H * Pk_1_hat * H' + R;

    % **Kalman Gain
    K = (Pk_1_hat * H') / S;

    % **Update**
    xk_hat = xk_1_hat + K * Inno;

    Pk_hat = Pk_1_hat - (K * S * K');
end