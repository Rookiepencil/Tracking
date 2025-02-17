function [xk,Q] = moveTarget(Process_Noise, xk_1, T) 
    F = [1, T, 0, 0; %state Transition matrix
         0, 1, 0, 0; % T is the sample time
         0, 0, 1, T;
         0, 0, 0, 1
        ]; 

    G = [(T^2 / 2), 0; T, 0; 0, (T^2 / 2); 0, T]; % process noise matrix
    RandomNoise = (G * sqrt(Process_Noise) *randn(2,1)); %Random noise calculation

    xk = (F * xk_1)+ RandomNoise; % state update
    Q  = G * Process_Noise * G'; % process noise covariance calculation
end