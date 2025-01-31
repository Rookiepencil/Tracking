function [xk,Q,trueValue] = moveTarget(Process_Noise, xk_1, T, Ideal) % add additional parameters, if necessary
    F = [1, T, 0, 0;
         0, 1, 0, 0;
         0, 0, 1, T;
         0, 0, 0, 1
        ];

    trueValue = F * Ideal;

    G = [(T^2 / 2), 0; 0, T; (T^2 / 2), 0; 0, T];
    RandomNoise = (G * sqrt(Process_Noise) *randn(2,1));
    disp(RandomNoise);
    xk = (F * xk_1)+ RandomNoise;
    Q  = G * Process_Noise * G';
end