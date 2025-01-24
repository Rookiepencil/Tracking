function [xk,Q] = moveTarget(parameter, xk_1, T) % add additional parameters, if necessary
    F = [1, T, 0, 0;
         0, 1, 0, 0;
         0, 0, 1, T;
         0, 0, 0, 1
        ];

    G = [(T^2 / 2), 0; 0, T; (T^2 / 2), 0; 0, T];

    xk = F * xk_1 + G * parameter *randn(2,1);
    Q  = G * parameter * G';
end