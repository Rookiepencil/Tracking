function p = parameters()
%% Scenario Case 


 % p.scenario.monte_runs = 2;
 % p.scenario.num_of_time_steps = 50;

p.scenario.monte_runs = 1;
%p.scenario.num_of_time_steps = 100;
p.scenario.num_of_time = 40;
%% Target Parameters


 p.target(1).start_time = 5.0;
 p.target(1).end_time = 30.0;
 p.target(1).start_state = [5000 0 3000 -30]';
 p.target(1).process_noise = 0.01;
 p.target(1).sampletime = 1;

 p.target(2).start_time = 15.0;
  p.target(1).end_time = 40.0;
 p.target(2).start_state = [1000 0 3000 -30]';
 p.target(2).process_noise = 0.01;
 p.target(2).sampletime = 1;




%% Sensor Parameters

 p.sensor(1).Pd = 0.9;
 p.sensor(1).FalseDensity = 1e-4;
 p.sensor(1).sampling_time = 0.02;
 p.sensor(1).Xpos = 3000;
 p.sensor(1).Ypos = 50;
 p.sensor(1).Velocity = 0;
 p.sensor(1).rangeSigma = 50; 
 p.sensor(1).azimuthSigma = 0.01;
 p.sensor(1).rangeUb = 10000;
 p.sensor(1).rangeLb = 0;
 p.sensor(1).AziUb = pi;
 p.sensor(1).AziLb = -pi;

%% Tracker Parameters

 %p.tracker.gate_size = chi2inv(0.99,2);%change this value
 p.tracker.gate_size = chi2inv(0.99,2); %change this value


 % p.InitialCov = [sigmax, 0, 0, 0;
 %                 0, sigmaxdot,0, 0;
 %                  0, 0, sigmay, 0;
 %                  0, 0, 0, sigmaydot];
 % 


%% Performance Evalution Parameters

 p.perf_eval.gate_size = 100;
 

end