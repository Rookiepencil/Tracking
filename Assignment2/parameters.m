function p = parameters()
%% Scenario Case 


 % p.scenario.monte_runs = 2;
 % p.scenario.num_of_time_steps = 50;

p.scenario.monte_runs = 1;
%p.scenario.num_of_time_steps = 100;
p.scenario.num_of_time = 45;

p.IDcounter = 1;
%% Target Parameters


 p.target(1).start_time = 5.0;
 p.target(1).end_time = 30.0;
 p.target(1).Xspeed = 10;
 p.target(1).Yspeed = 30;
 p.target(1).start_state = [1000 p.target(1).Xspeed 500 p.target(1).Yspeed]';
 p.target(1).process_noise = 0.01;
 p.target(1).sampletime = 1;
 

 p.target(2).start_time = 55.0;
 p.target(2).end_time = 40.0;
 p.target(2).Xspeed = 10;
 p.target(2).Yspeed = 30;
 p.target(2).start_state = [5000 p.target(2).Xspeed  3000 p.target(2).Yspeed]';
 p.target(2).process_noise = 0.01;
 p.target(2).sampletime = 1;




%% Sensor Parameters

 p.sensor(1).Pd = 1;
 p.sensor(1).FalseDensity = 1e-5;
 p.sensor(1).sampling_time = 1;
 p.sensor(1).Xpos = 3000;
 p.sensor(1).Ypos = 500;
 p.sensor(1).Velocity = 0;
 p.sensor(1).rangeSigma = 10; 
 p.sensor(1).azimuthSigma = 0.01;
 p.sensor(1).rangeUb = 10000;
 p.sensor(1).rangeLb = 0;
 p.sensor(1).AziUb = pi;
 p.sensor(1).AziLb = -pi;

%% Tracker Parameters
 
 p.tracker(1).Pg = 0.99;
 p.tracker(2).Pg = 0.99;
 p.tracker(1).gate_size = chi2inv(p.tracker(1).Pg,2); %change this value
 p.tracker(2).gate_size = chi2inv(p.tracker(1).Pg,2);
 % p.tracker.Status = "Tentative";
 % p.tracker.Score = 0;
 % p.tracker.P = eye(4)*300;
 p.tracker(1).Ntent = 5;
 p.tracker(1).Mtent = 3;


 p.tracker(1).Nconf = 5;
 p.tracker(1).Mconf = 2;


%% Performance Evalution Parameters

 p.perf_eval.gate_size = 100;
 %% Q matrix
    G = [(p.target(2).sampletime / 2), 0; p.target(2).sampletime, 0; 0, (p.target(2).sampletime^2 / 2); 0, p.target(2).sampletime]; % process noise matrix

     p.other.Q = G * p.target(1).process_noise * G'; % process noise co
 

end