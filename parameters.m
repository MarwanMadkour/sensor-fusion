function p = parameters()
%%% You can change the names as you like

 p.scenario.monte_runs = 100;
 p.scenario.num_of_time_steps = 60;
 % add anything related to the scenario
 p.number_of_targets= 2; %eval num of targets instead 
 p.target(1).start_time = 2;
 p.target(1).end_time = 25;
 p.target(1).start_state = [3000; 3000; 30; -20]; %away   %[x y xdot ydot] 
 p.target(1).weight_x=0.01;
 p.target(1).weight_y=0.01;
 p.target(1).max_acc_x=1;
 p.target(1).max_acc_y=1;
 p.target(1).process_noise_x =  p.target(1).weight_x* p.target(1).max_acc_x;
 p.target(1).process_noise_y =  p.target(1).weight_x* p.target(1).max_acc_y;
 p.target(1).mu=0;
 p.target(1).sigma=2;
 p.target(1).survival_probability = 1;
 p.target(1).vmax= 50;

 %need to add the same parameters for every target
 p.target(2).start_time = 10;
 p.target(2).end_time = 30;
 p.target(2).start_state = [1100; 1800; 30; -20]; %away   %[x y xdot ydot]

 % add anything related to the target

 p.sensor(1).sampling_time = 3;
 p.sensor(1).start_state=[6000; 1000];
 p.sensor(1).probability_of_detection=0.9;
 p.sensor(1).false_alarm_density=1e-4;%1e-4
 p.sensor(1).range=[0 5000];
 p.sensor(1).azimuth=[-pi pi];
 p.sensor(1).lambda= p.sensor(1).false_alarm_density * (p.sensor(1).azimuth(1,2) - p.sensor(1).azimuth(1,1)) * (p.sensor(1).range(1,2) - p.sensor(1).range(1,1));
 p.sensor(1).azimuth_std=0.01;
 p.sensor(1).range_std=10;
 p.sensor(1).mu=[0 0];
 p.sensor(1).R=diag([p.sensor(1).range_std^2 p.sensor(1).azimuth_std^2]);


 p.sensor(2).sampling_time = 2.5;
 p.sensor(2).start_state=[2000; 1000];
 p.sensor(2).probability_of_detection=0.9;
 p.sensor(2).false_alarm_density=1e-4;%1e-4
 p.sensor(2).range=[0 5000];
 p.sensor(2).azimuth=[-pi pi];
 p.sensor(2).lambda= p.sensor(1).false_alarm_density * (p.sensor(1).azimuth(1,2) - p.sensor(1).azimuth(1,1)) * (p.sensor(1).range(1,2) - p.sensor(1).range(1,1));
 p.sensor(2).azimuth_std=0.01;
 p.sensor(2).range_std=10;
 p.sensor(2).mu=[0 0];
 p.sensor(2).R=diag([p.sensor(1).range_std^2 p.sensor(1).azimuth_std^2]);
 % add anything related to the the sensor

 % p.tracker.gate_size = 2.1;%change this value
 p.tracker.mu=p.target(1).start_state;
 p.tracker.Pk= diag([100^2 100^2 10^2 10^2]);
 % p.tracker.threshold_dlt_tentative = 0.1;
 % p.tracker.threshold_prmt_active = 0.5;
 p.tracker.p_g=0.99;
 p.tracker.asso_range=25;
 p.tracker.n_tent=3;%3
 p.tracker.m_tent=3;%3
 p.tracker.n_conf=5;%5
 p.tracker.m_conf=1;%1
 % add anything related to the tracker

  p.perf_eval.gate_size = 100^2;
 % add anything related to performance evaluation

end