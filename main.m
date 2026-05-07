%%% This is a sample template. You can change as you like
clear all
close all

p = parameters();

%%% perform any initialization


% error_monte.position=[];
% error_monte.speed=[];

latency_monte=[];
false_tracks_monte=[];
term_latency_monte=[];

for r=1:p.scenario.monte_runs
    %%% perform any initialization this run
    
    % x_k=[];
    for i=1:p.number_of_targets
        x_k.target(i).status=0;
    end
    truth=[];
    % x_k = moveTarget(p, 0 , x_k);
    % truth = [truth, x_k];
  
  
    
    xk_1_hat=[];
    pk_1_hat=[];
    estimate.xk=[];
   
    measurements.range=[];
    measurements.azimuth =[];
    unasso_meas.range=[];
    confirmed_tracks=[];

    tracks=[];
    hist.truthx=[];
    hist.truthy=[];
    hist.truthxdot=[];
    hist.truthydot=[];
    hist.measx=[];
    hist.measy=[];
    hist.measxdot=[];
    hist.measydot=[];
    hist.is_associated=[];

    % error.position=[];
    % error.position =[];
    % error.speed=[];
    position=[];
    speed=[];
    false_tracks_matrix=[];
    latency_matrix=[];
    term_latency=[];
    latency=[];

    for i=1:numel(p.target)
        latency_checked.target(i)=0;
    end

    % last_sample_time=[];
    % for i=1:numel(p.sensor)
    %     last_sample_time(i)=p.sensor(i).sampling_time;
    % end
    last_sample_time = zeros(size(p.sensor));
    time=0;
    k=0;
    totaltime=[];
    % for k=1:p.scenario.num_of_time_steps
    while time<p.scenario.num_of_time_steps     
         k=k+1;
         [time,dt,sensor_id,last_sample_time] = nextSampleTime(p,last_sample_time,time);
         totaltime=[totaltime time];


        % x_k = moveTarget(p, k*p.sensor(1).sampling_time, x_k);
        x_k = moveTarget(p, time, x_k,dt);
        truth = [truth, x_k];
        

        [polar,false_alarms] = generateMeasurements(p, x_k,sensor_id);
        current_measurements.range=[];
        current_measurements.azimuth=[];
        for i=1:p.number_of_targets
            measurements.range=[measurements.range, polar(i).range];
            measurements.azimuth=[measurements.azimuth, polar(i).azimuth];
            current_measurements.range=[current_measurements.range, polar(i).range ];
            current_measurements.azimuth=[current_measurements.azimuth, polar(i).azimuth];
        end
        
        for i=1:numel(false_alarms)
            measurements.range=[measurements.range, false_alarms(i).range];
            measurements.azimuth=[measurements.azimuth, false_alarms(i).azimuth];
            current_measurements.range=[current_measurements.range, false_alarms(i).range ];
            current_measurements.azimuth=[current_measurements.azimuth, false_alarms(i).azimuth];
        end
                  
        unasso_meas = current_measurements;
        for i=1:numel(tracks)
            [xk_1_hat, pk_1_hat, unasso_meas,associated] = kalmanFilter(p, i, tracks(i).xk_1_hat{end}, tracks(i).pk_1_hat{end}, unasso_meas , p.sensor(sensor_id).R,dt,sensor_id);
            %uncomment next line and comment previous line to run Nearest Neighbor
            % [xk_1_hat, pk_1_hat,unasso_meas,associated] = kalmanFilterNN(p, i, tracks(i).xk_1_hat{end}, tracks(i).pk_1_hat{end}, unasso_meas , p.sensor(1).R);
            tracks(i).xk_1_hat{end+1}=xk_1_hat;
            tracks(i).pk_1_hat{end+1}=pk_1_hat;
            tracks(i).associated(end+1)=associated;
            estimate.xk = [estimate.xk xk_1_hat];
        end
        
        

        h=numel(tracks);
        for m = 1:numel(unasso_meas.range)
             meas.range=unasso_meas.range(m);
             meas.azimuth = unasso_meas.azimuth(m);
             [xk_1_hat, pk_1_hat] = trackInit(p, meas,sensor_id);
             tracks(h+m).xk_1_hat = {xk_1_hat};
             tracks(h+m).pk_1_hat = {pk_1_hat};
             tracks(h+m).status = 1; %tentative
             tracks(h+m).associated=1;
        end
       
        [tracks,confirmed_tracks,term_latency] = trackManager(p,tracks, confirmed_tracks,time,term_latency,sensor_id);

        false_tracks=0;
        for ft=1:numel(tracks)
            if tracks(ft).status==2
                false_tracks=false_tracks + 1;
            end
        end
        for i=1:numel(truth(end).target)
            error(k).target(i) = struct('position', 0, 'speed', 0);
            if truth(end).target(i).status==1
                [x,y,xdot,ydot,closest_x,closest_y,closest_xdot,closest_ydot,is_associated] = associateTracks(p,truth,tracks,i);                
                hist.truthx = [hist.truthx x];
                hist.truthy = [hist.truthy y];
                hist.truthxdot = [hist.truthxdot xdot];
                hist.truthydot = [hist.truthydot ydot];
                hist.measx = [hist.measx closest_x];
                hist.measy = [hist.measy closest_y];
                hist.measxdot = [hist.measxdot closest_xdot];
                hist.measydot = [hist.measydot closest_ydot];
                hist.is_associated = [hist.is_associated is_associated];
                
                if is_associated
                    error(k).target(i).position =  ((x-closest_x)^2 + (y-closest_y)^2);
                    error(k).target(i).speed = ((xdot-closest_xdot)^2 + (ydot-closest_ydot)^2);
                    
                    false_tracks = false_tracks-1;
                    if latency_checked.target(i) == 0
                        latency.target(i)=time-p.target(i).start_time;
                        % latency.target(i)=k-p.target(i).start_time;
                        latency_checked.target(i)=1;
                    end
                else
                    error(k).target(i).position = 0;
                    error(k).target(i).speed = 0;
                end
            end
        end
       false_tracks_matrix=[false_tracks_matrix false_tracks];
       
    end
    new_number_of_time_steps=k; 

    for k=1:new_number_of_time_steps
        for i=1:numel(truth(end).target)
            error_monte.run(r).target(i).position(k) = error(k).target(i).position;
            error_monte.run(r).target(i).speed(k) = error(k).target(i).speed;
        end
    end
    
    latency_monte= [latency_monte latency];
    false_tracks_monte=[false_tracks_monte ; false_tracks_matrix];
    if isempty(term_latency)
        term_latency.target=p.tracker.n_conf;
    end
    term_latency_monte=[term_latency_monte term_latency];
    
    % term_latency_monte=[term_latency_monte ; latency_matrix];
    %calculate values for RMSE
    %add column together then divide by 100 then take sqrrt
    
end



%calculate and plot RMSE and Valid Runs -------------------------------------------
valid_matrix = [];
for i=1:numel(truth(end).target)
    for k=1:new_number_of_time_steps
        sum=0;
        sum_speed=0;
        valid_runs=0;
        for r=1:p.scenario.monte_runs
            if error_monte.run(r).target(i).position(k) ~= 0
                sum = sum + error_monte.run(r).target(i).position(k);
                sum_speed=sum_speed + error_monte.run(r).target(i).speed(k);
                valid_runs=valid_runs+1;
            end
        end
        sum_error.frame(k).target(i) = sqrt(sum/valid_runs);
        sum_error_speed.frame(k).target(i) = sqrt(sum_speed/valid_runs);
        valid_matrix(k).target(i)=valid_runs;
    end
end

figure;
subplot(2,2,1); 
hold on;
for i = 1:numel(truth(end).target)
    x_values = []; 
    y_values = []; 
    for k = 1:new_number_of_time_steps
        y = sum_error.frame(k).target(i); 
        x = k;
        x_values = [x_values x];
        y_values = [y_values y];
    end
    plot(x_values, y_values, '-', 'DisplayName', ['Target ' num2str(i)]);
end
hold off;
xlabel('Frame ');
ylabel('RMSE Position');
title('RMSE Position');
legend('show');


subplot(2,2,2);
hold on;
for i = 1:numel(truth(end).target)
    x_values = []; 
    y_values = []; 
    for k = 1:new_number_of_time_steps
        y = sum_error_speed.frame(k).target(i); 
        x = k;
        x_values = [x_values x];
        y_values = [y_values y];
    end
    plot(x_values, y_values, '-', 'DisplayName', ['Target ' num2str(i)]);
end
hold off;
xlabel('Frame ');
ylabel('RMSE Speed');
title('RMSE Speed');
legend('show');


subplot(2,2,[3 4]);
hold on
for i = 1:numel(truth(end).target)
    x_values = []; 
    y_values = []; 
    for k = 1:new_number_of_time_steps
        x = k;
        y = valid_matrix(k).target(i);
        x_values = [x_values x];
        y_values = [y_values y];
    end
    plot(x_values, y_values, '-', 'DisplayName', ['Target ' num2str(i)]);
end
hold off;
xlabel('Frame ');
ylabel('Valid Runs');
legend('show');

%plot latency and false tracks--------------------------------------------
figure;
subplot(2,2,1)
hold on;
for i = 1:numel(truth(end).target)
    x_values = []; 
    y_values = []; 
    for k = 1:p.scenario.monte_runs
    % Check if latency_monte(k).target(i) exists
        if k<=numel(latency_monte) && isfield(latency_monte(k), 'target') && numel(latency_monte(k).target) >= i
            x = k; 
            y = latency_monte(k).target(i); 
            x_values = [x_values x];
            y_values = [y_values y];
        end
    end
    plot(x_values, y_values, '-', 'DisplayName', ['Target ' num2str(i)]);
end
hold off;
xlabel('Monte Carlo Run');
ylabel('Latency');
title('Start Latency for Each Target');
legend('show');


subplot(2,2,2)
hold on;
for i = 1:numel(truth(end).target)
    x_values = []; 
    y_values = []; 
    for k = 1:p.scenario.monte_runs
        if isfield(term_latency_monte(k), 'target') && numel(term_latency_monte(k).target) >= i
            x = k; 
            y = term_latency_monte(k).target(i); 
            x_values = [x_values x];
            y_values = [y_values y];
        end
    end
    plot(x_values, y_values, '-', 'DisplayName', ['Target ' num2str(i)]);
end
hold off;
xlabel('Monte Carlo Run');
ylabel('Latency');
title('Termination Latency for Each Target');
legend('show');

subplot(2,2,[3 4]);
hold on
averages = []; 
for i = 1:size(false_tracks_monte, 2)
    col_avg = mean(false_tracks_monte(:, i));
    averages = [averages col_avg];
end
plot(1:size(false_tracks_monte, 2), averages, '-');
hold off
xlabel('Frame');
ylabel('Average per monte run');
title('False Tracks');




%plot Truth ---------------------------------------------------------------
figure;
subplot(2,2,1)
hold on;
for i = 1:numel(truth)
    for j = 1:p.number_of_targets
        if truth(i).target(j).status == 1
            plot(truth(i).target(j).state(1), truth(i).target(j).state(2),'bx');
        end
    end
end
for k = 1:numel(p.sensor)
    x_center = p.sensor(k).start_state(1);
    y_center = p.sensor(k).start_state(2);
    radius = p.sensor(k).range(2);
    theta = linspace(0, 2*pi, 100);
    x_circle = x_center + radius * cos(theta);
    y_circle = y_center + radius * sin(theta);
    plot(x_circle, y_circle, 'r'); % 'r' for red color
end

plot(p.sensor(1).start_state(1), p.sensor(1).start_state(2), 'o', p.sensor(2).start_state(1), p.sensor(2).start_state(2), 'o')
% plot(p.sensor(1).start_state(1), p.sensor(1).start_state(2), 'o')
hold off;
xlabel('x');
ylabel('y');
title('Truth');

%Plot all measurements-----------------------------------------------------
subplot(2,2,3)
x_values = estimate.xk(1, :); 
y_values = estimate.xk(2, :); 
plot(x_values, y_values, '.'); 

%Plot confirmed tracks and truths------------------------------------------
x_values = [];
y_values = [];
track_numbers=[];
for i = 1:numel(confirmed_tracks)
    for j = 1:numel(confirmed_tracks(i).xk_1_hat)
        x_values = [x_values confirmed_tracks(i).xk_1_hat{j}(1)];
        y_values = [y_values confirmed_tracks(i).xk_1_hat{j}(2)];
        track_numbers = [track_numbers i];
    end
end
unique_tracks = unique(track_numbers);
subplot(2,2,[2 4])
hold on;
for track_num = unique_tracks
    track_indices = find(track_numbers == track_num);
    x_track = x_values(track_indices);
    y_track = y_values(track_indices);
    plot(x_track, y_track, 'b.-', 'DisplayName', ['Track ' num2str(track_num)]);
end
x_values = [];
y_values = [];
truth_numbers=[];
for i = 1:numel(truth)
    for j = 1:p.number_of_targets
        if truth(i).target(j).status == 1
            x_values = [x_values truth(i).target(j).state(1)];
            y_values = [y_values truth(i).target(j).state(2)];
            truth_numbers = [truth_numbers j];
        end
    end 
end
truth_hist=[];
for truth_num = unique(truth_numbers)
    truth_indices = find(truth_numbers == truth_num);
    x_track = x_values(truth_indices);
    y_track = y_values(truth_indices);
    plot(x_track, y_track, 'r.-', 'DisplayName', ['Truth ' num2str(truth_num)]);
    truth_hist(truth_num).x=x_values;
    truth_hist(truth_num).y=y_values;
end
hold off
xlabel('X');
ylabel('Y');
title('Tracks');
legend('show');





