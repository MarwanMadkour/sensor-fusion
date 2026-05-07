function [time,dt,sensor_id,last_sample_time] = nextSampleTime(p,last_sample_time,prev_time)

time=inf;
for i=1:numel(p.sensor)
    if last_sample_time(i) + p.sensor(i).sampling_time <time
        time = last_sample_time(i)+p.sensor(i).sampling_time;
        sensor_id=i;
    end
end
dt = time - prev_time;
last_sample_time(sensor_id)=time;




















end