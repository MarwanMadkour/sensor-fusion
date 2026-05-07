function [polar,false_alarms] = generateMeasurements(p, x_k,sensor_id) % add additional parameters, if necessary
x_s=p.sensor(sensor_id).start_state(1,1);
y_s=p.sensor(sensor_id).start_state(2,1);
false_alarms_per_frame=poissrnd(p.sensor(sensor_id).lambda);
false_alarms.range=[];
false_alarms.azimuth=[];
w=mvnrnd(p.sensor(sensor_id).mu,p.sensor(sensor_id).R);

for i=1:p.number_of_targets
    polar(i).range= [];
    polar(i).azimuth = [];
    if rand<=p.sensor(sensor_id).probability_of_detection && x_k.target(i).status == 1
        range = sqrt((x_k.target(i).state(1,1)-x_s)^2+(x_k.target(i).state(2,1)-y_s)^2);
        if range < (p.sensor(sensor_id).range(2) - p.sensor(sensor_id).range(1))
            polar(i).range=sqrt((x_k.target(i).state(1,1)-x_s)^2+(x_k.target(i).state(2,1)-y_s)^2);
            polar(i).azimuth=atan2((x_k.target(i).state(2,1)-y_s),(x_k.target(i).state(1,1)-x_s));
            
            polar(i).range=polar(i).range + w(1);
            polar(i).azimuth=polar(i).azimuth + w(2);
        end
    else
        polar(i).range=[];
        polar(i).azimuth=[];
    end   
end


for i=1:false_alarms_per_frame
        range=unifrnd(p.sensor(sensor_id).range(1,1), p.sensor(sensor_id).range(1,2));
        if range < (p.sensor(sensor_id).range(2) - p.sensor(sensor_id).range(1))
            azimuth=unifrnd(p.sensor(sensor_id).azimuth(1,1), p.sensor(sensor_id).azimuth(1,2));
            false_alarms(i).range=range;
            false_alarms(i).azimuth=azimuth;
        end
end

end
