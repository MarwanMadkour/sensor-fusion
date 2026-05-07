function [x0_hat, P0_hat] = trackInit(p,measurement,sensor_id)
% R = p.sensor(1).range_std^2; %=100
x_s=p.sensor(sensor_id).start_state(1,1);
y_s=p.sensor(sensor_id).start_state(2,1);
r=measurement.range;
a=measurement.azimuth;
b=exp(-1*(p.sensor(sensor_id).azimuth_std^2)/2);
x=(b^-1) * r * cos(a) + x_s;
y=(b^-1) * r * sin(a) + y_s;

co2=exp(-2*p.sensor(sensor_id).azimuth_std^2);
sig_r=p.sensor(sensor_id).range_std;
R_11 = (((b^-2) - 2) * (r^2) * cos(a).^2) + (0.5*(r^2 + sig_r^2) * (1 + co2*cos(2*a)));
R_22 = (((b^-2) - 2) * (r^2) * sin(a).^2) + (0.5*(r^2 + sig_r^2) * (1 - co2*cos(2*a)));
R_12 = (((b^-2) - 2) * (r^2) * cos(a) * sin(a)) + (0.5*(r^2 + sig_r^2) * (co2*sin(2*a)));

x0_hat = [x y 0 0]';

acc = (p.target(1).vmax/2)^2;
P0_hat = [R_11   R_12   0     0
          R_12   R_22   0     0
          0      0      acc   0
          0      0      0     acc]; %correct R and acc???



end
