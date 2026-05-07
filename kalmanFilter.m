function [xk_hat, Pk_hat, outside_gate,associated] = kalmanFilter(p, i,xk_1_hat, Pk_1_hat, z, R,dt,sensor_id) % add additional parameters, if necessary


x_s=p.sensor(sensor_id).start_state(1,1);
y_s=p.sensor(sensor_id).start_state(2,1);
%jacobian takes in the eqns and state
%h
F_k = [1 0 dt 0;
       0 1 0 dt;
       0 0 1 0;
       0 0 0 1];

G_k = [dt^2/2 0;
       0 dt^2/2;
       dt 0;
       0 dt];

Q_k = G_k * [p.target(1).process_noise_x^2 0; 0 p.target(1).process_noise_y^2] * G_k';%squared so variance


xk_hat = F_k * xk_1_hat;
% Hk_1=[(xk_hat(1)-x_s)/sqrt((xk_hat(1)-x_s)^2 + (xk_hat(2)-y_s)^2) 0 (xk_hat(2)-y_s)/sqrt((xk_hat(1)-x_s)^2 + (xk_hat(2)-y_s)^2) 0;
%  -(xk_hat(2) - y_s) / ( (xk_hat(2) - y_s)^2 + (xk_hat(1) - x_s)^2 ) 0 (xk_hat(1) - x_s) / ( (xk_hat(2) - y_s)^2 + (xk_hat(1) - x_s)^2 ) 0];
Hk_1=[(xk_hat(1)-x_s)/sqrt((xk_hat(1)-x_s)^2 + (xk_hat(2)-y_s)^2)  (xk_hat(2)-y_s)/sqrt((xk_hat(1)-x_s)^2 + (xk_hat(2)-y_s)^2) 0 0;
 -(xk_hat(2) - y_s) / ( (xk_hat(2) - y_s)^2 + (xk_hat(1) - x_s)^2 )  (xk_hat(1) - x_s) / ( (xk_hat(2) - y_s)^2 + (xk_hat(1) - x_s)^2 ) 0 0];

Pk_hat = F_k * Pk_1_hat * F_k' + Q_k; 

zk_hat = [sqrt((xk_hat(1)-x_s)^2 + (xk_hat(2) - y_s)^2); atan2((xk_hat(2)-y_s),(xk_hat(1)-x_s)) ];


[z,outside_gate, beta_zero] = PDA(p, Pk_hat, zk_hat, z,Hk_1,sensor_id);

sum_v = 0;
spread_sum=[0;0];
if ~isempty(z.range)
    for i=1:numel(z.range)
        v = [z.range(i) - zk_hat(1);z.azimuth(i) - zk_hat(2)];
        % if v(2)>pi
        %     v(2)=v(2)-2*pi;
        % elseif v(2)<-pi
        %     v(2) = v(2)+2*pi;
        % end
        sum_v = sum_v + v * z.beta(i);
        spread_sum = spread_sum + z.beta(i)*v*v' ;
    end
    spread_sum = spread_sum - sum_v*sum_v';
end

if ~(isempty(z.range))
 
    % vk = zk - zk_hat; 
    associated=1;
    Sk = Hk_1 * Pk_hat * Hk_1' + R;
    
    Wk = Pk_hat * Hk_1' * inv(Sk);
    
    xk_hat = xk_hat + Wk * sum_v; 
    
    p_c = Pk_hat - Wk * Sk * Wk';

    p_spread = Wk*spread_sum*Wk';

    Pk_hat = beta_zero*Pk_hat + (1-beta_zero)*p_c + p_spread;
else
    associated=0;
end

end