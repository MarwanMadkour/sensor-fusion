function [xk_hat, Pk_hat,unasso_meas,associated] = kalmanFilterNN(p, i, xk_1_hat, Pk_1_hat, z, R) % add additional parameters, if necessary

dt=p.sensor(1).sampling_time;
x_s=p.sensor(1).start_state(1,1);
y_s=p.sensor(1).start_state(2,1);

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

Hk_1=[(xk_hat(1)-x_s)/sqrt((xk_hat(1)-x_s)^2 + (xk_hat(2)-y_s)^2)  (xk_hat(2)-y_s)/sqrt((xk_hat(1)-x_s)^2 + (xk_hat(2)-y_s)^2) 0 0;
 -(xk_hat(2) - y_s) / ( (xk_hat(2) - y_s)^2 + (xk_hat(1) - x_s)^2 )  (xk_hat(1) - x_s) / ( (xk_hat(2) - y_s)^2 + (xk_hat(1) - x_s)^2 ) 0 0];


Pk_hat = F_k * Pk_1_hat * F_k' + Q_k; 

zk_hat = [sqrt((xk_hat(1)-x_s)^2 + (xk_hat(2) - y_s)^2); atan2((xk_hat(2)-y_s),(xk_hat(1)-x_s)) ];

[zk,unasso_meas] = dataAssociation(p, Pk_hat, zk_hat, z, R,Hk_1);
% zk = associated.z;
% R_k_1 = associated.R;


if ~(isempty(zk))
    zk=[zk.range; zk.azimuth];
    vk = zk - zk_hat; 
    
    Sk = Hk_1 * Pk_hat * Hk_1' + R;
    
    Wk = Pk_hat * Hk_1' * inv(Sk);
    
    xk_hat = xk_hat + Wk * vk; 
    
    Pk_hat = Pk_hat - Wk * Sk * Wk';
    associated=1;
else
    associated=0;
end

end