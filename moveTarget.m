function x_k = moveTarget(p, k_time, xk_1,dt) % add additional parameters, if necessary

% dt=p.sensor(1).sampling_time;

%[x y xdot ydot]
F_k = [1 0 dt 0;
    0 1 0 dt;
    0 0 1 0;
    0 0 0 1];

G_k = [dt^2/2 0;
    0 dt^2/2;
    dt 0;
    0 dt];

for i=1:p.number_of_targets
    x_k.target(i).state=[];

    if k_time < p.target(i).start_time
        x_k.target(i).status = 0;
    elseif k_time >= p.target(i).start_time && (xk_1.target(i).status == 0) && k_time <= p.target(i).end_time
        x_k.target(i).state = p.target(i).start_state;
        x_k.target(i).status = 1;
    elseif k_time >= p.target(i).start_time && k_time <= p.target(i).end_time
        v_kx =mvnrnd(p.target(1).mu, p.target(1).process_noise_x^2 );  %std dev
        v_ky =mvnrnd(p.target(1).mu,p.target(1).process_noise_y^2);
        x_k.target(i).state = F_k * xk_1.target(i).state + G_k * [v_kx;v_ky];
        x_k.target(i).status = 1;
    elseif k_time> p.target(i).end_time
        x_k.target(i).status = 0;
    end
end
% if k == p.target(2).start_time
%     x_k.target2 = p.target(2).start_state;
% elseif k > p.target(2).start_time &&  k <= p.target(2).end_time
%     v_kx =mvnrnd(p.target(1).mu, p.target(1).process_noise_x^2 );  %std dev
%     v_ky =mvnrnd(p.target(1).mu,p.target(1).process_noise_y^2);
%     x_k.target2 = F_k * xk_1.target2 + G_k * [v_kx;v_ky];
% end

end