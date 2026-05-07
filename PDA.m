function [z, outside_gate, beta_zero] = PDA(p, Pk_hat, zk_hat, z,Hk_1,sensor_id)
R=p.sensor(sensor_id).R;
p_g =p.tracker.p_g;
gamma=chi2inv(p_g,2);
p_d= p.sensor(sensor_id).probability_of_detection;
maxIndex=[];
j=1;
outside_gate.range=[];
for i = numel(z.range):-1:1
    vk = [z.range(i) - zk_hat(1);z.azimuth(i) - zk_hat(2)];
    Sk = Hk_1 * Pk_hat * Hk_1' + R;
    Dz = vk' * inv(Sk) * vk;
    if Dz > gamma
        outside_gate.range(j)=z.range(i);
        outside_gate.azimuth(j) = z.azimuth(i);
        
        j=j+1;
        z.range(i) = [];
        z.azimuth(i) = [];
        
    end
end

L_sum=0;
if ~isempty(z.range)
    for i=numel(z.range)
       z.L(i)=mvnpdf([z.range(i);z.azimuth(i)],zk_hat,Sk) * p_d/p.sensor(sensor_id).false_alarm_density;%Sk is not working???
       L_sum = L_sum+z.L(i);
    end
end

if ~isempty(z.range)
    for i=numel(z.range)
        z.beta(i) = z.L(i)/(1 - p_d * p_g + L_sum);
    end
    % [~, maxIndex] = max(z.beta);
end
beta_zero = (1 - p_d*p_g)/(1-p_d*p_g + L_sum);



end