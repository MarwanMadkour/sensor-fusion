function [associated,unasso_meas] = dataAssociation(p, Pk_hat, zk_hat, z, R,Hk_1) % add additional parameters, if necessary
    
gamma=chi2inv(0.95,2);
min_dist = inf;
j=1;
unasso_meas.range=[];
for i=1:numel(z.range)
    
    vk = [z.range(i);z.azimuth(i)] - zk_hat;
    
    Sk = Hk_1 * Pk_hat * Hk_1' + R;
    
    Dz = sqrt(vk' * inv(Sk) * vk);

    if Dz < min_dist  
        associated.range =z.range(i);
        associated.azimuth=z.azimuth(i);
        min_dist=Dz;
    else
        
    end
    if Dz>gamma
        unasso_meas.range(j)= z.range(i);
        unasso_meas.azimuth(j)=z.azimuth(i);
        j=j+1;
    end
     
end

if min_dist>gamma
    associated=[];
end

end