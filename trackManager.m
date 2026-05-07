function [tracks,confirmed_tracks,term_latency] = trackManager(p,tracks, confirmed_tracks,k,term_latency,sensor_id)
p_s = p.target(1).survival_probability;
p_d = p.sensor(sensor_id).probability_of_detection;
n_tent=p.tracker.n_tent;
m_tent=p.tracker.m_tent;
n_conf=p.tracker.n_conf;
m_conf=p.tracker.m_conf;
for i=numel(tracks):-1:1

    if tracks(i).status == 1%tentative


        if numel(tracks(i).associated) >= n_tent
            asso_count = 0;
            for j=n_tent:-1:1
                if tracks(i).associated(j) == 1
                    asso_count =  asso_count + 1;
                end
            end
            if asso_count >= m_tent
                tracks(i).status = 2;%promote track to confirmed
                tracks(i).conf_since=1;
                tracks(i).confirmedat = k;
            else
                 tracks(i)=[];
            end
        end  
       
            
    
    
    elseif tracks(i).status == 2%confirmed

        tracks(i).conf_since = tracks(i).conf_since + 1;
        if tracks(i).conf_since > n_conf
            asso_count = 0;
            for r=0:(n_conf-1)
                if tracks(i).associated(end-r) == 1
                    asso_count = asso_count + 1;
                end
            end

            if asso_count < m_conf
                if ~exist('confirmed_tracks', 'var')
                    confirmed_tracks = struct();
                end
                tracks(i).terminatedat=k;
                max_end_time = -inf;
                for r = 1:numel(p.target)
                    
                    if p.target(r).end_time <= k && p.target(r).end_time > max_end_time && ((k - p.target(r).end_time) <(n_conf+3))
                            max_end_time = p.target(r).end_time;
                            term_latency.target(r) = k - max_end_time;
                    end
                end
                
                confirmed_tracks=[confirmed_tracks tracks(i)];
                % confirmed_tracks
                tracks(i)=[];
            end
        end
        
    end

end



end