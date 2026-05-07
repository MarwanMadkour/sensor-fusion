function [x1, y1, xdot1, ydot1, closest_x, closest_y, closest_xdot, closest_ydot, is_associated] = associateTracks(p,truth,tracks,i)

asso_range=p.tracker.asso_range;
is_associated=0;
min_dist=inf;
closest_x=0;
closest_y=0;
closest_xdot=0;
closest_ydot=0;

x1 = truth(end).target(i).state(1);
y1 = truth(end).target(i).state(2);
xdot1 = truth(end).target(i).state(3);
ydot1 = truth(end).target(i).state(4);
for j=1:numel(tracks)
    if tracks(j).status==2
        x2 = tracks(j).xk_1_hat{end}(1);
        y2 = tracks(j).xk_1_hat{end}(2);
        dist = sqrt((x2 - x1)^2 + (y2 - y1)^2);
        if dist<min_dist && dist<asso_range
            min_dist=dist;
            closest_x=x2;
            closest_y=y2;
            closest_xdot=tracks(j).xk_1_hat{end}(3);
            closest_ydot=tracks(j).xk_1_hat{end}(4);
            is_associated=1;
        end
    end
end















end
