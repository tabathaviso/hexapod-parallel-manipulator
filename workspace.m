
syms X Y

% nominal values of kinematic parameters in mm
u_nom = [305.4001, -56.4357, -248.9644, -248.9644, -56.4357, 305.4001;...
         111.1565, 320.0625, 208.9060, -208.9060, -320.0625, -111.1565];
s_nom = [92.1597, 27.055, -119.2146, -119.2146, 27.055, 92.1597;...
         84.4488, 122.037, 37.58822, -37.5882, -122.037, -84.4488];
lo_nom = [604.8652, 604.8652, 604.8652, 604.8652, 604.8652, 604.8652];

% real values of kinematic parameters in mm
u_real = [305.2599, -55.2814, -244.7954, -252.5755, -53.9678, 302.4266;...
         115.0695, 322.9819, 208.0087, -211.8783, -320.6115, -109.4351];
s_real = [96.6610, 22.2476, -122.4519, -120.6859, 24.7769, 91.3462;...
         81.7602, 125.2511, 36.6453, -34.4565, -125.0489, -80.9866];
lo_real = [604.4299, 607.2473, 600.4441, 605.9031, 604.5251, 600.0616];

% define sphere centerpoints
spheres = u_nom - s_nom;

radii = 123456789;


% find where the spheres intersect and define the workspace
intersection_points=[];

for i = 1:5
    eq1 = (X-spheres(1,i))^2 + (Y-spheres(2,i))^2;
    eq2 = (X-spheres(1,i+1))^2 + (Y-spheres(2,i+1))^2;
    [x,y] = vpasolve([eq1 == (radii)^2, eq2 == (radii)^2],[X,Y]);
    intersection_points = vertcat(intersection_points,[x,y]);
end

eq1 = (X-spheres(1,1))^2 + (Y-spheres(2,1))^2; 
eq2 = (X-spheres(1,6))^2 + (Y-spheres(2,6))^2;
[x,y] = vpasolve([eq1 == (radii)^2, eq2 == (radii)^2],[X,Y]);
intersection_points = vertcat(intersection_points,[x,y]);
intersection_points = double(intersection_points);

% only collect the points within the workspace
workspace_points = [];
num = size(intersection_points);

for i = 1:num
    count = 0;
    for j = 1:6
        if (intersection_points(i,1)-spheres(1,j))^2 + (intersection_points(i,2)-spheres(2,j))^2 <= radii^2
            count = count+1;
        end
    end
    if count >= 6
        workspace_points = vertcat(workspace_points,[intersection_points(i,1),intersection_points(i,2)]);
    end
end

