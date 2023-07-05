% TABATHA VISO - RBE521 - LEGGED ROBOTICS
% Calibrates a hexapod parallel manipulator 

clear all

%% DEFINE GIVENS
% nominal values of kinematic parameters in mm 
u_nom = [305.4001, -56.4357, -248.9644, -248.9644, -56.4357, 305.4001;...
        111.1565, 320.0625, 208.9060, -208.9060, -320.0625, -111.1565;...
        0,0,0,0,0,0];
s_nom = [92.1597, 27.055, -119.2146, -119.2146, 27.055, 92.1597;...
        84.4488, 122.037, 37.58822, -37.5882, -122.037, -84.4488;...
        0,0,0,0,0,0];
lo_nom = [604.8652, 604.8652, 604.8652, 604.8652, 604.8652, 604.8652];
kinparams_nom = [s_nom; u_nom; lo_nom];

% real values of kinematic parameters in mm
u_real = [305.2599, -55.2814, -244.7954, -252.5755, -53.9678, 302.4266;...
        115.0695, 322.9819, 208.0087, -211.8783, -320.6115, -109.4351;...
        2.6210, 4.2181, 3.9365, -3.0128, 4.3181, 3.3812];
s_real = [96.6610, 22.2476, -122.4519, -120.6859, 24.7769, 91.3462;...
        81.7602, 125.2511, 36.6453, -34.4565, -125.0489, -80.9866;...
        1.0684, -0.5530, 4.3547, -4.9014, -4.8473, 0.2515];
lo_real = [604.4299, 607.2473, 600.4441, 605.9031, 604.5251, 600.0616];
kinparams_real = [s_real; u_real; lo_real];

%% STEP 1: choose m configurations
% find all workspace configs but don't show the figure
list_of_configs = workspace(u_nom, s_nom, 800, false); 

% find configs on/near the boundary
%boundary_configs = workspace_boundary(list_of_configs, 0)
boundary_configs = [-300, 525, 800, 0, 0, 0;
                    -465, -300, 800, 0, 0, 0;
                    -505, 200, 800, 0, 0, 0;
                    320, 435, 800, 0, 0, 0;
                    500, 245, 800, 0, 0, 0;
                    600, 10, 800, 0, 0, 0;
                    545, -165, 800, 0, 0, 0;
                    -310, -515, 800, 0, 0, 0;
                    145, -520, 800, 0, 0, 0;
                    155, 515, 800, 0, 0, 0];

% define the number of boundary configs we will use 
m = size(boundary_configs,1);
fprintf('There are %d configurations on/near the boundary.\n', m);

% using the first pose to get R matrix in Euler config
[l n R s] = IK(boundary_configs(1,:)', s_nom, u_nom);

for j = 1:m
    %% STEP 2: use IK to obtain leg lengths 
    % find leg lengths for boundary configs, also return XYZ Euler R matrix
    l_nom(j,:)= IK(boundary_configs(j,:)', s_nom, u_nom);

    %% STEP 3: implement leg lengths
    % using nominal leg lengths, final "real" ee pose for boundary configs
    p_real(j,:) = FK(boundary_configs(j,:)', l_nom(j,:), s_real, u_real);

    %% STEP 4: "measure" real leg lengths
    % find leg lengths for "real" poses of boundary configs
    l_real(j,:) = IK(p_real(j,:)', s_nom, u_nom);
end

% finding leg length variations from IK, nominal values 
delta_l = l_nom - lo_nom;
delta_s = s_real - s_nom;
delta_u = u_real - u_nom;
delta_lo = lo_real - lo_nom;
delta_kinparams = [delta_s; delta_u; delta_lo];

%% STEP 5: obtain cost function
CF = @(x) cost_function(x, s_nom, u_nom, lo_nom, p_real, delta_l, R, m); 

%% STEP 6: minimize the cost function
% Set options for lsqnonlin
options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt');

% optimize, using nominal kinematic parameters as initial guess
[x, resnorm] = lsqnonlin(CF, delta_kinparams, [], [], options);

% extract optimized variables s_nom, u_nom, and lo_nom
s_calc = reshape(x(1:numel(s_nom)), size(s_nom));
u_calc = reshape(x(numel(s_nom)+1:numel(s_nom)+numel(u_nom)), size(u_nom));
lo_calc = reshape(x(numel(s_nom)+numel(u_nom)+1:end), size(lo_nom));

% print results of calibration
disp('Calibrated Simulated Real Kinematic Parameters:');
disp(s_calc);
disp(u_calc);
disp(lo_calc);
disp('Minimized Cost Function Value:');
disp(resnorm);

% plot errors before and after calibration
num_params = 1:42;

before_calibration = abs(kinparams_real - kinparams_nom);
after_calibration = abs(kinparams_real - [s_calc; u_calc; lo_calc]);
before_calibration = reshape(before_calibration, 42,1);
after_calibration = reshape(after_calibration, 42, 1);
ones_for_scale = ones(42,1);

results = [before_calibration, ones_for_scale, after_calibration];

figure;
bar3(num_params, results, 'grouped');
ylabel('Kinematic Parameters');
zlabel('Error (mm)');
title('Identification Results');
legend('Before Calibration', 'Ones for Scale', 'After Calibration');
ylim([1 42]);
%zlim([0 5]);
