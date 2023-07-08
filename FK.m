% TABATHA VISO - RBE521 - LEGGED ROBOTICS
% Forward Kinematics of Parallel Robots with Prismatic Legs
% comment out the B matrix for XYZ or ZYZ Euler Angles

function p = FK(p0, lg, s, u)

%% robot params (given)
Rm = 250.9339/2; % moving platform radius in mm
Rf = 634.2376/2; % fixed platform radius in mm
alpha = 40*pi/180;
beta = 85*pi/180;

%% initial guess
%p0 = [0 0 100 0 0 0]';
%lg = [250.1730 247.7072 253.3073 277.6336 278.4548 254.3322];
pose(:,1) = p0; % step 1
i = 2;

%% iterative loop to improve guess
Dp = 1;
e = 0.0001;

while Dp > e
    if nargin > 1 % use provided kinematic parameters
        J = jacobianV(pose(:,i-1), s, u); % step 2
    else % calculate kinematic parameters
        J = jacobianV(pose(:,i-1)); % step 2
    end 
    
    % step 3, euler angles in degrees
    a = pose(4, i-1) *pi/180;
    b = pose(5, i-1) *pi/180;
    c = pose(6, i-1) *pi/180; 
    
    % ZYZ Euler Angles
    %B = [0, -sin(a), sin(b)*cos(a); 0, cos(a), sin(b)*sin(a); 1, 0, cos(b)]; 
    
    % XYZ Euler Angles
    B = [1, 0, sin(b); 0, cos(a), -sin(a)*cos(b); 0, sin(a), cos(b)*cos(a)]; 
    T = [eye(3), zeros(3,3); zeros(3,3), B]; 
    
    if nargin > 1 % use provided kinematic parameters
        [l n R s] = IK(pose(:, i-1), s, u); % step 4 
    else % calculate kinematic parameters
        [l n R s] = IK(pose(:, i-1)); % step 4
    end 
    
    Dl = lg' - l';
    pose(:, i) = pose(:, i-1) + pinv(J*T)*Dl; % step 5
    Dp = norm(pose(:, i) - pose(:, i-1), 2); % step 6
    i = i+1;
end 

p = pose(:,end);


