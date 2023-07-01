% TABATHA VISO - RBE521 - LEGGED ROBOTICS
% Inverse Kinematics of Parallel Robots with Prismatic Legs
% comment out XYZ or ZYZ Euler Angles rotation matrix

function [l n R s] = IK(pose)

% givens
%pose = [10 20 150 0 0 0]';  % desired pose [x,y,z,a,b,c]T in mm and degrees

o = pose(1:3,1); % translation vector from base frame origin to local frame origin
a = pose(4)*pi/180; % euler angle infromation
b = pose(5)*pi/180;
c = pose(6)*pi/180;

Rm = 250/2; % moving platform radius in mm
Rf = 650/2; % fixed platform radius in mm
alpha = 40*pi/180;
beta = 80*pi/180;

% rotation matrices
Rxa = [1, 0, 0; 0, cos(a), -sin(a); 0, sin(a), cos(a)];
Ryb = [cos(b), 0, sin(b); 0, 1, 0; -sin(b), 0, cos(b)];
Rza = [cos(a), -sin(a), 0; sin(a), cos(a), 0; 0, 0, 1];
Rzc = [cos(c), -sin(c), 0; sin(c), cos(c), 0; 0, 0, 1];

%R = Rza*Ryb*Rzc; % ZYZ Euler Angles
R = Rxa*Ryb*Rzc; % XYZ Euler Angles

% upper joint positions w.r.t. local frame
s1 = [Rm*cos(beta/2), Rm*sin(beta/2), 0]';
s2 = [-Rm*sin(pi/6-beta/2), Rm*cos(pi/6-beta/2), 0]';
s3 = [-Rm*sin(pi/6+beta/2), Rm*cos(pi/6+beta/2), 0]';
s4 = [-Rm*cos(pi/3-beta/2), -Rm*sin(pi/3-beta/2), 0]';
s5 = [-Rm*cos(pi/3+beta/2), -Rm*sin(pi/3+beta/2), 0]';
s6 = [Rm*cos(beta/2), -Rm*sin(beta/2), 0]';
s = [s1 s2 s3 s4 s5 s6];

% lower joint positions w.r.t. base frame
u1 = [Rf*cos(alpha/2), Rf*sin(alpha/2), 0]';
u2 = [-Rf*sin(pi/6-alpha/2), Rf*cos(pi/6-alpha/2), 0]';
u3 = [-Rf*sin(pi/6+alpha/2), Rf*cos(pi/6+alpha/2), 0]';
u4 = [-Rf*cos(pi/3-alpha/2), -Rf*sin(pi/3-alpha/2), 0]';
u5 = [-Rf*cos(pi/3+alpha/2), -Rf*sin(pi/3+alpha/2), 0]';
u6 = [Rf*cos(alpha/2), -Rf*sin(alpha/2), 0]';
u = [u1 u2 u3 u4 u5 u6];

%% EULER ANGLES
for i=1:6
    L(:,i) = o + R*s(:,i) - u(:,i); % leg vector
    l(i) = norm(L(:,i),2); % leg length
    n(:,i) = L(:,i)/l(:,i); % unit vector of leg
end

