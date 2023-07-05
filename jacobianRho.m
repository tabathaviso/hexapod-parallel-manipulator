% TABATHA VISO - RBE521 - LEGGED ROBOTICS
% calculate identification matrix

function J = jacobianRho(pose, s, u)

if nargin > 1 % use provided kinematic parameters
    [l n R s] = IK(pose, s, u);
else % calculate kinematic parameters
    [l n R s] = IK(pose);
end

J = [n(:,1)'*R, -n(:,1)', -1, zeros(1,35);
    zeros(1,7), n(:,2)'*R, -n(:,2)', -1, zeros(1,28);
    zeros(1,14), n(:,3)'*R, -n(:,3)', -1, zeros(1,21);
    zeros(1,21), n(:,4)'*R, -n(:,4)', -1, zeros(1,14);
    zeros(1,28), n(:,5)'*R, -n(:,5)', -1, zeros(1,7);
    zeros(1,35), n(:,6)'*R, -n(:,6)', -1;];