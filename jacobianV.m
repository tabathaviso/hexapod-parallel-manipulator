% TABATHA VISO - RBE521 - LEGGED ROBOTICS
% calculate velocity jacobian

function J = jacobianV(pose, s, u)

if nargin > 1 % use provided kinematic parameters
    [l n R s] = IK(pose, s, u);
else % calculate kinematic parameters
    [l n R s] = IK(pose);
   end

J = [n(:,1)', cross(R*s(:,1), n(:,1))';
    n(:,2)', cross(R*s(:,2), n(:,2))';
    n(:,3)', cross(R*s(:,3), n(:,3))';
    n(:,4)', cross(R*s(:,4), n(:,4))';
    n(:,5)', cross(R*s(:,5), n(:,5))';
    n(:,6)', cross(R*s(:,6), n(:,6))'];