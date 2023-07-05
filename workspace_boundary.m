% TABATHA VISO - RBE521 - LEGGED ROBOTICS
% Find points along/near the boundary of the workspace of a kinematic parallel manipulator

function [boundary_points] = workspace_boundary(list_of_configs, threshold)

% extracts x and y coordinates from all the configurations in the workspace
x = list_of_configs(:, 1);
y = list_of_configs(:, 2);

% define the threshold distance from the maximum and minimum values
%threshold = 10; % adjust this value based on requirements

% finds points close to the maximum and minimum x and y values
max_x = max(x);
min_x = min(x);
max_x_points = list_of_configs(abs(x - max_x) <= threshold, :);
min_x_points = list_of_configs(abs(x - min_x) <= threshold, :);

max_y = max(y);
min_y = min(y);
max_y_points = list_of_configs(abs(y - max_y) <= threshold, :);
min_y_points = list_of_configs(abs(y - min_y) <= threshold, :);

% concats all points
boundary_points = [max_x_points; min_x_points; max_y_points; min_y_points];

% removes duplicate points, just in case
boundary_points = unique(boundary_points, 'rows');

%fprintf('There are %d configurations on/near the boundary.\n', size(boundary_points,1));

