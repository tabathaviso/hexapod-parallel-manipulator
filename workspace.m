% TABATHA VISO - RBE521 - LEGGED ROBOTICS
% Finds points within the workspace of a kinematic parallel manipulator
% Considering only Z=800mm, horizontal top platform a=b=c=0

function [meshPoints] = workspace(u_nom, s_nom, z, dispFigure)

syms X Y Z

% leg length limits in mm
l_min = 604.8652;
l_max = 1100;

% define sphere centerpoints and radii (ignoring z values) 
spheres = u_nom - s_nom;
r = l_max;

% create circle equations at z=800 height
circles = arrayfun(@(i) (X - spheres(1,i)).^2 + (Y - spheres(2,i)).^2 + (z - spheres(3,i))^2 - r^2, 1:size(spheres,2), 'UniformOutput', false);

% search range, only looking at every 5mm to save computating time
range = -1000:5:1000;
meshPoints = [];

for x = range
    for y = range
        isInWorkspace = false;

        % check if point is in the workspace (i.e., in every circle)
        if (x - spheres(1,1))^2 + (y - spheres(2,1))^2 + (z - spheres(3,1))^2 - r^2 <= 0
            if (x - spheres(1,2))^2 + (y - spheres(2,2))^2 + (z - spheres(3,1))^2 - r^2 <= 0
                if (x - spheres(1,3))^2 + (y - spheres(2,3))^2 + (z - spheres(3,1))^2 - r^2 <= 0
                    if (x - spheres(1,4))^2 + (y - spheres(2,4))^2 + (z - spheres(3,1))^2 - r^2 <= 0
                        if (x - spheres(1,5))^2 + (y - spheres(2,5))^2 + (z - spheres(3,1))^2 - r^2 <= 0
                            if (x - spheres(1,6))^2 + (y - spheres(2,6))^2 + (z - spheres(3,1))^2 - r^2 <= 0
                                isInWorkspace = true;
                            end
                        end
                    end
                end
            end
        end

        % save workspace points in this list
        if isInWorkspace
            meshPoints = [meshPoints; x, y];
        end
    end
end

% show me how many workspace points I found
%fprintf('There are %d configurations in the workspace.\n', size(meshPoints,1));

% add constant z value to workspace points, as well as angles
meshPoints(:,end+1) = z; 
meshPoints(:,end+1:end+3) = 0;

if dispFigure == true
    % plot
    figure;
    hold on;
    %scatter(u_nom(1,:), u_nom(2,:), 'r', 'DisplayName', 'u\_nom', 'Marker', 'o', 'LineWidth', 1.5);
    %scatter(s_nom(1,:), s_nom(2,:), 'b', 'DisplayName', 's\_nom', 'Marker', 'x', 'LineWidth', 1.5);
    %scatter(spheres(1,:), spheres(2,:), 'g', 'DisplayName', 'centerpoints', 'Marker', 's', 'LineWidth', 1.5);
    cellfun(@(circle) fimplicit(circle, 'LineWidth', 1), circles);
    scatter(meshPoints(:,1),meshPoints(:,2));
    hold off;
    axis equal;
    xlim([-1000, 1000]);
    ylim([-1000, 1000]);
    xlabel('X (mm)');
    ylabel('Y (mm)');
    title('Workspace at z=800mm');
end
