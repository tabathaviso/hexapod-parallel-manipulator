
% compute cost function

function CF = cost_function(x, s_nom, u_nom, lo_nom, p_real, delta_l, R, m)
    % Extract the optimization variables from the input vector x
    s_nom = reshape(x(1:numel(s_nom)), size(s_nom));
    u_nom = reshape(x(numel(s_nom)+1:numel(s_nom)+numel(u_nom)), size(u_nom));
    lo_nom = reshape(x(numel(s_nom)+numel(u_nom)+1:end), size(lo_nom));
    
    % Perform the nested loop calculation of e_ij
    e_ij = zeros(m, 6);
    for j = 1:m
        for i = 1:6
            e_ij(j,i) = (norm((p_real(j,1:3)' + R*s_nom(:,1) - u_nom(:,1)))^2 - (lo_nom(:,i) + delta_l(j,i))^2)^2;
        end
    end
    
    % Compute the cost function CF as the sum of e_ij
    CF = sum(e_ij(:));
end