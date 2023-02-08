% This file constrains the ub and lb of the 2 states in the system trajectory
function [constr, z_out] = bool_mu_region(x, x_lb, x_ub, y_lb, y_ub)

[bds_x1, z_x_lb] = bool_mu(x(:,1), 1, x_lb);
[bds_x2, z_x_ub] = bool_mu(x(:,1), -1, -x_ub);
[bds_y1, z_y_lb] = bool_mu(x(:,2), 1, y_lb);
[bds_y2, z_y_ub] = bool_mu(x(:,2), -1, -y_ub);
constr = [bds_x1; bds_y1; bds_x2; bds_y2];
% 
N = size(x, 1);
z_out = sdpvar(N, 1,'full');
for t = 1:N   
    [constr_and, z_out(t)] = bool_and([z_x_lb(t); z_y_lb(t); z_x_ub(t); z_y_ub(t)]);
    constr = [constr; constr_and];
end

% constr =bds_x2;
% z_out = z_x_ub;
end

