% This file construct the binary variables and MILP constraints for the linear STL predicates
function [constr, z] = bool_mu(x, a, b)
% ax_t >= b 
% therefore, ax - b >= 0
% a, b \in R
% x is (part of) the system state
M = 1e4;   % M cannot be too large, or use sdpvar for z
eps = 1e-8;

N = size(x, 1);
z = binvar(N, 1,'full');
% z = sdpvar(N, 1,'full');

constr=[];
for k = 1:N
    constr = [constr; M*(z(k)-1) <= a*x(k) - b];
    constr = [constr; a*x(k) - b <= (M)*z(k) - eps]; 
end

end
