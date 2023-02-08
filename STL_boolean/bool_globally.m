% This file construct the binary variables and MILP constraints for a
% globally formula
% constr: constraints
% z_out: single binary variable indicating satisfaction
% a and b are time points starting at time t

function [constr, z_out] = bool_globally(z, a, b)

N = size(z, 1);
z_out = sdpvar(N,1,'full');
constr = [];
for k = 1:N
    atN = min(k+a, N);
    btN = min(k+b, N);
    [c_and, z_out(k)] = bool_and(z(atN:btN));
    constr = [constr; c_and];
end

% [constr, z_out] = bool_and(z(a:b));

end