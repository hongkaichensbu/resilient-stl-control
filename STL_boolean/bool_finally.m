% This file construct the binary variables and MILP constraints for a
% finally formula
% constr: constraints
% z_out: binary variables indicating satisfaction
% a and b are absolute time points
function [constr, z_out] = bool_finally(z, a, b)

N = size(z, 1);

z_out = intvar(N,1);
constr = [];

for k = 1:N
    atN = min(k+a, N);
    btN = min(k+b, N);
    [c_or, z_out(k)] = bool_or(z(atN:btN));
    constr = [constr; c_or];
end

% [constr, z_out] = bool_or(z(a:b));

end