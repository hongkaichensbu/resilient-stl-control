% This file implements the MILP encoding of the conjunction operator
% constr: constraints
% z_out: binary variables indicating satisfaction
function [constr, z_out] = bool_and(z)

z_out = binvar(1, 1,'full');

N = size(z, 1);
constr = [];

for k = 1:N
    constr = [constr; z_out <= z(k)];
end
constr = [constr; z_out >= 1 - N + sum(z)];


end