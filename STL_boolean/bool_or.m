% This file implements the MILP encoding of the disjunction operator
% constr: constraints
% z_out: binary variables indicating satisfaction
function [constr, z_out] = bool_or(z)

z_out = binvar(1, 1);

N = size(z, 1);
constr = [];

for k = 1:N
    constr = [constr; z_out >= z(k)];
end
constr = [constr; z_out <= sum(z)];

end