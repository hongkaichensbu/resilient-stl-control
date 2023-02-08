% This file encode the if-then-else logic into MILP constraints
% y = b*i 
% b: binary
% i: integer
% if b == 1, y = c; else y = 0
% x_lb and x_ub are small and large enough numbers
function [constr, y] = if_then_else(b, x, x_lb, x_ub) 

N = size(b, 1);
assert(all(size(b)==size(x)), 'unmatch size');

y = intvar(N, 1);

constr = [];
for t = 1:N
    constr = [constr; x_lb * b(t) <= y(t)];
    constr = [constr; y(t)<= x_ub * b(t)];
    constr = [constr; x(t) - x_ub*(1-b(t)) <= y(t)];
    constr = [constr; y(t) <= x(t) - x_lb*(1-b(t))];
end

end