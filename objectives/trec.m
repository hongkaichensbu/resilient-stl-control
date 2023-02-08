% This file construct the constraints and objective from z_t (the boolean
% of STL predicate or general formula) to t_rec
function [constr, trec_value] = trec(z)
N = size(z, 1);
constr = [];

trec_value = intvar(N, 1); % t_rec
counter_c_zero = intvar(N, 1); % c_t^0

counter_c_zero(N) = 0; % the last is zero

for t= N-1:-1:1
    % c_t^0 = (1-z_t)*(c_t+1^0 +1)
    % c_t^0 lower bound and upper bound are 0 and N
    [c1, counter_c_zero(t)]  = if_then_else(1-z(t),counter_c_zero(t+1)+1,  0, N);
    constr = [constr; c1];
end

for t=1:N
    trec_value(t) = counter_c_zero(t);
end

end

