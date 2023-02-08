% This file construct the constraints and objective from z_t (the boolean
% of STL predicate or general formula) to t_dur
function [constr, tdur_value] = tdur(z)
N = size(z, 1);
constr = [];

tdur_value = intvar(N, 1);

counter_c_one = intvar(N, 1); % c_t^1
counter_c_two = intvar(N, 1); % c_t^2

counter_c_one(N) = 0; % the last is zero
counter_c_two(N) = 0; % the last is zero

for t= N-1:-1:1
    % c_t^1 = z_t*(c_t+1^1 +1)
    % c_t^1 lower bound and upper bound are 0 and N
    [c1, counter_c_one(t)]  = if_then_else(z(t),counter_c_one(t+1)+1,  0, N);
    % c_t^2 = (1-z_t)*(c_t+1^1 + c_t+1^2)
    % c_t^2 lower bound and upper bound are 0 and N (still N, when violate at 0 then maintain satisfaction afterwards)   
    [c2, counter_c_two(t)]  = if_then_else(1-z(t),counter_c_one(t+1)+counter_c_two(t+1),  0, N);
    constr = [constr; c1; c2];
end

for t=1:N
    tdur_value(t) = counter_c_one(t) + counter_c_two(t);
end

end

