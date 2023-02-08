% This file test a predicate STL formula
clear 
close all

params.H = 10;

constraints = [];

%% 
x = sdpvar(params.H, 1,'full'); % inputs

% constraints and z_t for STL predicate 1*x>=0.1
[c_p, z_p] = bool_mu(x(:,1), 1, 0.1);
constraints = [constraints; c_p];

% construct t_rec and t_dur with the z_t of the STL general formula
[c_tec, t_rec] = trec(z_p);
constraints = [constraints; c_tec];

[c_dur, t_dur] = tdur(z_p);
constraints = [constraints; c_dur];

objective = -t_rec(1);
constraints = [constraints;  t_dur(1) >= 5];


options = sdpsettings('debug', 1,...
'solver', 'mosek', ...
    'verbose',1);
sol = optimize(constraints, -objective, options);

recoverability = value(t_rec);
fprintf('t_rec: %i(time units)\n', recoverability);
durability = value(t_dur);
fprintf('t_dur: %i(time units)\n', durability);
x_value = value(x);
fprintf('x: %i\n', x_value);
z_p_value = value(z_p);
fprintf('z_p: %i\n', z_p_value);