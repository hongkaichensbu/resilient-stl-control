% This file test an STL Until formula
clear 
close all

params.H = 10;

constraints = [];

%% 
x = sdpvar(params.H, 1,'full'); % inputs
% force values to x to test the objective
% x = [1;1;-1;-1;-1;1;1;1;1;1]; % table 1

% constraints and z_t for STL predicate 1*x-0.1>=0  ==>  x>= 0.1
[c_p1, z_p1] = bool_mu(x(:,1), 1, 0.1);
constraints = [constraints; c_p1];
% constraints and z_t for STL predicate -1*x-0.5>=0  ==>  x<= -0.5
[c_p2, z_p2] = bool_mu(x(:,1), -1, 0.5);
constraints = [constraints; c_p2];

% constraints and z_t for STL formula (1*x>=0.1) U_[0,3] (-1*x>=0.5)
a1 = 1;
b1 = 3;
[c_u, z_u] = bool_until(z_p1, z_p2, a1, b1);
constraints = [constraints; c_u];

% construct t_rec and t_dur with the z_t of the STL general formula
[c_rec, t_rec] = trec(z_u);
constraints = [constraints; c_rec];

[c_dur, t_dur] = tdur(z_u);
constraints = [constraints; c_dur];

objective = -t_rec(1);
% constraints = [constraints;  t_dur(1) <=2];


options = sdpsettings('debug', 1,...
'solver', 'mosek', ...
    'verbose',1);
% sol = optimize(constraints, -objective, options); % minimize t_rec(1)
sol = optimize(constraints, objective, options); % maximize t_rec(1)

check(constraints)
recoverability = value(t_rec);
durability = value(t_dur);
fprintf('t_rec: %i(time units)    t_dur: %i(time units)\n', [recoverability,durability]');
x_value = value(x);
fprintf('x: %i\n', x_value);
z_p1_value = value(z_p1);
z_p2_value = value(z_p2);
fprintf('z_p1: %i   z_p2: %i\n', [z_p1_value,z_p2_value]');
% fprintf('z_p1: %i\n', z_p1_value);
z_u_value = value(z_u);
fprintf('z_u: %i\n', z_u_value);