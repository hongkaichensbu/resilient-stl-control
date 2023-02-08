% This file tests the MILP encoding of the two objectives

clear 
close all
% z_t = [0;1;1;0;0;0;1;1;1;0]; % table 1
z_t = randi([0 1],15,1); % random

%% Test the t_dur in table 1
constraints = [];
[c_dur, t_dur] = tdur(z_t);
constraints = [constraints; c_dur];

options = sdpsettings('debug', 1,...
'solver', 'mosek', ...
    'verbose',1);
sol = optimize(constraints, -t_dur(1), options);

durability = value(t_dur);
fprintf('t_dur: %i(time units)\n', durability);

%% Test the t_rec in table 1
constraints = [];
[c_rec, t_rec] = trec(z_t);
constraints = [constraints; c_rec];

options = sdpsettings('debug', 1,...
'solver', 'mosek', ...
    'verbose',1);
sol = optimize(constraints, t_rec(1), options);

recoverability = value(t_rec);
fprintf('t_rec: %i(time units)\n', recoverability);