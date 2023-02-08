% This files performs some testing for debugging

%% test the constrants in STL predicates
x = 1;
a = 1;
b = 0.1;

M = 1e6;
eps = 1e-8;
N = size(x, 1);
z = sdpvar(N, 1);
constraints=[];
for k = 1:N
    constraints = [constraints; (500000*(z(k)-1) + 0.00000001) <= 0.9];
    constraints = [constraints; 0.9 <= (500000*z(k) - 0.00000001)]; 
end
options = sdpsettings('debug', 1,...
'solver', 'mosek', ...
    'verbose',1);
% sol = optimize(constraints, -objective, options); % minimize t_rec(1)
sol = optimize(constraints, [],options); % minimize t_rec(1)
value(z)
check(constraints)

%% test the encoding for until
clear
z1 = [1;1;0;0;0;1;1;1;1;1];
z2 = [0;0;1;1;1;0;0;0;0;0];
a = 0;
b = 3;

N = size(z1, 1); % length

z_out = sdpvar(N,1); % z^phi
constr = [];

% compute binary for unbounded until first
z_unbound_until = intvar(N,1); % z^phi1 U phi2
z_unbound_until(N) = z2(N);
for k = N-1:-1:1
    [c_1, z_tmp] = bool_and([z1(k); z_unbound_until(k+1)]);
    constr = [constr; c_1];
    [c_2, z_unbound_until(k)] = bool_or([z2(k); z_tmp]);
    constr = [constr; c_2];
end

% now we have the binaries from (1) phi1 (2) phi2 (3) phi1 U phi2
% We can compute the binaries for phi = phi_1 U_[t+a, t+b] phi_2
% z^phi = z^G_[0,a]phi1 AND z^F_[a,b]phi2 AND z^F_[a,a]phi1_U_phi2
z_first_term = sdpvar(N,1); % 
z_second_term = sdpvar(N,1);
z_third_term = sdpvar(N,1);
for k = 1:N 
    atN = min(k+a, N);
    btN = min(k+b, N);
    [c_first_term, z_first_term(k)] = bool_globally(z1,k,atN);
    constr = [constr; c_first_term];
    [c_second_term, z_second_term(k)] = bool_finally(z2, atN, btN);
    constr = [constr; c_second_term];
    [c_third_term, z_third_term(k)] = bool_finally(z_unbound_until, atN, atN);
    constr = [constr; c_third_term];

    [c_first_and, z_first_and] = bool_and([z_first_term(k); z_second_term(k)]);
    constr = [constr; c_first_and];

    [c_second_and, z_out(k)] = bool_and([z_first_and; z_third_term(k)]);
    constr = [constr; c_second_and];
end
options = sdpsettings('debug', 1,...
'solver', 'mosek', ...
    'verbose',0);
sol = optimize(constr, [],options); % minimize t_rec(1)
value(z_first_term)
% check(constr)