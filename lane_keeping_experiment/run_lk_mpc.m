% This file run an experiment in which a vechile is controlled by the
% resilient STL controller in a lane-keeping problem.
% The controller computes multiple optimal solutions and choose a control action based on a DM.
% The first control action is executed and the controller re-computes at
% the next time step. The controller repeat for N steps.

clear
config_lk;

x_0 = [5, 10, 0, 2]';
% x_0 = [5, 6, 0, 2]';
deltaT = 0.1;

A = [0,1,0,0;
    0, a_c1, 0, a_c2;
    0, 0, 0, 1;
    0, a_c3, 0, a_c4];
B = [0; 2*C_alphaF/m; 0; 2*l_F*C_alphaF/I_z];
H = 61;  % H * 0.1 secs horizon

alpha_value = 18;
beta_value = 25;
% alpha_value = 15;
% beta_value = 15;

u_max = 0.72;
du_max = 0.72;

episode_length = 60;

% a straight track
% [y_upperbounds, y_lowerbounds] = zero_curvature(yl, yu, H);
% a curve track
yl = -1; yu = 1;
delta_x = v * deltaT; track_lambda = 30;
[y_upperbounds, y_lowerbounds] = sine_curvature(yl, yu, episode_length+H, track_lambda, delta_x);

disp('Running lane-keeping with DM...');

for num_dm = 1:4
    switch num_dm
        case 1
            current_dm = @pro_rec_dm;
            fprintf("Use pro-recoverability DM.\n");
        case 2
            current_dm = @pro_dur_dm;
            fprintf("Use pro-durability DM.\n");
        case 3
            current_dm = @adaptive_dm;
            fprintf("Use adaptive DM.\n");
        case 4
            current_dm = @mini_dis_dm;
            fprintf("Use minimal-distance DM.\n");
    end
%%
states = zeros(4,episode_length);
controls = zeros(1,episode_length-1);
pe = zeros(1,episode_length-1);
optimal_solution_all = cell(0);
optimal_set_all=cell(0);
optimal_states_all=cell(0);
solve_time_all = cell(0);
states(:,1) = x_0;
%%
for ts = 1:episode_length-1

    current_state = states(:,ts);

    u = sdpvar(H-1,1);
    constraints = [];
    for k = 1:H-1
        constraints = [constraints; u(k) <= u_max; u(k)>=-u_max];
    end
    % constrain the u change rate
%     constraints = [constraints; u(1)<=0.5 ;u(1)>=-0.5];
    constraints = [constraints; u(1)<=du_max ;u(1)>=-du_max];
    for k = 1:H-2
        constraints = [constraints; u(k+1)-u(k)>=-du_max; u(k+1)-u(k)<=du_max];
    end

    % sys state
    x_n = sdpvar(4,H); % with H number of points, signal length is H-1

    constraints = [constraints; x_n(:,1) == current_state];
    % compute system constraints
    for i = 1:H-1
        x_dot = A*x_n(:,i)+B*u(i);
        constraints = [constraints; x_n(:,i+1) == x_n(:,i) + x_dot.*deltaT];
    end

    z_p1 = sdpvar(H,1);
    z_p2 = sdpvar(H,1);
    for i = 1:H
        [constr, z_p1(i)] = bool_mu(x_n(1,i)', 1, y_lowerbounds(ts+i-1)); %  x>= y_lowerbounds
        constraints = [constraints;constr];
        [constr, z_p2(i)] = bool_mu(x_n(1,i)', -1, -y_upperbounds(ts+i-1)); % x<= y_upperbounds
        constraints = [constraints;constr];
    end

    %  x>= y_lowerbounds and x<= y_upperbounds
    z_and = sdpvar(H,1);
    for j = 1:H
        [constr, z_and(j)] = bool_and([z_p1(j); z_p2(j)]);
        constraints = [constraints;constr];
    end
    [constr, z_and_g] = bool_globally(z_and, 0, 2);
    constraints = [constraints;constr];

    [constr, trec_value] = trec(z_and_g);
    constraints = [constraints;constr];

    [constr, tdur_value] = tdur(z_and_g);
    constraints = [constraints;constr];

    options = sdpsettings('debug', 0,...
        'solver', 'mosek', ...
        'verbose',0);
    objective = -tdur_value(1);
    [optimal_solution,optimal_set, optimal_states, num_pe, solve_times] = rescon(constraints, trec_value(1), objective, x_n, u, options, H, alpha_value, beta_value);

    if mod(ts,10) == 1
    disp(sprintf('Progress: %d/%d', ts, episode_length-1));
    end

%     rng(seed(num_episode));
    optimal_solution_all{ts} = optimal_solution;
    optimal_set_all{ts} = optimal_set;
    optimal_states_all{ts} = optimal_states;
    solve_time_all{ts} = solve_times;
    pe(ts) = num_pe;
    os_index = current_dm(optimal_solution,alpha_value,H-beta_value);
    selected_controls = optimal_set{os_index};
    controls(:,ts) = selected_controls(1);
    % system evolve
    state_dot = A*states(:,ts)+B*controls(:,ts);
    states(:,ts+1) = states(:,ts) + state_dot.*deltaT;
end
disp('Finished.')
%%
% states_all =[ states_all; states]; controls_all = [controls_all; controls];
bool_traj = states(1,:) >= y_lowerbounds(1:episode_length)-1e-8 & states(1,:) <= y_upperbounds(1:episode_length)+1e-8;
rec_dur_pair = compute_pair_over_traj(bool_traj, 18, 25);
% rec_dur_pair_all = [rec_dur_pair_all,rec_dur_pair];
% end
    switch num_dm
        case 1
            save('../results/lk_mpc_prorec.mat', 'rec_dur_pair', 'optimal_solution_all', 'optimal_set_all', 'optimal_states_all', 'pe', 'controls', 'states', 'solve_time_all');
        case 2
            save('../results/lk_mpc_produr.mat', 'rec_dur_pair', 'optimal_solution_all', 'optimal_set_all', 'optimal_states_all', 'pe', 'controls', 'states', 'solve_time_all');
        case 3
            save('../results/lk_mpc_adaptive.mat', 'rec_dur_pair', 'optimal_solution_all', 'optimal_set_all', 'optimal_states_all', 'pe', 'controls', 'states', 'solve_time_all');
        case 4
            save('../results/lk_mpc_mindis.mat', 'rec_dur_pair', 'optimal_solution_all', 'optimal_set_all', 'optimal_states_all', 'pe', 'controls', 'states', 'solve_time_all');
    end
end
