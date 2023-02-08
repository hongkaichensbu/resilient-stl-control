% This file uses epsilon-constraint methods to solve a MO-MILP problem
% optimal_solution: all the Pareto-optimal solutions
% optimal_set: decision variable for the optimal solutions

% Using YALMIP, we implement the strategy using an optimizer object
function [optimal_solution,optimal_set,optimal_states, num_pe, solve_times] = rescon(constraints, trec_value_0, objective, states, u, options, H, alpha_value, beta_value)


optimal_solution = [];
optimal_set = cell(0);
optimal_states = cell(0);
solve_times = [];

epsilon = alpha_value - (H-1);
jjj=1;
while epsilon < alpha_value + 1 
    % fprintf("iteration"+num2str(jjj));jjj=jjj+1;
    eps_constr = [constraints; alpha_value - trec_value_0>=epsilon];
    sol = optimize(eps_constr, objective, options);
    if sol.problem ~= 0
        break;
    else
        solve_times = [solve_times, sol.solvertime];
        recoverability = alpha_value - value(trec_value_0);
        durability = -value(objective) - beta_value;
        optimal_solution = [optimal_solution;recoverability,durability];
%         optimal_solution = [optimal_solution;value(trec_value_0),-value(objective)];
        ctrl = value(u);
        optimal_set = [optimal_set; ctrl];
        sys_states = value(states);
        optimal_states = [optimal_states; sys_states];
        epsilon = recoverability+1;%break;
    end
end
num_pe = size(optimal_states,1);
[optimal_solution, optimal_index] = max_res_set(optimal_solution);
optimal_set = optimal_set(optimal_index,:);
optimal_states = optimal_states(optimal_index);
end

