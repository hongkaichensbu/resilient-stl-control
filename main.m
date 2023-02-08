function main()


%--------------------------------------------------------------------------
% Below are the example sections
fprintf("----------------------------------\n");
% Plot Figure 3 in the paper which are the optimal solution visulization of the 
% lane-keeping solutions (-0.2,1.5), (0.1, -2.2), (0.2, -2.3).
fprintf("Run experiment for Figure 3.\n");
fprintf("Plot and save Figure 3 to 'Figure_3.png'.\n");
run_lk;

fprintf("\n");
fprintf("----------------------------------\n");
% Plot Figure 4 in the paper which are the visulization of the 
% Decision Maker trajectories (0.3,-2.1), (-0.2,1.5), (0,-1.9), (-0.2,1.5).
fprintf("Run experiment for Figure 4.\n");
run_lk_mpc;
fprintf("Plot and save Figure 4 to 'Figure_4.png'.\n");
analys_mpc_results;

fprintf("\n");
fprintf("----------------------------------\n");
% Plot Figure 5 in the paper which are the optimal solution visulization of the 
% delivery mission (-4,4),(9,-5),(-3,1).
fprintf("Run experiment for Figures 5 and 6.\n");
run_mas; % uncomment on a powerful machine and use gurobi instead of mosek
fprintf("Plot and save Figure 5 to 'Figure_5a.png' and 'Figure_5b.png'.\n");
fprintf("Plot and save Figure 6 to 'Figure_6a.png' and 'Figure_6b.png'.\n");
run_mas_plot;

fprintf("----------------------------------\n");
fprintf("Run experiment for deadline-driven package delivery in MPC setting.\n");
% run_mas_mpc;  % uncomment on a powerful machine and use gurobi instead of mosek
run_mas_mpc_plot;

end