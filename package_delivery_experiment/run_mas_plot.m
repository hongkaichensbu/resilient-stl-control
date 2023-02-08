%%
clear
config_mas;
num_stations = 2;
[charg_left_bottoms, charg_right_tops] = charging_stations(num_stations);
num_regions = 2;
[surveil_left_bottoms, surveil_right_tops] = surveillance_spots(num_regions);
alpha_value = 25;
beta_value =20;
H = 60;  % horizon
p_lim = 10; % bounds for the battery power 
p_globally_a = 0;
p_globally_b = H;

surveil_a1 = 0; surveil_b1 = 30;
surveil_a2 = 30; surveil_b2 = 60;


load ../experiment_data/optimal_mas.mat

fprintf("Optimal Solutions for Figure 5 is:\n");
disp(optimal_solution);

[~,max_dur] = max(optimal_solution(:,2));
[~,max_rec] = max(optimal_solution(:,1));

for plot_idx = [max_dur, max_rec]

ctrl_actions = optimal_set{plot_idx};
states = optimal_states{plot_idx};
color_red = "#ff3333";color_blue = [43, 130, 250]/255;
color_yellow = "#FF9933"; color_green="#33ff99";
font_size = 15;
figure;
set(gca,'position',[0.09 0.1 0.88 0.88],'box','on','linewidth',0.8);
% tiledlayout(1,1,'TileSpacing','Compact','Padding','Compact');
% subplot(2,1,1); hold on
hold on
for cs = 1:num_stations
    if cs ==1
        charg_name = 'Charging Regions';
    else
        charg_name = 'no';
    end
    h_ch1 = fill_between_y([charg_left_bottoms(cs,1),charg_right_tops(cs,1)],...
        charg_left_bottoms(cs,2)*ones(1,2), charg_right_tops( cs,2)*ones(1,2), charg_name);
    h_ch1.FaceColor = color_yellow;
    alpha(h_ch1, .6);
end
for cs = 1:num_regions
    if cs ==1
        surveil_name = 'Delivery Regions';
    else
        surveil_name = 'no';
    end
    h_ch1 = fill_between_y([surveil_left_bottoms(cs,1),surveil_right_tops(cs,1)],...
        surveil_left_bottoms(cs,2)*ones(1,2), surveil_right_tops(cs,2)*ones(1,2),surveil_name);
    h_ch1.FaceColor = color_blue;
    alpha(h_ch1, .6);
end
plot(states(1,1:end),states(3,1:end),'-','LineWidth',2,'Color',color_red); 
plot(states(7,1:end),states(9,1:end),'-','LineWidth',2,'Color',color_green); 
scatter(states(1,1),states(3,1),120,"pentagram",'filled','HandleVisibility','off','MarkerEdgeColor',"#cc0000",'MarkerEdgeColor','none');
scatter(states(7,1),states(9,1),120,"pentagram",'filled','HandleVisibility','off','MarkerFaceColor',"#00cc66");
h = legend({'Delivery Regions','Charging Regions','$robot_1$ Trajectory','$robot_2$ Trajectory'},'Interpreter','latex','Location','south','Fontsize',font_size);
pos = get(h,'Position');
set(h,'Position',[pos(1)-0.05 pos(2) pos(3) pos(4)]);
text(charg_left_bottoms(1,1),charg_left_bottoms(1,2)+0.5,'$C_1$','Interpreter','latex','FontSize',font_size+1);
text(charg_left_bottoms(2,1),charg_left_bottoms(2,2)+0.5,'$C_2$','Interpreter','latex','FontSize',font_size+1);
text(surveil_left_bottoms(1,1),surveil_left_bottoms(1,2)+0.5,'$R_1$','Interpreter','latex','FontSize',font_size+1);
text(surveil_left_bottoms(2,1),surveil_left_bottoms(2,2)+0.5,'$R_2$','Interpreter','latex','FontSize',font_size+1);
text(states(1,1),states(3,1)+0.5,'$robot_1$','Interpreter','latex','FontSize',font_size+1);
text(states(7,1),states(9,1)+0.5,'$robot_2$','Interpreter','latex','FontSize',font_size+1);
ylabel('y-coordinate','FontSize',font_size+2);yticks([0:11])
xlabel('x-coordinate','FontSize',font_size+2);
xlim([-0.5,10.5]); ylim([-0.5,11.5])
% subplot(2,1,2); hold on
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 7.4 5])
if plot_idx == max_dur
    saveas(gcf, '../results/Figure_5a.png');
else
    saveas(gcf, '../results/Figure_5b.png');
end
close(gcf);
% print('mrs_traj_3','-dpng','-r300')
% print('mrs_traj_4','-dpng','-r300')
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = [states(1, :); states(3, :)];
x2 = [states(7, :); states(9, :)];
p1 = states(5,:);
p2 = states(11,:);
constraints = [];
e1 = binvar(1,H-1,'full'); % control of charging
e2 = binvar(1,H-1,'full'); % control of charging
% constrain the charging control action (charged when robot is in one of the charging stations)
% charging stations
% robot 1:
[constr_charg_agent1_region1, z_charg_agent1_region1] = bool_mu_region(x1', charg_left_bottoms(1,1), charg_right_tops(1,1),...
    charg_left_bottoms(1,2), charg_right_tops(1,2)); % 1st agent in 1st charging region
constraints = [constraints; constr_charg_agent1_region1];
[constr_charg_agent1_region2, z_charg_agent1_region2] = bool_mu_region(x1', charg_left_bottoms(2,1), charg_right_tops(2,1),...
    charg_left_bottoms(2,2), charg_right_tops(2,2)); % 1st agent in 2nd charging region
constraints = [constraints; constr_charg_agent1_region2];
z_charg_agent1 = binvar(1,H,'full');
for i = 1:H % 1st agent is charged in either charging region
    [constr_charg_agent1, z_charg_agent1(i)] = bool_or([z_charg_agent1_region1(i);z_charg_agent1_region2(i)]);
    constraints = [constraints; constr_charg_agent1];
end
constraints = [constraints; e1 == z_charg_agent1(1:end-1)];
% % robot 2:
[constr_charg_agent2_region1, z_charg_agent2_region1] = bool_mu_region(x2', charg_left_bottoms(1,1), charg_right_tops(1,1),...
    charg_left_bottoms(1,2), charg_right_tops(1,2)); % 2nd agent in 1st charging region
constraints = [constraints; constr_charg_agent2_region1];
[constr_charg_agent2_region2, z_charg_agent2_region2] = bool_mu_region(x2', charg_left_bottoms(2,1), charg_right_tops(2,1),...
    charg_left_bottoms(2,2), charg_right_tops(2,2)); % 2nd agent in 2nd charging region
constraints = [constraints; constr_charg_agent2_region2];
z_charg_agent2 = binvar(1,H,'full');
for i = 1:H % 2nd agent is charged in either charging region
    [constr_charg_agent2, z_charg_agent2(i)] = bool_or([z_charg_agent2_region1(i);z_charg_agent2_region2(i)]);
    constraints = [constraints; constr_charg_agent2];
end
constraints = [constraints; e2 == z_charg_agent2(1:end-1)];

% power requirement: battery power >= p_lim
[constr_p1, z_p1] = bool_mu(p1', 1, p_lim);
constraints = [constraints;constr_p1];
[constr_p2, z_p2] = bool_mu(p2', 1, p_lim);
constraints = [constraints;constr_p2];
% [constr_p1_g, z_p1_g] = bool_globally(z_p1, p_globally_a, p_globally_b);
% constraints = [constraints;constr_p1_g];
% [constr_p2_g, z_p2_g] = bool_globally(z_p2, p_globally_a, p_globally_b);
% constraints = [constraints;constr_p2_g];
z_p = sdpvar(H,1,'full');
for i =1:H
    [constr_p, z_p(i)] = bool_and([z_p1(i); z_p2(i)]);
    constraints = [constraints;constr_p];
end

% surveillance requirement: every surveillance region needs to be entered
% by at least one robot every 30 steps.
% region 1:
[constr_surveil_agent1_region1, z_surveil_agent1_region1] = bool_mu_region(x1', surveil_left_bottoms(1,1), surveil_right_tops(1,1),...
    surveil_left_bottoms(1,2), surveil_right_tops(1,2)); % 1st agent in 1st charging region
constraints = [constraints; constr_surveil_agent1_region1];
[constr_surveil_agent2_region1, z_surveil_agent2_region1] = bool_mu_region(x2', surveil_left_bottoms(1,1), surveil_right_tops(1,1),...
    surveil_left_bottoms(1,2), surveil_right_tops(1,2)); % 2nd agent in 1st charging region
constraints = [constraints; constr_surveil_agent2_region1];
z_surveil_region1 = binvar(H,1,'full');
for i = 1:H % region 1 is surveillentd by either robot
    [constr_surveil_region1, z_surveil_region1(i)] = bool_or([z_surveil_agent1_region1(i);z_surveil_agent2_region1(i)]);
    constraints = [constraints; constr_surveil_region1];
end
[constr_suveil_region1_finally1, z_suveil_region1_finally1] = bool_finally(z_surveil_region1, surveil_a1, surveil_b1);
constraints = [constraints; constr_suveil_region1_finally1]; % region 1 is visited the 1st time 
[constr_suveil_region1_finally2, z_suveil_region1_finally2] = bool_finally(z_surveil_region1, surveil_a2, surveil_b2);
constraints = [constraints; constr_suveil_region1_finally2]; % region 1 is visited the 2nd time 
z_suveil_region1_finally_and = sdpvar(H,1,'full');
for i = 1:H % region 1 is visited twice
    [constr_surveil_region1_finally_and, z_suveil_region1_finally_and(i)] = bool_and([z_suveil_region1_finally1(i);z_suveil_region1_finally2(i)]);
    constraints = [constraints; constr_surveil_region1_finally_and];
end
% region 2:
[constr_surveil_agent1_region2, z_surveil_agent1_region2] = bool_mu_region(x1', surveil_left_bottoms(2,1), surveil_right_tops(2,1),...
    surveil_left_bottoms(2,2), surveil_right_tops(2,2)); % 1st agent in 2nd charging region
constraints = [constraints; constr_surveil_agent1_region2];
[constr_surveil_agent2_region2, z_surveil_agent2_region2] = bool_mu_region(x2', surveil_left_bottoms(2,1), surveil_right_tops(2,1),...
    surveil_left_bottoms(2,2), surveil_right_tops(2,2)); % 2nd agent in 1st charging region
constraints = [constraints; constr_surveil_agent2_region2];
z_surveil_region2 = binvar(H,1,'full');
for i = 1:H % region 2 is surveillentd by either robot
    [constr_surveil_region2, z_surveil_region2(i)] = bool_or([z_surveil_agent1_region2(i);z_surveil_agent2_region2(i)]);
    constraints = [constraints; constr_surveil_region2];
end
[constr_suveil_region2_finally1, z_suveil_region2_finally1] = bool_finally(z_surveil_region2, surveil_a1, surveil_b1);
constraints = [constraints; constr_suveil_region2_finally1]; % region 2 is visited the 1st time 
[constr_suveil_region2_finally2, z_suveil_region2_finally2] = bool_finally(z_surveil_region2, surveil_a2, surveil_b2);
constraints = [constraints; constr_suveil_region2_finally2]; % region 2 is visited the 2nd time
z_suveil_region2_finally_and = sdpvar(H,1,'full');
for i = 1:H % region 2 is visited twice
    [constr_surveil_region2_finally_and, z_suveil_region2_finally_and(i)] = bool_and([z_suveil_region2_finally1(i);z_suveil_region2_finally2(i)]);
    constraints = [constraints; constr_surveil_region2_finally_and];
end
% region 1 AND 2:
z_suveil = sdpvar(H,1,'full');
for i = 1:H % Both regions 1 and 2 are visited twice
    [constr_surveil, z_suveil(i)] = bool_and([z_suveil_region1_finally_and(i);z_suveil_region2_finally_and(i)]);
    constraints = [constraints; constr_surveil];
end
% power requirement AND surveillance requirement
z_p_and_surveil = sdpvar(H,1,'full');
for i =1:H
    [constr_p_and_surveil, z_p_and_surveil(i)] = bool_and([z_p(i); z_suveil(i)]);
    constraints = [constraints;constr_p_and_surveil];
end
[constr_trec, trec_value] = trec(z_p_and_surveil);
constraints = [constraints;constr_trec];
[constr_tdur, tdur_value] = tdur(z_p_and_surveil);
constraints = [constraints;constr_tdur];
objective = -tdur_value(1);
options = sdpsettings('debug', 0,...
'solver', 'mosek', ...
    'verbose',0);
optimize(constraints, objective, options);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ls1 = '-';
ls2 = ':';
xaxis_label = 'Time'; font_size =9.5; yl_pos = -4;
figure; 
% subplot(2,3,1);
t = tiledlayout(2,3,'TileSpacing','compact','Padding','tight');
nexttile
hold on;plot_binary(z_charg_agent1_region1, color_red,ls1);
plot_binary(z_charg_agent1_region2, color_blue,ls2); 
% title(['\it robot1',' charging'])
legend({'$robot_1\in C_1$','$robot_1\in C_2$'},'Interpreter','latex','NumColumns',2,'Location','northoutside','FontSize',font_size)
xlabel(xaxis_label); 
% yl=ylabel('$z^{\varphi_{c1}}$','Interpreter','latex','Rotation',0,'FontSize',font_size);
% yl.Position(1)=yl_pos;
yticks([0,1]);ylim([-0.1,1.1])
% subplot(2,3,2);
nexttile
hold on;
plot_binary(z_surveil_agent1_region1, color_red,ls1);
plot_binary(z_surveil_agent2_region1, color_blue,ls2); 
% title(['\it R1', ' surveillance'])
legend({'$robot_1\in R_1$','$robot_2\in R_1$'},'Interpreter','latex','NumColumns',2,'Location','northoutside','FontSize',font_size)
xlabel(xaxis_label); 
% yl=ylabel('$z^{\varphi_{s1}}$','Interpreter','latex','Rotation',0,'FontSize',font_size);
% yl.Position(1)=yl_pos;
yticks([0,1]);ylim([-0.1,1.1])
% subplot(2,3,3);
nexttile
hold on;
plot_binary(z_p1, color_red,ls1);
plot_binary(z_p2, color_blue,ls2); 
% title('Battery power')
legend({'$e^1_t\geq E_l$','$e^2_t\geq E_l$'},'Interpreter','latex','NumColumns',2,'Location','northoutside','FontSize',font_size)
% xlabel(xaxis_label); yl=ylabel('$z^{\varphi_{e1}}$','Interpreter','latex','Rotation',0,'FontSize',font_size);
% yl.Position(1)=yl_pos;
yticks([0,1]);ylim([-0.1,1.1]);xlabel(xaxis_label); 
% subplot(2,3,4);
nexttile
hold on;
plot_binary(z_charg_agent2_region1, color_red,ls1);
plot_binary(z_charg_agent2_region2, color_blue,ls2); 
% title(['\it robot2',' charging'])
legend({'$robot_2\in C_1$','$robot_2\in C_2$'},'Interpreter','latex','NumColumns',2,'Location','northoutside','FontSize',font_size)
yticks([0,1]);ylim([-0.1,1.1]);xlabel(xaxis_label); 
% subplot(2,3,5);
nexttile
hold on;
plot_binary(z_surveil_agent1_region2, color_red,ls1);
plot_binary(z_surveil_agent2_region2, color_blue,ls2); 
% title('\textit{R2} surveillance $\varphi_{s2}$' ,'Interpreter','latex')
% legend({'\it robot1','\it robot2'})
% yl=ylabel('$z^{\varphi_{sur}}$','Interpreter','latex','Rotation',0,'FontSize',font_size);
% yl.Position(1)=yl_pos;
legend({'$robot_1\in R_2$','$robot_2\in R_2$'},'Interpreter','latex','NumColumns',2,'Location','northoutside','FontSize',font_size)
xlabel(xaxis_label); 
yticks([0,1]);ylim([-0.1,1.1])
% subplot(2,3,6);
nexttile
hold on;
plot_binary(z_p_and_surveil, color_red,ls1); 
% title('\textbf{Problem requirement} $\varphi_{sur}$','Interpreter','latex')
xlabel(xaxis_label); 
legend({'$(x,t)\models\varphi_{sur}$'},'Interpreter','latex','NumColumns',2,'Location','northoutside','FontSize',font_size)
% yl=ylabel('$z^{\varphi_{sur}}$','Interpreter','latex','Rotation',0,'FontSize',font_size);
% yl.Position(1)=yl_pos;
yticks([0,1]);ylim([-0.1,1.1])
xticks([0,20,40,60])

set(gcf,'PaperUnits','inches','PaperPosition',[0 0 7.4 4])
if plot_idx == max_dur
    saveas(gcf, '../results/Figure_6a.png');
else
    saveas(gcf, '../results/Figure_6b.png');
end
close(gcf);
% print('mrs_details_3','-dpng','-r300')
% print('mrs_details_4','-dpng','-r300')
end