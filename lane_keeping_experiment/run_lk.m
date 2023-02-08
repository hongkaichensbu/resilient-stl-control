% This file run an experiment in which a vechile is controlled by the
% STL-based resilient controller in a lane-keeping problem.
% The controller computes multiple optimal solutions at this moment.

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

yl = -1;
yu = 1;

track_lambda = 30;
%%
u = sdpvar(H-1,1);
constraints = [];
for k = 1:H-1
    constraints = [constraints; u(k) <= u_max; u(k)>=-u_max];
end
% constrain the u change rate
constraints = [constraints; u(1)<=0.5 ;u(1)>=-0.5];
for k = 1:H-2
    constraints = [constraints; u(k+1)-u(k)>=-du_max; u(k+1)-u(k)<=du_max];
end

% sys state
x_n = sdpvar(4,H); % with H number of points, signal length is H-1

constraints = [constraints; x_n(:,1) == x_0];
% compute system constraints
for i = 1:H-1
    x_dot = A*x_n(:,i)+B*u(i);
    constraints = [constraints; x_n(:,i+1) == x_n(:,i) + x_dot.*deltaT];
end


% a straight track
% [y_upperbounds, y_lowerbounds] = zero_curvature(yl, yu, H);
% a curve track
delta_x = v * deltaT;
[y_upperbounds, y_lowerbounds] = sine_curvature(yl, yu, H, track_lambda, delta_x);

z_p1 = sdpvar(H,1);
z_p2 = sdpvar(H,1);
for i = 1:H
[constr, z_p1(i)] = bool_mu(x_n(1,i)', 1, y_lowerbounds(i)); %  x>= y_lowerbounds
constraints = [constraints;constr];
[constr, z_p2(i)] = bool_mu(x_n(1,i)', -1, -y_upperbounds(i)); % x<= y_upperbounds
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
[optimal_solution,optimal_set, optimal_states] = rescon(constraints, trec_value(1), objective, x_n, u, options, H, alpha_value, beta_value);

fprintf("Optimal Solutions for Figure 3 is:\n");
disp(optimal_solution*deltaT);

%%
% color = []
xpos = -1.9;
figure;
set(gca,'position',[0.5 0.19 0.80 0.78],'box','on','linewidth',0.8);
t = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
nexttile
% subplot(3,1,1); 
hold on
states_tmp = optimal_states{1};
plot(states_tmp(1,:),'-','LineWidth',1.5);
states_tmp = optimal_states{2};
plot(states_tmp(1,:),'--','LineWidth',1.5);
states_tmp = optimal_states{3};
plot(states_tmp(1,:),'-.','LineWidth',1.5);
set(gca,'ColorOrderIndex',1)
fh = fill_between_y(1:H,y_lowerbounds,y_upperbounds,'no');
fh.FaceColor = "#B2B2B2";
text(1.5,-2,'Lane')
scatter(1,x_0(1),100,"pentagram",'filled');
legend({'trajectory 1','trajectory 2','trajectory 3'},'NumColumns',3,'Location','northoutside','FontSize',12)
yl=ylabel('y-coordinate (m)','FontSize',13);
pos=get(yl,'Pos');set(yl,'Pos',[xpos pos(2) pos(3)])
xlabel('x-coordinate (m)','FontSize',14);
ylim([-7.5,9]);xlim([1,H-1]);xticks([1:5:H-1,60]);xticklabels([0:5:H-1,60]);
title('Locations','FontSize',10);
set(gca,'box','on','linewidth',1)

% subplot(3,1,2); 
nexttile; hold on
control_tmp = optimal_set{1};plot(control_tmp','-','LineWidth',1.5);
control_tmp = optimal_set{2};plot(control_tmp','--','LineWidth',1.5);
control_tmp = optimal_set{3};plot(control_tmp','-.','LineWidth',1.5);
umax = yline(u_max,'k','LineWidth',1,'LineStyle','--'); umax.Alpha = 0.5;
umin = yline(-u_max,'k','LineWidth',1,'LineStyle','--');umin.Alpha = 0.5;
% legend('trajectory 1','trajectory 2','trajectory 3')
xlabel('Time (sec)','FontSize',14);yl=ylabel('Steering angel (rad)','FontSize',13);
pos=get(yl,'Pos');set(yl,'Pos',[xpos pos(2) pos(3)])
yticks([-0.72, 0, 0.72]);xticks([1:10:H]);
xticklabels([0:10:H]*deltaT);xlim([1,H-1]);xticks([1:10:H-1,60]);xticklabels([0:10:H-1,60]*deltaT);
title('Control Actions','FontSize',10);
set(gca,'box','on','linewidth',1)

% subplot(3,1,3); 
nexttile; hold on
states_tmp = optimal_states{1}; bool1 = states_tmp(1,:) >= y_lowerbounds-1e-8 & states_tmp(1,:) <= y_upperbounds+1e-8;
% plot(bool1,'-','LineWidth',1.5);
stairs(bool1,'-','LineWidth',1.5);
states_tmp = optimal_states{2}; bool1 = states_tmp(1,:) >= y_lowerbounds-1e-8 & states_tmp(1,:) <= y_upperbounds+1e-8;
% plot(bool1,'--','LineWidth',1.5);
stairs(bool1,'--','LineWidth',1.5);
states_tmp = optimal_states{3}; bool1 = states_tmp(1,:) >= y_lowerbounds-1e-8 & states_tmp(1,:) <= y_upperbounds+1e-8;
% plot(bool1,'-.','LineWidth',1.5);
stairs(bool1,'-.','LineWidth',1.5);
% legend('trajectory 1','trajectory 2','trajectory 3')
xlabel('Time (sec)','FontSize',14);yl=ylabel('$z^{\varphi_{lk}}$','Interpreter','latex','Rotation',0,'FontSize',17);
pos=get(yl,'Pos');set(yl,'Pos',[xpos 0.3 pos(3)])
title('Lane-keeping requirement','FontSize',10);
ylim([-0.1,1.1]);xlim([1,H-1]);yticks([0,1]);xticks([1:10:H-1,60]);xticklabels([0:10:H-1,60]*deltaT);
set(gca,'box','on','linewidth',1)

set(gcf,'PaperUnits','inches','PaperPosition',[0 0 7.4 6])
saveas(gcf, '../results/Figure_3.png');
close(gcf);
% print('results/lane_keeping_trajectory','-dpng','-r300')