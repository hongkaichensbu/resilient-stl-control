%%
states_rec = load("../results/lk_mpc_prorec.mat","solve_time_all","states","rec_dur_pair");
states_dur = load("../results/lk_mpc_produr.mat","solve_time_all","states","rec_dur_pair");
states_adaptive = load("../results/lk_mpc_adaptive.mat","solve_time_all","states","rec_dur_pair");
states_mindis = load("../results/lk_mpc_mindis.mat","solve_time_all","states","rec_dur_pair");

fprintf("The (rec,dur) pairs of the DM are.....\n");
disp("The pro-recoverability DM: "+ num2str(states_rec.rec_dur_pair'*0.1));
disp("The pro-durability DM: "+ num2str(states_dur.rec_dur_pair'*0.1));
disp("The adaptive DM: "+ num2str(states_adaptive.rec_dur_pair'*0.1));
disp("The minimal-distance DM: "+ num2str(states_mindis.rec_dur_pair'*0.1));

aa = 0;
for i = 1:size(states_rec.solve_time_all,2)
    a = sum(states_rec.solve_time_all{i});
    aa = aa + a;
end
for i = 1:size(states_dur.solve_time_all,2)
    a = sum(states_dur.solve_time_all{i});
    aa = aa + a;
end
for i = 1:size(states_adaptive.solve_time_all,2)
    a = sum(states_dur.solve_time_all{i});
    aa = aa + a;
end
for i = 1:size(states_mindis.solve_time_all,2)
    a = sum(states_dur.solve_time_all{i});
    aa = aa + a;
end
avg_time = aa/(size(states_rec.solve_time_all,2)+size(states_dur.solve_time_all,2)+size(states_adaptive.solve_time_all,2)+size(states_mindis.solve_time_all,2));
fprintf("Average computation time for a control action: "+num2str(avg_time)+" secs.");

%%

xpos = -1.9;
f = figure;
set(gca,'position',[0.09 0.18 0.88 0.80],'box','on','linewidth',0.8);font_size = 11;
hold on
plot(states_rec.states(1,:),'-','LineWidth',1.5);
plot(states_dur.states(1,:),'--','LineWidth',1.5);
plot(states_adaptive.states(1,:),'-.','LineWidth',1.5);
plot(states_mindis.states(1,:)+0.15,':','LineWidth',1.5);
set(gca,'ColorOrderIndex',1)
fh = fill_between_y(1:H,y_lowerbounds(1:H),y_upperbounds(1:H),'no');
fh.FaceColor = "#B2B2B2";
text(2,3,'Lane','FontSize',16)
scatter(1,x_0(1),100,"pentagram",'filled','HandleVisibility','off');
legend({'pro-recoverability DM','pro-durability DM','adaptive DM','minimal-distance DM'},...
    'FontSize',font_size,'Location','northeast','NumColumns',2)
yl=ylabel('y-coordinate (m)','FontSize',16);
pos=get(yl,'Pos');set(yl,'Pos',[xpos pos(2) pos(3)])
xlabel('x-coordinate (m)','FontSize',16);
ylim([-12,8.5]);xlim([1,H-1])
% title('Location')
set(gca,'box','on','linewidth',1)

set(gcf,'PaperUnits','inches','PaperPosition',[0 0 7.4 2.5])
saveas(gcf, '../results/Figure_4.png');
close(gcf);
% print('lk_mpc','-dpng','-r300')