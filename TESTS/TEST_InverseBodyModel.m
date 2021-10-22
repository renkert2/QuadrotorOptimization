%%
qr = QR_InitialDesign();
bm = qr.BM;
makeSimulinkModel(bm, 'BodyModel_Simulink_OpenLoop')

%% init
% rt = ReferenceTrajectory3D();
% rt.Sigmoid('FinalPosition', [1 2 3]);
% rt.Speed = 1;
% rt.Cycles = 1;
% rt.delta_S = 0.01;
% rt.init();
% rt.setTimeSeries();
% 
% rt = ReferenceTrajectory3D();
% rt.Lemniscate()
% rt.Speed = 1;
% rt.Cycles = 1;
% rt.delta_S = 0.01;
% rt.init();
% rt.setTimeSeries();

rt = ReferenceTrajectory3D();
rt.Sigmoid('FinalPosition', [5 10 15], 'MaxVelocity', 10, 'Offset', 10);
rt.Speed = 1;
rt.Cycles = 1;
rt.delta_S = 0.01;
rt.init();
rt.setTimeSeries();

t = rt.t;
set_param('BodyModel_Simulink_OpenLoop', 'StopTime', num2str(t(end)))
p_des = rt.R_Data;
a_des = rt.ddR_Data;

v0_des = rt.dR_Data(:,1);
a0_des = a_des(:,1);
p0_des = p_des(:,1);

bm.x0(bm.I.x.p) = p0_des;
bm.x0(bm.I.x.v) = v0_des;

%% Test
vel_data_i = v0_des + cumtrapz(t, a_des, 2);
pos_data_i = cumtrapz(t, vel_data_i, 2);

plot(t,p_des(:,:))
hold on
plot(t,pos_data_i(:,:))
hold off

%%
[omega_tilde_des, Theta_des, b_omega_des] = bm.InverseBodyModel(a_des, rt.delta_T);
bm.x0(bm.I.x.Theta) = Theta_des(:,1);
bm.x0(bm.I.x.omega) = b_omega_des(:,1);

omega_tilde_ts = timeseries(omega_tilde_des, rt.t);

%% Forward Simulate to Check Accuracy
simOut = sim('BodyModel_Simulink_OpenLoop');
t_sim = simOut.tout';
p_sim = get(simOut.yout, 'p_out').Values.Data';

%% Compare Inverse Body Model signals to Closed-Loop Signals
bms = BodyModelSystem(bm);
bms.RefTraj = rt;
bms.init()
%%
[t,y,r,u] = bms.Simulate();

%%  Make Plots
% 
% figure(1)
% plot(t,u', '-b', 'DisplayName', "Feedback")
% hold on
% plot(rt.t, omega_tilde_des, '-r', 'DisplayName', "Calculated")
% hold off
% title("Input Comparison")
% legend
% 
% figure(2)
% plot(t,y(:, bm.I.x.Theta)', '-b', 'DisplayName', "Feedback")
% hold on
% plot(rt.t, Theta_des, '-r', 'DisplayName', "Calculated")
% hold off
% title("Input Comparison")
% legend

figure(1)
tl = tiledlayout(3,1);
labels = ["$$x$$", "$$y$$", "$$z$$"];
for i = 1:3
   nexttile(i);
   plot(t, p_des(i,:),'-b');
   hold on
   plot(t_sim, p_sim(i,:), '-r');
   hold off
   title(sprintf("Position: %s", labels(i)), 'Interpreter', 'latex')
   ylabel(sprintf("%s (m)", labels(i)));
end
xlabel("t (s)")
title(tl,"Desired vs. Actual Position", 'FontName', 'Times')
lg = legend(["$$r_{des}(t)$$","$$r(t) = BM(BM^{-1}(r_{des}(t)))$$"], 'Interpreter','latex');
lg.Orientation = 'horizontal';
lg.Location = 'southoutside';

figure(2)
p_sim_interp = interp1(t_sim, p_sim', t)';
plot(t, vecnorm(p_des - p_sim_interp, 2),'-r');
title("Position Error", 'FontName', 'Times')
ylabel("$$e$$ (m)", 'Interpreter', 'latex');
xlabel("t (s)")
lg = legend("$$e(t) = ||p_{des}(t) - p(t)||$$", 'Interpreter','latex');
lg.Orientation = 'horizontal';
lg.Location = 'southoutside';



