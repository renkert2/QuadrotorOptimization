%%
qr = QR_InitialDesign();
bm = qr.BM;
makeSimulinkModel(bm, 'BodyModel_Simulink_OpenLoop')

%% init
rt = ReferenceTrajectory3D();
rt.Sigmoid('FinalPosition', [5 10 15], 'MaxVelocity', 10, 'Offset', 10);
rt.Speed = 1;
rt.Cycles = 1;
rt.delta_S = 0.01;
rt.init();
rt.setTimeSeries();

% rt = ReferenceTrajectory3D();
% rt.Lemniscate()
% rt.Speed = 1;
% rt.Cycles = 1;
% rt.delta_S = 0.01;
% rt.init();
% rt.setTimeSeries();

t = rt.t;
set_param('BodyModel_Simulink_OpenLoop', 'StopTime', num2str(t(end)))
%%
[~,omega_tilde_des, x_des, x_des_0] = bm.InverseBodyModel(rt);
Theta_des = x_des(bm.I.x.Theta, :);
b_omega_des = x_des(bm.I.x.omega, :);

%% Calculate Optimal u, omega w.r.t. Input Constraints
pt = qr.PT;

[sol_u, sol_x, sol_omega_tilde, diagnostic] = OptiControl(pt,...
    numel(rt.t), rt.delta_T, omega_tilde_des, qr.SS_QAve);

%% Rotor Speed Error
figure(1)
tl = tiledlayout(4,1);
for i = 1:4
   nexttile(i);
   plot(t, omega_tilde_des(i,:),'-b');
   hold on
   plot(t, sol_omega_tilde(i,:), '-r');
   hold off
   title(sprintf("Rotor %d", i))
   ylabel("$$\omega$$ (rad/s)")
end
xlabel("t (s)")
title(tl,"Rotor Speeds", 'FontName', 'Times')
lg = legend(["$$\omega_{des}(t)$$","$$\omega(t)$$"], 'Interpreter','latex');
lg.Orientation = 'horizontal';
lg.Location = 'southoutside';

%% Inputs

figure(2)
plot(t(1:end-1),sol_u)
title("Input")
xlabel("t (s)")
ylabel("T (N)")
lgnd_names = compose("$$u_%d^*(t)$$", (1:4)');
legend(lgnd_names, 'Interpreter','latex')

%% Battery SOC
figure(3)
q = value(sol_x(1,:));
plot(t, q)
title("Battery SOC")
xlabel("t (s)")
ylabel("$$q$$")

%% Create Time Series from Optimized Trajectory
omega_tilde_ts = timeseries(sol_omega_tilde, rt.t);

%% Forward Simulate to Check Accuracy
simOut = sim('BodyModel_Simulink_OpenLoop');
t_sim = simOut.tout';
p_sim = get(simOut.yout, 'p_out').Values.Data';
x_out = get(simOut.yout, 'y_out').Values.Data';

%% Plot Trajectory
bm.plot(x_out')

%% Position Error
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




