%%
qr = QR_InitialDesign();
o = Optimization(qr);
bm = qr.BM;
makeSimulinkModel(bm, 'BodyModel_Simulink_OpenLoop')

% Initialize Controller
pt = qr.PT;
ptc = PowerTrain_OptiController(pt);

%% init
rt = ReferenceTrajectory3D();
max_speed = 11;
rt.Sigmoid('FinalPosition', [5 10 15], 'MaxVelocity', max_speed, 'Offset', 10);
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

% Plot Reference Trajectory
plot(rt.TimeSeries);
title(sprintf("Reference Trajectory: $$v_{max} = %0.1f$$ (m/s)", max_speed), 'Interpreter', 'latex')
ylabel("Position (m)")
legend(["$$x$$", "$$y$$", "$$z$$"], 'Interpreter', 'latex')

%%

t = rt.t;
set_param('BodyModel_Simulink_OpenLoop', 'StopTime', num2str(t(end)))
ptc.formulate(numel(rt.t), rt.delta_T, 'SolverOpts', {'solver','quadprog','verbose',2});
%% Evaluate Performance
o.resetQR();
out_struct = Evaluate(qr,bm,ptc,rt);
out_struct.Desc = "Initial Design";
WeightStudyStruct(1)=out_struct;

% Set to Smallest Feasible Frame Mass and Evaluate Performance
qr.Frame.Mass.Value = 0;
o.updateQR(false);

out_struct = Evaluate(qr,bm,ptc,rt);
out_struct.Desc = "Small Mass";
WeightStudyStruct(2)=out_struct;

% Set to Largest Feasible Frame Mass and Evaluate Performance
qr.Frame.Mass.Value = 1;
o.updateQR(false);

out_struct = Evaluate(qr,bm,ptc,rt);
out_struct.Desc = "Large Mass";
WeightStudyStruct(3)=out_struct;

%save('WeightStudy_DynamicOptimization_Output.mat', 'WeightStudyStruct')

%% Plot Input Solution
for i = 1:3
    plot(t(1:end-1), mean(WeightStudyStruct(i).sol_u,1), 'DisplayName', WeightStudyStruct(i).Desc)
    hold on
end
hold off
title("Optimal Input (Average)")
ylabel("Input")
xlabel("Time (s)")
legend

%% Plot Trajectory
bm.plot(x_out')

%% Position Error
figure(1)
tl = tiledlayout(3,1);
labels = ["$$x$$", "$$y$$", "$$z$$"];
j_labels = ["Initial Design", "Small Mass", "Large Mass"];
for i = 1:3
   nexttile(i);
   plot(t, WeightStudyStruct(1).p_des(i, :),'-b', 'DisplayName', "$$r_{des}(t)$$");
   hold on
   for j = 1:3
       s = WeightStudyStruct(j);
       plot(s.t_sim, s.p_sim(i,:), 'DisplayName', j_labels(j));
   end
   hold off
   title(sprintf("Position: %s", labels(i)), 'Interpreter', 'latex')
   ylabel(sprintf("%s (m)", labels(i)));
end
xlabel("t (s)")
title(tl,"Desired vs. Actual Position", 'FontName', 'Times')
lg = legend('Interpreter', 'latex');
lg.Orientation = 'horizontal';
lg.Location = 'southoutside';
%%
figure(2)
p_sim_interp = interp1(t_sim, p_sim', t)';
plot(t, vecnorm(p_des - p_sim_interp, 2),'-r');
title("Position Error", 'FontName', 'Times')
ylabel("$$e$$ (m)", 'Interpreter', 'latex');
xlabel("t (s)")
lg = legend("$$e(t) = ||p_{des}(t) - p(t)||$$", 'Interpreter','latex');
lg.Orientation = 'horizontal';
lg.Location = 'southoutside';

function out_struct = Evaluate(qr,bm,ptc,rt)
    [~,omega_tilde_des, x_des, x_des_0] = bm.InverseBodyModel(rt);
    Theta_des = x_des(bm.I.x.Theta, :);
    b_omega_des = x_des(bm.I.x.omega, :);

    %% Calculate Optimal u, omega w.r.t. Input Constraints

    % 
    % [sol_u, sol_x, sol_omega_tilde, diagnostic] = OptiControl(pt,...
    %     numel(rt.t), rt.delta_T, omega_tilde_des, qr.SS_QAve);

    [sol_x, sol_u, sol_y, errorcode] = Solve(ptc, qr.SS_QAve, omega_tilde_des);
    
    out_struct.omega_tilde_des = omega_tilde_des;
    out_struct.x_des = x_des;
    out_struct.p_des = x_des(bm.I.x.p, :);
    out_struct.x_des_0 = x_des_0;
    out_struct.Theta_des = Theta_des;
    out_struct.b_omega_des = b_omega_des;
    out_struct.sol_x = sol_x;
    out_struct.sol_u = sol_u;
    out_struct.sol_y = sol_y;
    
    %% Create Time Series from Optimized Trajectory
    omega_tilde_ts = timeseries(sol_y, rt.t);
    mdlWks = get_param('BodyModel_Simulink_OpenLoop','ModelWorkspace');
    assignin(mdlWks, 'omega_tilde_ts', omega_tilde_ts);

    %% Forward Simulate to Check Accuracy
    simOut = sim('BodyModel_Simulink_OpenLoop');
    t_sim = simOut.tout';
    p_sim = get(simOut.yout, 'p_out').Values.Data';
    x_sim = get(simOut.yout, 'y_out').Values.Data';
    
    out_struct.t_sim = t_sim;
    out_struct.p_sim = p_sim;
    out_struct.x_sim = x_sim;
end




