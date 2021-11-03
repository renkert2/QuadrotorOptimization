%%
qr = QR_InitialDesign();
o = Optimization(qr);
bm = qr.BM;
pt = qr.PT;

%% init
rt = ReferenceTrajectory3D();
max_speed = 50;
rt.Sigmoid('FinalPosition', [5 10 15], 'MaxVelocity', max_speed, 'Offset', 2);
rt.Speed = 1;
rt.Cycles = 1;
rt.delta_S = 0.01;
rt.init();
rt.setTimeSeries();

rt_des = rt.R_Data;

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

%% Plant

% PowerTrain Model
% Use full dynamics without simplifications.  Could also use simplified
% version with Battery SOC
x_ss_pt = simple2full(qr.SS_QAve);
u_ss_pt = qr.SS_QAve.u;
[A_pt,B_pt,C_pt,D_pt] = CalcMatrices(pt, qr.SS_QAve);

% Body Model
x_ss_b = zeros(12,1);
u_ss_b = repmat(qr.SS_QAve.RotorSpeed, 4, 1);
d_ss_b = zeros(3,1);
[A_b,B_b,~,C_b,D_b,~] = CalcMatrices(bm.LinearModel,x_ss_b,u_ss_b,d_ss_b);

% Combined System
A = [A_pt, zeros(size(A_pt,1),size(A_b,2)); B_b*C_pt, A_b];
B = [B_pt; B_b*D_pt];

% We only want position outputs
N_x = size(A_pt,1)+size(A_b,1);
I_x_p = size(A_pt,1) + bm.I.x.p;
N_y = numel(I_x_p);
C = zeros(N_y, N_x);
for j = 1:N_y
    C(j,I_x_p(j)) = 1;
end

% Discretize
[A_d, B_d] = LinearModel.Discretize(A,B,[],[],rt.delta_T);

% Steady State
u_ss = u_ss_pt;
x_ss = [x_ss_pt; x_ss_b];
x_ss(1) = 0; % Dont modify battery SOC since it's steady state doesn't exist

% Initial Condition
x0 = [x_ss_pt; x_ss_b];
x0(1) = 1; % Set initial state of charge to 1



%% Formulate Optimization (YALMIP)
N_x = pt.Model.Nx + bm.Nx;
N_u = pt.Model.Nu;
N_y = numel(bm.I.x.p);

N = numel(rt.t);
delta_x_ = sdpvar(N_x,N,'full');
delta_u_ = sdpvar(N_u,N-1); 

objs = 0;
for k = 2:N
    objs = objs + (1/2)*(C*(delta_x_(:,k)+x_ss) - rt_des(:,k))'*(C*(delta_x_(:,k)+x_ss) - rt_des(:,k));
end

%Penalty constraint to smooth things out
% du_penalty = 0.01;
% for k = 2:N-1
%     objs = objs + du_penalty*(delta_u_(:,k) - delta_u_(:,k-1));
% end

% Initial Condition
cons = [delta_x_(:,1) + x_ss == x0];
for k = 1:N-1
    % Dynamic Constraints
    cons = [cons, delta_x_(:,k+1) == A_d*delta_x_(:,k) + B_d*delta_u_(:,k)];

    % Input Constraints
    cons = [cons, zeros(N_u,1) <= delta_u_(:,k) + u_ss <= ones(N_u,1)];
end

du_limit = 0.1;
for k = 2:N-1
    cons = [cons, (delta_u_(:,k) - delta_u_(:,k-1))<=du_limit, (delta_u_(:,k) - delta_u_(:,k-1))>=-du_limit];
end

% Final Condition
cons = [cons, delta_x_(1,end)>=0.2];

opts = sdpsettings('solver','quadprog');

tic;
diagnostics = optimize(cons,objs,opts);
solve_time = toc;

sol_u = value(delta_u_) + u_ss;
sol_x = (value(delta_x_) + x_ss);
sol_y = C*sol_x;

%% Inputs

figure(1)
plot(rt.t(1:end-1),sol_u)
title("Input")
xlabel("t (s)")
ylabel("T (N)")
lgnd_names = compose("$$u_%d^*(t)$$", (1:4)');
legend(lgnd_names, 'Interpreter','latex')

%% Position Error
figure(2)
tl = tiledlayout(3,1);
labels = ["$$x$$", "$$y$$", "$$z$$"];
for i = 1:3
   nexttile(i);
   plot(rt.t, rt_des(i,:),'-b');
   hold on
   plot(rt.t, sol_y(i,:), '-r');
   hold off
   title(sprintf("Position: %s", labels(i)), 'Interpreter', 'latex')
   ylabel(sprintf("%s (m)", labels(i)));
end
xlabel("t (s)")
title(tl,"Desired vs. Actual Position", 'FontName', 'Times')
lg = legend(["$$r_{des}(t)$$","$$r^*(t)$$"], 'Interpreter','latex');
lg.Orientation = 'horizontal';
lg.Location = 'southoutside';

%% Orientations
sol_Theta = sol_x(pt.Model.Nx + bm.I.x.Theta, :);
sol_Theta = rad2deg(sol_Theta);

figure(3)
tl = tiledlayout(3,1);
labels = ["$$\phi$$", "$$\theta$$", "$$\psi$$"];
for i = 1:3
   nexttile(i);
   plot(rt.t, sol_Theta(i,:), '-r');
   hold on
   yline(-90)
   yline(90)
   hold off
   title(sprintf("Orientation: %s", labels(i)), 'Interpreter', 'latex')
   ylabel(sprintf("%s (deg)", labels(i)));
end
xlabel("t (s)")
title(tl,"Orientation", 'FontName', 'Times')




