pt = qr.PT;

% Get Powertrain SS linearized about steady state at average battery.
% Eventually we will need to use the real nonlinear dynamics.
% Inputs: Inverter Inputs u_1 through u_4
% Ouptuts: Rotor Speeds omega_1 through omega_4

[A,B,C,D] = CalcMatrices(pt, qr.SS_QAve);


%% Generate Desired Height Profile
mission_length = 5*60;
t = linspace(0,mission_length,1000);
T = t(2) - t(1);
N = numel(t);

% Create 4xN Reference Signal for desired rotor speeds
omega_des = repmat(t,4,1);
omega_des = mod(omega_des,omega_des(end)/2)>(omega_des(end)/4);
omega_des = omega_des.*qr.SS_QAve.RotorSpeed + qr.SS_QAve.RotorSpeed/2;
a_mod = [0.5 0.75 1 1.25];
for i = 1:4
    k = round(i*(1/T)*(mission_length/25));
    omega_des(i,:) =a_mod(i)*circshift(omega_des(i,:), k);
end

%r_des = ones(size(t));
plot(t,omega_des)

%% Discretize the Linear System

[A_d, B_d, C_d, D_d] = LinearModel.Discretize(A,B,C,D,T);

%% Formulate Optimization (YALMIP)
x_ = sdpvar(pt.Model.Nx,N,'full');
u_ = sdpvar(pt.Model.Nu,N-1); 

objs = 0;
for k = 2:N-1
    objs = objs + (1/2)*(C_d*x_(:,k) - omega_des(:,k))'*(C_d*x_(:,k) - omega_des(:,k));
end


% Initial Condition
cons = [x_(:,1) == [1; zeros(pt.Model.Nx-1,1)]];  % Start with Full Battery
for k = 1:N-1
    % Dynamic Constraints
    cons = [cons, x_(:,k+1) == A_d*x_(:,k) + B_d*u_(:,k)];

    % Input Constraints
    cons = [cons, (zeros(pt.Model.Nu,1) <= u_(:,k)) & (u_(:,k) <= ones(pt.Model.Nu,1))];
end

% Final Condition
cons = [cons, x_(1,end)>=0.2];

opts = sdpsettings('solver','quadprog');

tic;
diagnostics = optimize(cons,objs,opts);
solve_time = toc;

%%
sol_u = value(u_);
sol_y = C*value(x_);

figure(1)
tl = tiledlayout(4,1);
for i = 1:4
   nexttile(i);
   plot(omega_des(i,:),'-b');
   hold on
   plot(sol_y(i,:), '-r');
   hold off
   title(sprintf("Rotor %d", i))
   ylabel("$$\omega$$ (rad/s)")
end
xlabel("t (s)")
title(tl,"Rotor Speeds", 'FontName', 'Times')
lg = legend(["$$\omega_{des}(t)$$","$$\omega(t)$$"], 'Interpreter','latex');
lg.Orientation = 'horizontal'
lg.Location = 'southoutside';

figure(2)
plot(t(1:end-1),sol_u)
title("Input")
xlabel("t (s)")
ylabel("T (N)")
lgnd_names = compose("$$u_%d^*(t)$$", (1:4)');
legend(lgnd_names, 'Interpreter','latex')

figure(3)
q = value(x_(1,:));
plot(t, q)
title("Battery SOC")
xlabel("t (s)")
ylabel("$$q$$")