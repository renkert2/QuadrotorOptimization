%% LinearModel from GraphModel
qr = QuadRotor();
%%
[ssMod, x0, u0, y0] = calcLinearModel(qr); % Obtain linearized model at hover
ssMod.InputName = 'u';
ssMod.OutputName = 'T';
gain = dcgain(ssMod);
dom_pole = min(abs(eigs(ssMod.A)));
ssMod_simple = tf(gain, [(1/dom_pole), 1]);

%% Get Linear Interp of Steady-State Input vs. Batt SOC
q_vals = 0:0.05:1;
u0_vals = zeros(size(q_vals));
for i = 1:numel(q_vals)
    ss_temp = calcSteadyState(qr, [], q_vals(i));
    u0_vals(i) = ss_temp.u;
end

u0_func = @(q) interp1(q_vals, u0_vals, q, 'pchip');

%% Body Model
% u(T) = T-F_g = thrust - gravitational force
m = getProp(qr.extrinsicProps, 'Mass'); % kg

% States: X = [y; dy/dt]
A = [0 1;0 0];
B = [0; 1/m];
C = [1 0];
D = [0];

body = ss(A,B,C,D);

%step(body)
%% F This - PID
plant = series(ssMod_simple,body);

opts = pidtuneOptions('DesignFocus','disturbance-rejection', 'PhaseMargin', 60);
[pdController, pidinfo] = pidtune(plant, 'PD', opts);
K_PD = [pdController.Kp, pdController.Kd];


clsys = feedback(series(pdController,plant),1);

%% Controller Model
% Computes states [y diff(y)]

Ac = [0 1;0 0];
Bc = [0; 1/m];
Cc = [1 0; 0 1];
Dc = [0];

C_ss = ss(Ac,Bc,Cc,Dc);

f_c = @(x_c,u_c) Ac*x_c + Bc*u_c;
g_c = @(x_c,u_c) Cc*x_c + Dc*u_c;

%% Reference Filter
f_r = @(x_r, u_r, tau) -(1/tau)*x_r + u_r;
g_r = @(x_r, u_r, tau) [1/tau; -1/tau^2]*x_r + [0; 1/tau]*u_r;
%% All Together Now
r_ = @(t) (t>=0) - (1/2)*(t>=250) + ((t-500)/500).*(t>=500 & t<=750);


% Indices of x_g, x_c in composite x = [x_g; x_c; x_r]
i_x_g = 1:qr.SimpleModel.Nx;
i_x_c = i_x_g(end) + (1:size(Ac,1));
i_x_r = i_x_c(end) + 1;

x_0 = zeros(i_x_c(end),1);
x_0(i_x_g) = x0;
x_0(i_x_g(1)) = 1;
x_0(i_x_c) = [0;0];
x_0(i_x_r) = 0;

%%

[t,x] = ode23tb(@(t,x) f_sys_cl(t,x, i_x_g, i_x_c, i_x_r, K_PD, r_, qr.SimpleModel, f_c, f_r, g_r, 1, u0_func, y0), [0 1000], x_0);
x_c = x(:,i_x_c);
y = x_c(:,1);
figure(1)
plot(t,y,t,r_(t));
title("Tracking Performance");
legend(["System", "Reference"]);
xlabel("t")
ylabel("Height (m)")

% u = [];
% e = [];
% for i = 1:numel(t)
%     [e_i,u_i] = g_sys_cl(t(i),x(i,:)', i_x_g, i_x_c, K_PD, r_PD, u0_func);
%     u = [u;u_i];
%     e = [e; e_i'];
% end
% 
% figure(2)
% plot(t,u)
% title('Inverter Input')
% xlabel('t')
% ylabel('u')
% 
% figure(3)
% plot(t,e)
% title('Error Signal')
% xlabel('t')
% ylabel('e')
% legend(["P", "D"])


function x_dot = f_sys_cl(t,x, i_x_g, i_x_c, i_x_r, K_PD, r, model, f_c, f_r, g_r, tau, u0_func, y0)
    x_g = x(i_x_g);
    x_c = x(i_x_c);
    x_r = x(i_x_r);
    
    dot_x_r = f_r(x_r,r(t),tau);
    r_PD = g_r(x_r,r(t),tau);
    
    e = r_PD-x_c;
    u = K_PD*e + u0_func(x_g(1));
    u = min(max(u,0),1);
    
    dot_x_g = model.CalcF(x_g, u, []);
    
    y = model.CalcG(x_g,u, []);
    T = y(end) - y0(end);
    dot_x_c = f_c(x_c, T);
        
    x_dot = [dot_x_g; dot_x_c; dot_x_r];    
end

function [e,u] = g_sys_cl(t,x, i_x_g, i_x_c, K_PD, r_PD, u0_func)
    x_g = x(i_x_g);
    x_c = x(i_x_c);
    
    e = r_PD(t)-x_c;
    u = K_PD*e + u0_func(x_g(1));
    u = min(max(u,0),1);
end



