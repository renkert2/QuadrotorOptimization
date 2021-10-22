m = 1.5392;
g = 9.81;

A = [0 1; 0 0];
B = [0; 1/m];
C = [1 0];
D = [0];
sys = ss(A,B,C,D);

%% Discretize the Linear System
T = 0.01;
A_d = expm(A*T);

fun = @(dt) expm(dt*A);
Aint  = integral(fun,0,T,'AbsTol',1e-8,'RelTol',1e-8,'ArrayValued', true);
B_d = Aint*B;
C_d = C;
D_d = D;

%% Generate Desired Height Profile
t = 0:T:10;
N = numel(t);
r_des = mod(t,t(end)/2)>t(end)/4;
%r_des = ones(size(t));
plot(t,r_des)


%% Formulate Optimization (YALMIP)
x_ = sdpvar(2,N,'full');
u_ = sdpvar(1,N-1); 

objs = 0;
for k = 2:N-1
    objs = objs + (1/2)*(C_d*x_(:,k) - r_des(k))'*(C_d*x_(:,k) - r_des(k));
end


% Initial Condition
cons = [x_(:,1) == [0;0]]; 
for k = 1:N-1
    % Dynamic Constraints
    cons = [cons, x_(:,k+1) == A_d*x_(:,k) + B_d*u_(k)];

    % Input Constraints
    cons = [cons, -m*g <= u_(k) <= 10];

    % State Constraints
    cons = [cons, x_(1,k+1) >= 0]; % Cant go below ground
end

opts = sdpsettings('solver','quadprog');
diagnostics = optimize(cons,objs,opts);

%%
sol_u = m*g + value(u_);
sol_y = C*value(x_);

figure(1)
plot(t,r_des,t,sol_y)
title("Height")
xlabel("t (s)")
ylabel("h (m)")
legend(["$$r(t)$$", "$$y(t)$$"], 'Interpreter','latex')


figure(2)
plot(t(1:end-1),sol_u)
title("Input")
xlabel("t (s)")
ylabel("T (N)")
legend(["$$u^*(t)$$"], 'Interpreter','latex')
