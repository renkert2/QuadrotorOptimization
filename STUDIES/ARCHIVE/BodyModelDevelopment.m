bm = BodyModel(QR_S500.Params);
lm = getLinearModel(bm);

modelName = 'BodyModel_Simulink';
sim_h = makeSimulinkModel(bm, modelName);
open_system(sim_h)
mdlWks = get_param(sim_h, 'ModelWorkspace');
%%
x_ss = zeros(12,1);
u_ss = bm.calcSteadyStateInput(x_ss, [], repmat(580,4,1));
assignin(mdlWks, 'u_ss', u_ss);

%%
x_des  = zeros(12,1);
x_des(bm.I.x.p) = [10,10,-10]';

assignin(mdlWks, 'x_des', x_des)
%%
[A,B] = CalcMatrices(lm,x_ss,u_ss,[]);

Q = eye(bm.Nx);
R = 0.01*eye(bm.Nu);

N = [0];

K_lqr = lqr(A,B,Q,R,N);
assignin(mdlWks, 'K_lqr', K_lqr);

%% Motor Dynamics
k = 1e6;
A_m = -k.*eye(12);
B_m = k.*eye(12);
C_m = eye(12);
D_m = zeros(12);
assignin(mdlWks, 'A_m', A_m);
assignin(mdlWks, 'B_m', B_m);
assignin(mdlWks, 'C_m', C_m);
assignin(mdlWks, 'D_m', D_m);

%% Simulate


