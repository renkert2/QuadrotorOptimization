%% Optimize Battery Only
o.OptiVars.setEnabled([1 2 5 6], false(1,4));
% Switch into Continuous Mode
setDependent(o.DependentParams, true);
%% Set Trajectory
o.QRS.setLQR(0.1)

%% Set to Smallest Feasible Battery and Evaluate Performance
o.OptiVars(3).Value = 6;
o.OptiVars(4).Value = 1000;
o.updateQR(false);
K_small = qrs.K_lqr;
ss_small = qrs.LinearPlantModel;
Am_small = qrs.getClosedLoopMatrix(K_small);

%% Set to Largest Feasible Battery and Evaluate Performance
o.OptiVars(3).Value = 6;
o.OptiVars(4).Value = 6000;
o.updateQR(false);

K_large = qrs.K_lqr;
ss_large = qrs.LinearPlantModel;
Am_large = qrs.getClosedLoopMatrix(K_large);

%% Compare Results - Closed Loop Eigenvalues
eig_small = eig(Am_small);
eig_large = eig(Am_large);

figure(1)
plot(real(eig_small), imag(eig_small), 'ob')
hold on
plot(real(eig_large), imag(eig_large), 'or')
hold off

legend(["Small Battery", "Large Battery"])
title("Closed-Loop Eigenvalues")
grid on

t = table(eig_small, eig_large, 'VariableNames', ["Eigenvalues (Small)", "Eigenvalues (Large)"]);

table2latex(t, 'EigVals');

%% Open-Loop Eigenvalues - not super useful
figure(2)
oleig_small = eig(ss_small.A);
plot(real(oleig_small), imag(oleig_small), 'ob')
hold on
oleig_large = eig(ss_large.A);
plot(real(oleig_large), imag(oleig_large), 'or')
hold off

legend(["Small Battery", "Large Battery"])
title("Open-Loop Eigenvalues")
grid on

%% Controllability
c_small = ctrb(ss_small.A, ss_small.B);
[u_small,Sigma_small,~] = svd(c_small);

c_large = ctrb(ss_large.A, ss_large.B);
[u_large,Sigma_large,~] = svd(c_large);