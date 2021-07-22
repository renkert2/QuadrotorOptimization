%% Battery
% constraint_func = @(Xp) distToBoundary(o.QR.Battery.Fit.Boundary, Xp);
% [X,ft,X_opt,ft_opt,I,PD,DD] = sweep(o, ["N_s","Q"], 50, 'ReverseSearch', true, 'ConstraintFunction', constraint_func);
% s = struct();
% s.X = X;
% s.ft = ft;
% s.X_opt = X_opt;
% s.ft_opt = ft_opt;
% s.I = I;
% s.PD = PD;
% s.DD = DD;
% battSweep = s;
% save('battSweep.mat', 'battSweep');
% title("Battery Carpet Plot")

%% Motor
% constraint_func = @(Xp) distToBoundary(o.QR.Motor.Fit.Boundary, Xp);
% [X,ft,X_opt,ft_opt,I,PD,DD] = sweep(o, ["kV","Rm"], 50, 'ConstraintFunction', constraint_func);
% s = struct();
% s.X = X;
% s.ft = ft;
% s.X_opt = X_opt;
% s.ft_opt = ft_opt;
% s.I = I;
% s.PD = PD;
% s.DD = DD;
% motorSweep = s;
% save('motorSweep.mat', 'motorSweep');
% title("Motor Carpet Plot")

%% Propeller
% constraint_func = @(Xp) distToBoundary(o.QR.Propeller.Fit.Boundary, Xp);
% so = sweep(o, ["D","P"], 25, 'ConstraintFunction', constraint_func);

propSweep = so;
save('propSweep.mat', 'propSweep');
title("Propeller Carpet Plot")
