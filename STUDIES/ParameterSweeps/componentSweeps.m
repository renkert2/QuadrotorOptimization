%% Battery
[X,ft,X_opt,ft_opt,I,PD,DD] = sweep(o, ["N_p","N_s"], 50, 'ReverseSearch', true);
s = struct();
s.X = X;
s.ft = ft;
s.X_opt = X_opt;
s.ft_opt = ft_opt;
s.I = I;
s.PD = PD;
s.DD = DD;
battSweep = s;
save('battSweep.mat', 'battSweep');
title("Battery Carpet Plot")

%% Motor
constraint_func = @(Xp) distToBoundary(o.motorFit.Boundary, Xp);
[X,ft,X_opt,ft_opt,I,PD,DD] = sweep(o, ["kV","Rm"], 50, constraint_func);
s = struct();
s.X = X;
s.ft = ft;
s.X_opt = X_opt;
s.ft_opt = ft_opt;
s.I = I;
s.PD = PD;
s.DD = DD;
motorSweep = s;
save('motorSweep.mat', 'motorSweep');
title("Motor Carpet Plot")

%% Propeller
[X,ft,X_opt,ft_opt,I,PD,DD] = sweep(o, ["D","P"], 50);
s = struct();
s.X = X;
s.ft = ft;
s.X_opt = X_opt;
s.ft_opt = ft_opt;
s.I = I;
s.PD = PD;
s.DD = DD;
propSweep = s;
save('propSweep.mat', 'propSweep');
title("Propeller Carpet Plot")
