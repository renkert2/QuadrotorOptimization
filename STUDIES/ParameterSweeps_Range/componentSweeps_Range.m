%% Battery
battSweep = sweep(o, ["N_p","N_s"], 25, 'Objective', 'Range', 'ReverseSearch', true);
save('battSweep_Range.mat', 'battSweep');
title("Battery Carpet Plot")

%% Motor
constraint_func = @(Xp) distToBoundary(o.motorFit.Boundary, Xp);
motorSweep = sweep(o, ["kV","Rm"], 25,'Objective', 'Range', 'ConstraintFunction', constraint_func);

save('motorSweep_Range.mat', 'motorSweep');
title("Motor Carpet Plot")

%% Propeller
propSweep = sweep(o, ["D","P"], 25, 'Objective', 'Range', 'ReverseSearch', true);
save('propSweep_Range.mat', 'propSweep');
