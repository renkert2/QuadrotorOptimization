function componentSweeps(objective)
functions = [@BatterySweep, @MotorSweep, @PropellerSweep];
parfor i = 1:numel(functions)
    hb = HolyBroS500;
    o = Optimization(hb);
    o.Objective = objective;
    f = functions(i);
    f(o);
end
end

function BatterySweep(o)
constraint_func = OptiFunctions.BatteryBoundary;
so = sweep(o, ["N_s","Q"], 50, 'ReverseSearch', true, 'ConstraintFunction', constraint_func);
battSweep = so;
saveObj(battSweep, 'battSweep', o.Objective);
end

function MotorSweep(o)
%% Motor
constraint_func = OptiFunctions.MotorBoundary;
so = sweep(o, ["kV","Rm"], 50, 'ConstraintFunction', constraint_func);
motorSweep = so;
saveObj(motorSweep, 'motorSweep', o.Objective);
end

function PropellerSweep(o)
%% Propeller
constraint_func = OptiFunctions.PropellerBoundary;
so = sweep(o, ["D","P"], 25, 'ConstraintFunction', constraint_func);
propSweep = so;
saveObj(propSweep, 'propSweep', o.Objective);
end

function saveObj(SO, name, objective)
    file_name = string(name)+"_"+string(objective.Sym)+".mat";
    save(file_name, "SO");
end
