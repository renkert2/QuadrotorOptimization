function IndividualSweeps(objective)
hb = HolyBroS500;
o = Optimization(hb);
o.Objective = objective;

ov = o.OptiVars;
individualSweeps = SweepData.empty();
OO_opt = o.Optimize('OptimizationOutput', true);
for i = 1:numel(ov)
    v = ov(i);
    if ismember(v.Child.Sym,["P","D","kV","Rm","N_s"])
        rev_flag = true;
    else
        rev_flag = false;
    end
    
    individualSweeps(i) = sweep(o, v, 100, 'OptimalPoint', OO_opt, 'ReverseSearch', rev_flag);
end

filename = "individualSweeps_"+objective.Sym+".mat";
save(filename, 'individualSweeps');
end