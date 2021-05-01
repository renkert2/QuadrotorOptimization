ov = o.OptiVars;
individualSweeps_Range = SweepData.empty();
for i = 1:numel(ov)
    v = ov(i);
    if ismember(v.Sym,["D","P","kV"])
        rev_flag = true;
    else
        rev_flag = false;
    end
    
    individualSweeps_Range(i) = sweep(o, v, 50, 'Objective', 'Range', 'ReverseSearch', rev_flag);
end

save('individualSweeps_Range.mat', 'individualSweeps_Range');

%%
for i = 1:1
individualSweeps_Range(i).plotObjective()
end

