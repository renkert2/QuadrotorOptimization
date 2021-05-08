ov = o.OptiVars;
individualSweeps = SweepData.empty();
for i = 1:numel(ov)
    v = ov(i);
    if ismember(v.Sym,["D","P","kV"])
        rev_flag = true;
    else
        rev_flag = false;
    end
    
    individualSweeps(i) = sweep(o, v, 100, 'ReverseSearch', rev_flag);
end

save('individualSweeps.mat', 'individualSweeps');

