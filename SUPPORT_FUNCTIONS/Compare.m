function [pc] = Compare(reference,candidate)
%COMPARE Finds the % change from reference to cadidate

assert(numel(reference) == numel(candidate), "Objects to be compared must be of same size");

if isnumeric(reference)
    pc = relChange(reference, candidate);
else
    fnames = string(fieldnames(reference));
    pc = struct();
    for i = 1:numel(fnames)
        f = fnames(i);
        r = reference.(f);
        c = candidate.(f);
        pc.(fnames(i)) = relChange(r,c);
    end
end
end

function pc = relChange(r,c)
    if isnumeric(r) && isnumeric(c)
        pc = (c - r)./r;
    else
        pc = NaN;
    end
end