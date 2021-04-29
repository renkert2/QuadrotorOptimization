classdef OptimOutput
    %OPTIMOUTPUT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Objective OptimObjectives
        
        X_opt
        F_opt
        
        exitflag
        lambda
        grad
        hessian
        
        PerformanceData 
        DesignData
    end
    
    methods
        function s = lambdaData(obj_array)
            N = numel(obj_array);
            l = [obj_array.lambda];
            I = [obj_array.exitflag] ~= (-4);
            fnames = fields(l);
            s = struct();
            
            for i = 1:numel(fnames)
                f = fnames{i};
                dat_in = [l.(f)];
                dat = NaN(size(dat_in,1),N);
                dat(:,I) = dat_in;
                s.(f) = dat;
            end
        end
        
    end
        
end

