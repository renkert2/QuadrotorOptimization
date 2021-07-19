classdef OptimOutput
    %OPTIMOUTPUT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        OptimTime duration % Time required to complete optimization
        Objective
        SolverFunction
        ParamVals (:,1) compParamValue
        
        X0
        F0
        X_opt
        F_opt
        
        exitflag
        
        %fmincon outputs
        lambda
        lambdaDesc % Descriptions matching form of lambda struct
        grad
        hessian
        
        %ga outputs
        output
        population
        scores
        
        PerformanceData 
        DesignData
    end
    
    properties (Dependent)
        PercentChange
    end
    
    methods
        function p = get.PercentChange(obj)
            p = (obj.F_opt - obj.F0)./obj.F0;
        end
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

