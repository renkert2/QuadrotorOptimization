classdef Output
    %OPTIMOUTPUT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Counter Counter
        Objective
        
        X0
        F0 (1,1)
        X_opt
        F_opt (1,1)
        
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
    end   
end

