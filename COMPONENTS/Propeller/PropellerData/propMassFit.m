classdef propMassFit
    %MASSFIT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Data table
        Fit cfit
        
        lb double
        ub double
    end
    
    methods
        function  [m,J] = calcMassProps(obj, D)
            out_points = ~obj.isInBoundary(D);
            if any(out_points)
                out_str = num2str(find(out_points), '%d, ');
                out_str = [out_str, num2str(D)];
                warning("Extrapolating points: %s",out_str);
            end
            
            m = obj.Fit(D);
            J = 1/12*m.*D.^2;
        end
        
        function l = isInBoundary(obj,D)
            l = (D >= obj.lb) & (D <= obj.ub);
        end
    end
end

