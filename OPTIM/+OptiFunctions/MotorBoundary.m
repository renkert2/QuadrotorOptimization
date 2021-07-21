classdef MotorBoundary < OptiFunctions.Parents.Function
    methods
        function obj = MotorBoundary()
            obj.Sym = "B_M";
            obj.Unit = "unit";
        end
        
        function v = Value(obj, QR)
            mot = QR.PT.Motor;
            v = distToBoundary(mot.Fit.Boundary, [mot.Fit.Inputs.Value]');
        end
        
        function g = val2g(obj, v)
            g = v;
        end
        
        function v = g2val(obj, g)
            v = g;
        end
        
    end
end

