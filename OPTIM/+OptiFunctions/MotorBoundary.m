classdef MotorBoundary < OptiFunctions.Parents.QR_Function
    methods
        function obj = MotorBoundary(qr)
            obj = obj@OptiFunctions.Parents.QR_Function(qr);
            obj.Sym = "B_M";
            obj.Unit = "unit";
        end
        
        function v = Value(obj)
            mot = obj.QR.PT.Motor;
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

