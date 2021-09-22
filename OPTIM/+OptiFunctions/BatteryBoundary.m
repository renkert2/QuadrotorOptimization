classdef BatteryBoundary < OptiFunctions.Parents.QR_Function
    methods
        function obj = BatteryBoundary(qr)
            obj = obj@OptiFunctions.Parents.QR_Function(qr);
            obj.Sym = "B_B";
            obj.Unit = "unit";
        end
        
        function v = Value(obj)
            batt = obj.QR.PT.Battery;
            v = distToBoundary(batt.Fit.Boundary, [batt.Fit.Inputs.Value]');
        end
        
        function g = val2g(obj, v)
            g = v;
        end
        
        function v = g2val(obj, g)
            v = g;
        end
        
    end
end

