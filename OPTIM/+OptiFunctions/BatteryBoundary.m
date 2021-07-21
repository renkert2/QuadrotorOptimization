classdef BatteryBoundary < OptiFunctions.Parents.Function
    methods
        function obj = BatteryBoundary()
            obj.Sym = "B_B";
            obj.Unit = "unit";
        end
        
        function v = Value(obj, QR)
            batt = QR.PT.Battery;
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

