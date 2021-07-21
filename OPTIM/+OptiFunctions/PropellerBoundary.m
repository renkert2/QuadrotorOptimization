classdef PropellerBoundary < OptiFunctions.Parents.Function
    methods
        function obj = PropellerBoundary()
            obj.Sym = "B_P";
            obj.Unit = "unit";
        end
        
        function v = Value(obj, QR)
            prop = QR.PT.Propeller;
            v = distToBoundary(prop.Fit.Boundary, [prop.Fit.Inputs.Value]');
        end
        
        function g = val2g(obj, v)
            g = v;
        end
        
        function v = g2val(obj, g)
            v = g;
        end
        
    end
end

