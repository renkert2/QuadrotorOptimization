classdef PropellerBoundary < OptiFunctions.Parents.QR_Function
    methods
        function obj = PropellerBoundary(qr)
            obj = obj@OptiFunctions.Parents.QR_Function(qr);
            obj.Sym = "B_P";
            obj.Unit = "unit";
        end
        
        function v = Value(obj)
            prop = obj.QR.PT.Propeller;
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

