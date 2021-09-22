classdef InputConstraint < OptiFunctions.Parents.QR_Function
    methods
        function obj = InputConstraint(qr)
            obj = obj@OptiFunctions.Parents.QR_Function(qr);
            obj.Sym = "u";
            obj.Unit = "unit";
        end
        
        function v = Value(obj)
            v = obj.QR.SS_QAve.u;
        end
        
        function g = val2g(obj, v)
            g = v - 1;
        end
        
        function v = g2val(obj, g)
            v = g + 1;
        end
    end
end

