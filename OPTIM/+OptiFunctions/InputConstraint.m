classdef InputConstraint < OptiFunctions.Parents.Function
    methods
        function obj = InputConstraint()
            obj.Sym = "u";
            obj.Unit = "unit";
        end
        
        function v = Value(obj, QR)
            v = QR.SS_QAve.u;
        end
        
        function g = val2g(obj, v)
            g = v - 1;
        end
        
        function v = g2val(obj, g)
            v = g + 1;
        end
    end
end

