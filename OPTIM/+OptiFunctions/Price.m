classdef Price < OptiFunctions.Parents.Function
    methods
        function obj = Price()
            obj.Sym = "P";
            obj.Unit = "USD";
        end
        function v = Value(obj, QR)
            v = QR.Price.Value;
        end
        function f = val2f(obj, v)
            f = v;
        end
        function v = f2val(obj, f)
            v = f;
        end
        function g = val2g(obj, v)
            g = v - obj.UB;
        end
        function v = g2val(obj, g)
            v = g + obj.UB;
        end
    end
end


