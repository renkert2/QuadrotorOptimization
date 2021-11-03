classdef Price < OptiFunctions.Parents.QR_Function
    methods
        function obj = Price(qr)
            obj = obj@OptiFunctions.Parents.QR_Function(qr);
            obj.Sym = "P";
            obj.Unit = "USD";
        end
        function v = Value(obj)
            v = obj.QR.Price.Value;
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


