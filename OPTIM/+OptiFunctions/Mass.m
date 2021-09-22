classdef Mass < OptiFunctions.Parents.QR_Function
    methods
        function obj = Mass(qr)
            obj = obj@OptiFunctions.Parents.QR_Function(qr);
            obj.Sym = "m";
            obj.Unit = "kg";
        end
        function v = Value(obj)
            v = obj.QR.Mass.Value;
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


