classdef Range < OptiFunctions.Parents.QR_Function
    methods
        function obj = Range(qr)
            obj = obj@OptiFunctions.Parents.QR_Function(qr);
            obj.Sym = "d";
            obj.Unit = "m";
        end
        function v = Value(obj)
            v = obj.QR.Range;
        end
        function f = val2f(obj, v)
            f = -v;
        end
        function v = f2val(obj, f)
            v = -f;
        end
        function g = val2g(obj, v)
            g = obj.LB - v;
        end
        function v = g2val(obj, g)
            v = obj.LB - g;
        end
    end
end

