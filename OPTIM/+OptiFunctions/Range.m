classdef Range < OptiFunctions.Parents.Function
    methods
        function obj = Range()
            obj.Sym = "d";
            obj.Unit = "m";
        end
        function v = Value(obj, QR)
            v = QR.Range;
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

