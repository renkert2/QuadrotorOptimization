classdef FlightTime < OptiFunctions.Parents.QR_Function
    methods
        function obj = FlightTime(qr)
            obj = obj@OptiFunctions.Parents.QR_Function(qr);
            obj.Sym = "ft";
            obj.Unit = "s";
        end
        function v = Value(obj)
            v = obj.QR.FlightTime;
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

