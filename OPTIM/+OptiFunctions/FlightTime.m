classdef FlightTime < OptiFunctions.Parents.Function
    methods
        function obj = FlightTime()
            obj.Sym = "ft";
            obj.Unit = "s";
        end
        function v = Value(obj, QR)
            v = QR.FlightTime;
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

