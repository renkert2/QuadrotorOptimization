classdef TrackingError < OptiFunctions.Parents.Function
    methods
        function obj = TrackingError()
            obj.Sym = "E";
            obj.Unit = "m*s";
        end
        function v = Value(obj, QRS)
            v = QRS.trackingError();
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
            v = obj.UB + g;
        end
    end
end