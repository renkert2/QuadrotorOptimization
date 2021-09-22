classdef TrackingError < OptiFunctions.Parents.QRS_Function
    methods
        function obj = TrackingError(qrs)
            obj = obj@OptiFunctions.Parents.QRS_Function(qrs);
            obj.Sym = "E";
            obj.Unit = "m*s";
        end
        function v = Value(obj)
            v = obj.QRS.trackingError();
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