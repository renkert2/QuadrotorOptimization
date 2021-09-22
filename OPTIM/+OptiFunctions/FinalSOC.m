classdef FinalSOC < OptiFunctions.Parents.QRS_Function
    methods
        function obj = FinalSOC(qrs, varargin)
            obj = obj@OptiFunctions.Parents.QRS_Function(qrs, varargin{:});
            obj.Sym = "q_f";
            obj.Unit = "unit";
        end
        function v = Value(obj)
            [~,~,x] = getTS(obj.QRS.SimOut, 'PT_out', "Battery SOC");
            v = x(end);
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