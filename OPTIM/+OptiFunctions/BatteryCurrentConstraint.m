classdef BatteryCurrentConstraint < OptiFunctions.Parents.QR_Function
    methods
        function obj = BatteryCurrentConstraint(varargin)
            obj = obj@OptiFunctions.Parents.QR_Function(varargin{:});
            obj.Sym = "I_margin";
            obj.Unit = "Amps";
        end
        function v = Value(obj)
            batt_max_curr = obj.QR.PT.Battery.DischargeCurrent.Value;            
            v = [batt_max_curr] - obj.QR.SS_QAve.BusCurrent;
        end
        function g = val2g(obj, v)
            g = obj.LB - v;
        end
        function v = g2val(obj, g)
            v = obj.LB - g;
        end
    end
end