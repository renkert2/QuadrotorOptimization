classdef InverterCurrentConstraint < OptiFunctions.Parents.QR_Function
    methods
        function obj = InverterCurrentConstraint(qr, varargin)
            obj = obj@OptiFunctions.Parents.QR_Function(qr, varargin{:});
            obj.Sym = "I_margin";
            obj.Unit = "Amps";
        end
        function v = Value(obj)
            esc_max_curr = obj.QR.PT.Inverter.I_max.Value;
            v = [esc_max_curr] - obj.QR.SS_QAve.InverterCurrent_DC;
        end
        function g = val2g(obj, v)
            g = obj.LB - v;
        end
        function v = g2val(obj, g)
            v = obj.LB - g;
        end
    end
end