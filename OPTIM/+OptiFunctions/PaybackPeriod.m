classdef PaybackPeriod < OptiFunctions.Parents.QR_Function
    methods
        function obj = PaybackPeriod(qr)
            obj = obj@OptiFunctions.Parents.QR_Function(qr);
            obj.Sym = "ft/P";
            obj.Unit = "s/$";
        end
        function v = Value(obj)
            QR = obj.QR;
            ft = QR.FlightTime;
            p = QR.Price.Value;
            charge_time = QR.PT.Battery.ChargeTime;
            
            v = p./(ft./(ft + charge_time));
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

