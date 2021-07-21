classdef FlightProportion < OptiFunctions.Parents.Function
    methods
        function obj = FlightProportion()
            obj.Sym = "ft";
            obj.Unit = "s";
        end
        function v = Value(obj, QR)
            ft = QR.FlightTime;
            charge_time = QR.PT.Battery.ChargeTime;
            
            v = (ft./(ft + charge_time));
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

