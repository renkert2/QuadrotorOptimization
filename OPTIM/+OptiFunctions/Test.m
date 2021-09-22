classdef Test < OptiFunctions.Parents.QR_Function
    methods
        function obj = Test(qr)
            obj = obj@OptiFunctions.Parents.QR_Function(qr);
            obj.Sym = "ft";
            obj.Unit = "s";
        end
        function v = Value(obj)
            QR = obj.QR;
            v = QR.PT.Propeller.D.Value + QR.PT.Propeller.P.Value + QR.PT.Motor.Rm.Value + QR.PT.Motor.kV.Value;
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
            v = g - obj.UB;
        end
    end
end

