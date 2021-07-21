classdef OptimObjectives
    enumeration
        Mass
        Price
        FlightTime
        Range
        FlightTimePerPrice
        PaybackPeriod
        FlightProportion
    end
    
    methods
        function f = objfun(obj, QR)
            switch obj
                case OptimObjectives.FlightTime
                    f = -QR.FlightTime;
                case OptimObjectives.Range
                    f = -QR.Range;
                case OptimObjectives.FlightTimePerPrice
                    f = -QR.FlightTime./QR.Price.Value;
                case OptimObjectives.Price
                    f = QR.Price.Value;
                case OptimObjectives.Mass
                    f = QR.Mass.Value;
                case OptimObjectives.PaybackPeriod
                    ft = QR.FlightTime;
                    p = QR.Price.Value;
                    charge_time = QR.PT.Battery.ChargeTime;
                    
                    f = p./(ft./(ft + charge_time));
                case OptimObjectives.FlightProportion
                    ft = QR.FlightTime;
                    charge_time = QR.PT.Battery.ChargeTime;
                    
                    f = -(ft./(ft + charge_time));
            end
        end
        function F = processF(obj, f)
            switch obj
                case OptimObjectives.FlightTime
                    F = -f;
                case OptimObjectives.Range
                    F = -f;
                case OptimObjectives.FlightTimePerPrice
                    F = -f;
                case OptimObjectives.FlightProportion
                    F = -f;
                otherwise
                    F = f;
            end
        end
    end
end

