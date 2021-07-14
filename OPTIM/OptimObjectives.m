classdef OptimObjectives
    enumeration
        FlightTime
        Range
        FlightTimePerPrice
        Price
        Mass
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
            end
        end
        function F = processF(obj, f)
            switch obj
                case OptimObjectives.FlightTime
                    F = seconds(-f);
                    F.Format = 'hh:mm:ss';
                case OptimObjectives.Range
                    F = -f;
                case OptimObjectives.FlightTimePerPrice
                    F = -f;
                otherwise
                    F = f;
            end
        end
    end
end

