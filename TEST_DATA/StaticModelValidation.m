classdef StaticModelValidation < handle
    %MODELVALIDATION Summary of this class goes here
    %   Detailed explanation goes here
    properties
        FL FlightLog
        QR QuadRotor
        Pairs struct
    end
    
    methods
        function init(obj)
            obj.setPairs;
            
            % Set Operating Conditions
            flight_data = obj.FL;
            obj.QR.PT.Battery.OperatingSOCRange = [flight_data.EndingSOC, flight_data.StartingSOC];
            
            % Load Parameters
            loadValues(obj.QR.Params, obj.FL.Components);
            
            obj.QR.update()
        end
                    
        function flightTime(obj)
            ft_estimate = seconds(obj.QR.FlightTime);
            ft_estimate.Format = 'hh:mm:ss';
            
            ft_actual = obj.FL.FlightTime;
            
            err = (ft_estimate - ft_actual)/ft_actual;
            
            fprintf("Estimated Flight Time: %s \n", ft_estimate);
            fprintf("Actual Flight Time: %s \n", ft_actual);
            fprintf("Error: %0.2f %% \n", err*100);
        end
        
        function plotPairs(obj, arg)
            if nargin == 2
                N = numel(arg);
                if isstring(arg)
                    pairnames = vertcat(obj.Pairs.Name);
                    I = false(size(obj.Pairs));
                    for i = 1:N
                        I = I | contains(pairnames, arg(i));
                    end
                    I = find(I);
                else
                    I = arg;
                end
            else
                N = numel(obj.Pairs);
                I = 1:N;
            end
            for i = I
                s = obj.Pairs(i);
                figure
                compPlot(obj, s.MArgs, s.FLArgs);
                title(s.Name);
                ylabel(s.YLabel, 'Interpreter', 'Latex');
            end
        end
        
        function compPlot(obj, m_plt_args, fl_plt_args)
            % Plot FlightLog
            flp = plotTime(obj.FL, fl_plt_args{:}, 'ActiveTime', false, 'DurationTime', true);
            for i = 1:numel(flp)
                flp(i).DisplayName = "Measured: "+string(flp(i).DisplayName);
            end
            hold on
            
            % Plot Static Model
            for i = 1:numel(m_plt_args)
                m_plt_args = m_plt_args{i};
                data = obj.QR.PerformanceData.SteadyState.(m_plt_args(i));
                mp(i) = yline(data);
                mp(i).DisplayName = "Estimated: "+m_plt_args(i); 
            end
            hold off
        end
    end
    
    methods 
        function s = setPairs(obj)
            c = {"Bus Voltage", "V", {"BusVoltage"}, {"BAT", "VoltCorr"};...
                "Bus Current", "I", {"BusCurrent"}, {"BAT", "CurrCorr"};...
                "Throttle", "u", {"u"}, {"RCOU", ["ESCUMean"]};...
                };
            s = struct('Name', c(:,1), 'YLabel', c(:,2), 'MArgs', c(:,3), 'FLArgs', c(:,4));
            
            obj.Pairs = s;
        end
    end
end

