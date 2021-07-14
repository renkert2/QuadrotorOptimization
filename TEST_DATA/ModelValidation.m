classdef ModelValidation < handle
    %MODELVALIDATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        FL FlightLog
        M QuadRotorSystem
        Pairs struct
    end
    properties (SetAccess = private)
        MO QRSimOut
    end
    
    methods
        function Simulate(obj)
            obj.MO = obj.M.Simulate('Mode','Nonlinear');
        end
        
        function set.M(obj, val)
            obj.M = val;
            if ~isempty(val.SimOut)
                obj.MO = val.SimOut;
            end
        end
        
        function setTestConditions(obj)
            flight_data = obj.FL;
            
            setVehicleMass(obj.M.QR, flight_data.VehicleMass);
            obj.M.QR.PT.Model.x0(1) = flight_data.StartingSOC;
            obj.M.EnableStop = false;
            set_param(obj.M.Name, 'StopTime', num2str(seconds(flight_data.FlightTime)));
            
            refts = getRefTrajTS(flight_data);
            obj.M.RefTrajTS = refts;
            obj.M.RefTrajVar = "TimeSeries";
            
            distts = getDisturbanceForceTS(flight_data);
            obj.M.DisturbanceTS = distts;
            obj.M.DistVar = "TimeSeries";
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
            % Plot Simulation
            mp = plot(obj.MO, m_plt_args{:});

            hold on
            % Plot FlightLog
            flp = plotTime(obj.FL, fl_plt_args{:}, 'ActiveTime', true);
            hold off
            
            for i = 1:numel(mp)
                mp(i).DisplayName = "Simulated: "+string(mp(i).DisplayName); 
            end
            for i = 1:numel(flp)
                flp(i).DisplayName = "Measured: "+string(flp(i).DisplayName);
            end
        end
        
        function flightTime(obj)
            obj.M.QR.PT.Battery.OperatingSOCRange = [obj.FL.EndingSOC, obj.FL.StartingSOC];
            obj.M.QR.update();
            
            ft_estimate = seconds(obj.M.QR.FlightTime);
            ft_estimate.Format = 'hh:mm:ss';
            
            ft_actual = obj.FL.FlightTime;
            
            err = (ft_estimate - ft_actual)/ft_actual;
            
            fprintf("Estimated Flight Time: %s \n", ft_estimate);
            fprintf("Actual Flight Time: %s \n", ft_actual);
            fprintf("Error: %0.2f %% \n", err*100);
            
        end
    end
    
    methods 
        function s = setPairs(obj)
            c = {"Bus Voltage", "V", {"PT_out", "Internal Voltage"}, {"BAT", "VoltCorr"};...
                "Bus Current", "I", {"PT_out", "Internal Current"}, {"BAT", "CurrCorr"};...
                "Battery SOC", "q", {"PT_out", "Battery SOC"}, {"BAT", ["SOCCurr"]};...
                "Throttle", "u", {"u_out"}, {"RCOU", ["ESCUMean"]};...
                "Height", "m", {"y_out", "Z Position"}, {"POS", "RelHomeAlt"};...
                };
            s = struct('Name', c(:,1), 'YLabel', c(:,2), 'MArgs', c(:,3), 'FLArgs', c(:,4));
            
            obj.Pairs = s;
        end
    end
end

