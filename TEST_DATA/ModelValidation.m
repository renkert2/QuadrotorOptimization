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
            loadTestConditions(obj.M, obj.FL);
            obj.MO = obj.M.Simulate('Mode','Nonlinear');
        end
        
        function set.M(obj, val)
            obj.M = val;
            if ~isempty(val.SimOut)
                obj.MO = val.SimOut;
            end
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
                if s.Name == "Throttle"
                    % Hacky fix of throttle plot
                    ax = gca;
                    c = ax.Children;
                    d = vertcat(c(3:end).YData);
                    d = mean(d,1);
                    c(3).YData = d;
                    c(3).DisplayName = "Simulated: Inverter Input";
                    delete(c(4:end))
                    c(1).Color = 'r';
                    c(3).Color = 'b';
                    ax.Children = c([3 1 2]); 
                end
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
    end
    
    methods 
        function s = setPairs(obj)
            c = {"Bus Voltage", "V", {"PT_out", "Internal Voltage"}, {"BAT", "VoltCorr"};...
                "Bus Current", "I", {"PT_out", "Internal Current"}, {"BAT", "Curr"};...
                "Battery SOC", "q", {"PT_out", "Battery SOC"}, {"BAT", "SOC"};...
                "Throttle", "u", {"u_out"}, {"CTUN", ["ThOCorr", "ThHCorr"]};...
                "Height", "m", {"y_out", "Z Position"}, {"POS", "RelHomeAlt"};...
                };
            s = struct('Name', c(:,1), 'YLabel', c(:,2), 'MArgs', c(:,3), 'FLArgs', c(:,4));
            
            obj.Pairs = s;
        end
    end
end

