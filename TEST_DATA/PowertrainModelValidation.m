classdef PowertrainModelValidation < handle
    %MODELVALIDATION Summary of this class goes here
    %   Detailed explanation goes here
    properties (Constant)
        Name = "PowerTrain_Simulink_ModelValidation"
    end
    
    properties
        Wks Simulink.ModelWorkspace
        
        FL FlightLog
        PT PowerTrain
        Pairs struct
        
        InputTS timeseries
    end
    
    properties (SetAccess = private)
        SO QRSimOut
        SimOutDesc struct
    end
    
    methods
        function Simulate(obj)
            so = sim(obj.Name);
            obj.SO = QRSimOut(so, obj.SimOutDesc);
        end        
        
        function init(obj)
            obj.setPairs;
            obj.setSimOutDesc;
            makeSimulinkModel(obj.PT.Model, obj.Name);
            obj.Wks = get_param(obj.Name, 'ModelWorkspace');
            assignin(obj.Wks, "PT_Object", obj.PT);
            assignin(obj.Wks, "SYS_Object", obj);
        end
        
        function setTestConditions(obj)
            flight_data = obj.FL;

            obj.PT.Model.x0(1) = flight_data.StartingSOC;
            
            input = flight_data.Data.RCOU.ESCUMean;
            input_t = flight_data.Data.RCOU.TimeUS/1e6;
            obj.InputTS = timeseries(input, input_t);
            
            set_param(obj.Name, 'StopTime', num2str(input_t(end)));
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
            mp = plot(obj.SO, m_plt_args{:});

            hold on
            % Plot FlightLog
            flp = plotTime(obj.FL, fl_plt_args{:}, 'ActiveTime', false, 'DurationTime', true);
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
                "Bus Current", "I", {"PT_out", "Internal Current"}, {"BAT", "CurrCorr"};...
                "Battery SOC", "q", {"PT_out", "Battery SOC"}, {"BAT", ["SOC" "SOCCurr"]};...
                };
            s = struct('Name', c(:,1), 'YLabel', c(:,2), 'MArgs', c(:,3), 'FLArgs', c(:,4));
            
            obj.Pairs = s;
        end
    end
    
        
    methods (Hidden)% Helper Functions
        function s = setSimOutDesc(obj)
            s.PT_out = obj.PT.Model.OutputDescriptions;
            obj.SimOutDesc = s;
        end
    end
end

