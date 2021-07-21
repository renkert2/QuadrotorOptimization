classdef PowerTrainSystem < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        Name char = 'PowerTrain_Simulink'
    end
    
    properties
        QR QuadRotor
        PT PowerTrain
        Wks Simulink.ModelWorkspace
        
        PT_Full
        PT_Mod
        PT_Simple
        
        SimOut
    end

    methods
        function obj = PowerTrainSystem(QR)
            obj.QR = QR;
            obj.PT = QR.PT;
        end
        
        function init(obj)
            makeSimulinkModel(obj.PT.Model, obj.Name);
            obj.Wks = get_param(obj.Name, 'ModelWorkspace');
            assignin(obj.Wks, "PT_Object", obj.PT);
            assignin(obj.Wks, "SYS_Object", obj);
            setLinearMatrices(obj);
        end
        
        function setLinearMatrices(obj)
            %% PowerTrain Model
            [Apt,Bpt,Cpt,Dpt] = CalcMatrices(obj.QR.PT, obj.QR.SS_QAve);
            obj.PT_Full = ss(Apt,Bpt,Cpt,Dpt);
            
            % Omit Battery Dynamics
            Aptmod = Apt(2:end,2:end);
            Bptmod = Bpt(2:end,:);
            Cptmod = Cpt(:,2:end);
            Dptmod = Dpt(:,:);
            obj.PT_Mod = ss(Aptmod, Bptmod, Cptmod, Dptmod);
            
            gain = Dptmod-Cptmod*(Aptmod\Bptmod);
            dom_pole = min(abs(eigs(Apt)));
            den = [(1/dom_pole), 1];
            PT = tf(num2cell(gain), den);
            PT.InputName = {'u'};
            PT.OutputName = {'W'};
            obj.PT_Simple = ss(PT);
        end
        
        function Simulate(obj)
            obj.SimOut = sim(obj.Name);
        end
        
        function plot(obj)
            f = figure(1);
            ax = axes(f);
            xlabel("$$t$$", 'Interpreter', 'latex')
            ylabel("$$\omega$$ (rad/s)", 'Interpreter', 'latex')
            title("Step Input: Rotor Speed")
            hold on
            
            s = ["omega", "omega_linear_full", "omega_linear_nobatt", "omega_linear_firstorder"];
            lgnd_names = ["Nonlinear Model", "Linearized - Battery", "Linearized - No Battery", "Linearized - First Order"];
            for i = 1:numel(s)
                d = get(obj.SimOut.yout, s(i));
                ts = d.Values;
                plt = plot(ax,ts.Time,ts.Data);
                plt.DisplayName = lgnd_names(i);  
            end
            hold off
            legend
            
            figure(2);
            
            subplot(1,2,1)
            d = get(obj.SimOut.yout, "q");
            ts = d.Values;
            plot(ts.Time, ts.Data);
            ylim([0 1])
            xlabel("$$t$$", 'Interpreter', 'latex')
            ylabel("$$q$$", 'Interpreter', 'latex')
            title("Step Input: Battery SOC")
            
            subplot(1,2,2)
            d = get(obj.SimOut.yout, "omega");
            ts = d.Values;
            plot(ts.Time, ts.Data);
            xlabel("$$t$$", 'Interpreter', 'latex')
            ylabel("$$\omega$$ (rad/s)", 'Interpreter', 'latex')
            title("Step Input: Rotor Speed")
        end
            
    end
end

