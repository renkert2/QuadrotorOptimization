classdef QuadRotorSystem < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        Name char = 'QuadRotor_Simulink'
        PlantSubsystem char = 'QuadRotor_Simulink/PlantSubsystem'
        CombinedLinearPlantModel char = 'QuadRotor_Simulink/PlantSubsystem/CombinedLinearPlantModel'
        IndividualPlantModel char = 'QuadRotor_Simulink/PlantSubsystem/IndividualPlantModel'
        PowertrainSubsystem char = 'QuadRotor_Simulink/PlantSubsystem/IndividualPlantModel/PowertrainSubsystem'
        BodySubsystem char = 'QuadRotor_Simulink/PlantSubsystem/IndividualPlantModel/BodySubsystem'
    end
    
    properties
        QR QuadRotor
        Wks Simulink.ModelWorkspace
        
        RefTraj ReferenceTrajectory3D
        
        SimOut Simulink.SimulationOutput % Cache for simulation output
        SimOutLinear Simulink.SimulationOutput
    end
    
    properties
        K_lqr double
        SysSteadyState double
        
        LinearPlantModel
        LinearPowertrainModel
        LinearBodyModel
    end

    methods
        function obj = QuadRotorSystem(QR)
            obj.QR = QR;
        end
        
        function init(obj)
            obj.Wks = get_param(obj.Name, 'ModelWorkspace');
            assignin(obj.Wks, "QR_Object", obj);
            makeSimulinkModel(obj.QR.BM, obj.Name);
            makeSimulinkModel(obj.QR.PT.Model, obj.Name);
            
            obj.RefTraj = ReferenceTrajectory3D();
            obj.RefTraj.Speed = 1;
            obj.RefTraj.Lemniscate('a',10, 'PotatoChipHeight', 2);
            obj.RefTraj.init();
            obj.RefTraj.setTimeSeries(1);
            
            obj.setLQR();
        end
        
        
        function [t,y,r] = Simulate(obj, opts)
            arguments
                obj
                opts.Mode string = "Nonlinear"
            end
            
            setVariant(obj, opts.Mode);
            
            simOut = sim(obj.Name);
            if nargout
                t = simOut.tout;
                y = get(simOut.yout, 'y_out').Values.Data;
                r = get(simOut.yout, 'r_out').Values.Data;
            end
            
            switch opts.Mode
                case "Nonlinear"
                    obj.SimOut = simOut;
                case "Linear"
                    obj.SimOutLinear = simOut;
            end
        end
        
        function ax = plot(obj, so, pltopts)
            arguments
                obj
                so Simulink.SimulationOutput = obj.SimOut
                pltopts cell = {}
            end
            y = get(so.yout, 'y_out').Values.Data;
            if ~isempty(obj.SimOutLinear)
                sol = obj.SimOutLinear;
                yl = get(sol.yout, 'y_out').Values.Data;
                arg = {y,yl};
                lgnd_names = ["Nonlinear Model Trajectory", "Linear Model Trajectory"];
            else
                arg = y;
                lgnd_names = ["Trajectory"];
            end
            ax = obj.QR.BM.plot(arg, 'RefTraj', obj.RefTraj, 'TrajectoryNames', lgnd_names, pltopts{:});
        end
        
        function ax = animate(obj, so)
            if nargin == 1
                so = obj.SimOut;
            end
            
            t = so.tout;
            y = get(so.yout, 'y_out').Values.Data;
            ax = obj.QR.BM.animate(t,y, 'RefTraj', obj.RefTraj);
        end
        
        function setLQR(obj, rho)
            arguments
                obj
                rho (1,1) double = 1
            end
            
            %% PowerTrain Model
            PT = obj.QR.PT.getFirstOrderSS(obj.QR.SS_QAve);
            PT.InputName = {'u'};
            PT.OutputName = {'W'};
            obj.LinearPowertrainModel = PT;
            
            
            %% Body Model
            x_ss = zeros(12,1);
            u_ss = obj.QR.BM.calcSteadyStateInput(x_ss, [], repmat(obj.QR.HoverSpeed(),4,1));
            [Ab,Bb,~,Cb,Db,~] = CalcMatrices(obj.QR.BM.LinearModel,x_ss,u_ss,[]);
            obj.SysSteadyState = [u_ss;x_ss];
            
            BM = ss(Ab,Bb,Cb,Db);
            BM.InputName = {'W'};
            BM.OutputName = {'y'};
            obj.LinearBodyModel = BM;
            
            plant = connect(PT,BM, 'u', {'W','y'});
            obj.LinearPlantModel = plant;
            
            Q = eye(size(plant.A,1));
            Q(1:4,1:4) = 0; % We don't care about the speed state
            R = rho*eye(size(plant.B,2));
            
            N = [0];
            
            obj.K_lqr = lqr(plant,Q,R,N);
        end
        
        function plotTrackingError(obj)
            t = tiledlayout(2,1);
            
            nexttile
            
            [cum_err, t, norm_err, ~] = trackingError(obj, obj.SimOut);
            cum_errors(1) = cum_err;
            p = plot(t,norm_err);
            names(1) = ["Nonlinear"];
            
            
            if ~isempty(obj.SimOutLinear)
                [cum_err_lin, t_lin, norm_err_lin, ~] = trackingError(obj, obj.SimOutLinear);
                cum_errors(2) = cum_err_lin;
                hold on
                pl = plot(t_lin, norm_err_lin);
                names(2) = "Linear";
                hold off
            end
            
            legend(names)
            title("Distance Error Over Time")
            xlabel("$$t$$", 'Interpreter', 'latex')
            ylabel("$$d$$ (m)", 'Interpreter', 'latex')
            
            nexttile
            
            barh(cum_errors)
            title("Integrated Error")
            yticklabels(names)
            
        end
                    
        function [cum_err, t, norm_err, err] = trackingError(obj, so)
            if nargin == 1
                so = obj.SimOut;
            end
            
            t = so.tout;
            target_trans = get(so.yout, 'r_out').Values.Data;
            trans = get(so.yout, 'y_out').Values.Data(:, obj.QR.BM.I.x.p);
            
            err = target_trans - trans;
            norm_err = vecnorm(err, 2, 2);
            cum_err = trapz(t,norm_err);
        end
        
        function setVariant(obj, mde)
            switch mde
                case "Linear"
                    set_param(obj.PlantSubsystem,'LabelModeActiveChoice', 'Combined')
                case "Nonlinear"
                    set_param(obj.PlantSubsystem,'LabelModeActiveChoice', 'Individual')
                    set_param(obj.PowertrainSubsystem, 'LabelModeActiveChoice', 'Nonlinear')
                    set_param(obj.PowertrainSubsystem, 'LabelModeActiveChoice', 'Nonlinear')
            end
            
        end
    end
end

