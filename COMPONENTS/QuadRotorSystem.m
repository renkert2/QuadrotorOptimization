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
        RefTrajTS timeseries
        DisturbanceTS timeseries
        
        SimOut QRSimOut % Cache for simulation output
        SimOutLinear QRSimOut
        SimOutDesc struct
    end
    
    properties
        K_lqr double
        SysSteadyState double
        EnableStop logical = true
        
        Variant string
        DistVar string
        RefTrajVar string
        
        LinearPlantModel
        LinearPowertrainModel
        LinearBodyModel
    end

    methods
        function obj = QuadRotorSystem(QR)
            obj.QR = QR;
        end
        
        function init(obj)
            load_system(obj.Name);
            obj.Wks = get_param(obj.Name, 'ModelWorkspace');
            assignin(obj.Wks, "QR_Object", obj);
            makeSimulinkModel(obj.QR.BM, obj.Name);
            makeSimulinkModel(obj.QR.PT.Model, obj.Name);
            obj.setSimOutDesc();
            
            obj.RefTraj = ReferenceTrajectory3D();
            obj.RefTraj.Lemniscate('a',10, 'PotatoChipHeight', 2);
            obj.RefTraj.init();
            obj.RefTraj.Speed = 5;
            obj.RefTraj.Cycles = 1;
            obj.RefTraj.setTimeSeries();
            
            obj.setLQR();
            
            obj.EnableStop = true;
            sys_time = 2*obj.QR.FlightTime; % Maximum simulation time. Stop System block handles convergence or battery draining
            set_param(obj.Name, 'StopTime', num2str(sys_time));
        end
        
        function ts = get.RefTrajTS(obj)
            ts = obj.RefTraj.TimeSeries;
        end
        
        function update(obj)
            update(obj.QR);
            obj.setLQR();
            sys_time = 2*obj.QR.FlightTime; % Maximum simulation time. Stop System block handles convergence or battery draining
            set_param(obj.Name, 'StopTime', num2str(sys_time));
        end
        
        function qrso = Simulate(obj, opts)
            arguments
                obj
                opts.Mode string = "Nonlinear"
            end
            
            obj.Variant = opts.Mode;
            
            simOut = sim(obj.Name);
            qrso = QRSimOut(simOut, obj.SimOutDesc);
            
            switch opts.Mode
                case "Nonlinear"
                    obj.SimOut = qrso;
                case "Linear"
                    obj.SimOutLinear = qrso;
            end
        end
        
        function ax = plot(obj, so, pltopts)
            arguments
                obj
                so = obj.SimOut
                pltopts cell = {}
            end
            y = get(so.Data.yout, 'y_out').Values.Data;
            r = get(so.Data.yout, 'r_out').Values.Data;
            
            if ~isempty(obj.SimOutLinear)
                sol = obj.SimOutLinear;
                yl = get(sol.Data.yout, 'y_out').Values.Data;
                arg = {y,yl};
                lgnd_names = ["Nonlinear Model Trajectory", "Linear Model Trajectory"];
            else
                arg = y;
                lgnd_names = ["Trajectory"];
            end
            ax = obj.QR.BM.plot(arg, 'RefTraj', r, 'TrajectoryNames', lgnd_names, pltopts{:});
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
            u_ss = obj.QR.BM.calcSteadyStateInput(x_ss, zeros(3,1), repmat(obj.QR.HoverSpeed(),4,1));
            d_ss = zeros(3,1);
            [Ab,Bb,~,Cb,Db,~] = CalcMatrices(obj.QR.BM.LinearModel,x_ss,u_ss,d_ss);
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
            xlabel("Time $$t$$", 'Interpreter', 'latex')
            ylabel("Norm of Error (m)", 'Interpreter', 'latex')
            
            nexttile
            
            barh(cum_errors)
            title("Integrated Error")
            yticklabels(names)
            xlabel("Accumulated Error ($$m*s$$)",'Interpreter', 'latex');
            
        end
                    
        function [cum_err, t, norm_err, err] = trackingError(obj, so)
            if nargin == 1
                so = obj.SimOut;
            end
            
            so = so.Data;
            t = so.tout;
            target_trans = get(so.yout, 'r_out').Values.Data;
            trans = get(so.yout, 'y_out').Values.Data(:, obj.QR.BM.I.x.p);
            
            err = target_trans - trans;
            norm_err = vecnorm(err, 2, 2);
            cum_err = trapz(t,norm_err);
        end
        
        function set.Variant(obj, mde)
            switch mde
                case "Linear"
                    set_param(obj.PlantSubsystem,'LabelModeActiveChoice', 'Combined')
                case "Nonlinear"
                    set_param(obj.PlantSubsystem,'LabelModeActiveChoice', 'Individual')
                    set_param(obj.PowertrainSubsystem, 'LabelModeActiveChoice', 'Nonlinear')
                    set_param(obj.PowertrainSubsystem, 'LabelModeActiveChoice', 'Nonlinear')
            end
        end
        
        function set.DistVar(obj, mde)
            assert(ismember(mde, ["TimeSeries", "Constant", "Zeros"]));
            obj.DistVar = mde;
            set_param([obj.Name '/DistVar'],'LabelModeActiveChoice', mde)
        end
        
        function set.RefTrajVar(obj, mde)
            assert(ismember(mde, ["TimeSeries", "Constant", "Zeros"]));
            obj.RefTrajVar = mde;
            set_param([obj.Name '/RefTrajVar'],'LabelModeActiveChoice', mde)
        end
    end
    
    methods (Hidden)% Helper Functions
        function s = setSimOutDesc(obj)
            s = struct();
            s.y_out = obj.QR.BM.OutputDescriptions;
            s.r_out = ["X Ref"; "Y Ref"; "Z Ref"];
            s.PT_out = obj.QR.PT.Model.OutputDescriptions;
            s.u_out = obj.QR.PT.Model.InputDescriptions;
            s.omega_out = obj.QR.PT.Model.OutputDescriptions(obj.QR.PT.SpeedOutputs_I);
            obj.SimOutDesc = s;
        end
    end
end

