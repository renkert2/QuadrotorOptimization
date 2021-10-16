classdef QuadRotorSystem < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        Name char = 'QuadRotor_Simulink_LQI_BackCalculation'
        PlantSubsystem char = 'QuadRotor_Simulink_LQI_BackCalculation/PlantSubsystem'
        CombinedLinearPlantModel char = 'QuadRotor_Simulink_LQI_BackCalculation/PlantSubsystem/CombinedLinearPlantModel'
        IndividualPlantModel char = 'QuadRotor_Simulink_LQI_BackCalculation/PlantSubsystem/IndividualPlantModel'
        PowertrainSubsystem char = 'QuadRotor_Simulink_LQI_BackCalculation/PlantSubsystem/IndividualPlantModel/PowertrainSubsystem'
        BodySubsystem char = 'QuadRotor_Simulink_LQI_BackCalculation/PlantSubsystem/IndividualPlantModel/BodySubsystem'
    end
    
    properties
        QR QuadRotor
        Wks Simulink.ModelWorkspace
        
        RefTraj ReferenceTrajectory3D = ReferenceTrajectory3D("Lemniscate")
        RefTrajTS timeseries
        DisturbanceTS timeseries
        
        SimOut QRSimOut % Cache for simulation output
        SimOutLinear QRSimOut
        SimOutDesc struct
    end
    
    properties
        Nx double
        Ny double
        Nu double

        K_lqr double
        K_lqi double
        K_sat double % Integrator Anti-Windup Gain
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
            obj.init();
        end
        
        function init(obj)
            load_system(obj.Name);
            obj.Wks = get_param(obj.Name, 'ModelWorkspace');
            assignin(obj.Wks, "QR_Object", obj);
            makeSimulinkModel(obj.QR.BM, obj.Name);
            makeSimulinkModel(obj.QR.PT.Model, obj.Name);
            obj.setSimOutDesc();

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
            obj.Simulate('Timeout',10);
        end
        
        function qrso = Simulate(obj, opts)
            arguments
                obj
                opts.Mode string = "Nonlinear"
                opts.Timeout double = inf
                opts.CaptureErrors string = "on"
            end
            
            obj.Variant = opts.Mode;
            
            simOut = sim(obj.Name, "Timeout", opts.Timeout, 'CaptureErrors', opts.CaptureErrors);
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
                so QRSimOut = obj.SimOut
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
            t = so.Data.tout;
            y = get(so.Data.yout, 'y_out').Values.Data;
            ax = obj.QR.BM.animate(t,y, 'RefTraj', obj.RefTraj);
        end
        
        function setLQR(obj, rho)
            arguments
                obj
                rho (1,1) double = 0.1
            end
            q_omega = 0;
            q_p = .01;
            q_v = .01;
            q_phi = 1;
            q_b_omega = 1;
            q_I = 10;
            k_sat = 0.01;
            
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
            
            comb = connect(PT,BM, 'u', {'W','y'});
            N_x = size(comb.A,1); % Number of states in combined system; quite messy
            N_u = size(comb.B,2);

            % Set Output to Position States
            I_x_p = 4 + obj.QR.BM.I.x.p;
            N_y = numel(I_x_p);
            C = zeros(N_y, N_x);
            for j = 1:N_y
                C(j,I_x_p(j)) = 1;
            end
            D = comb.D(1:size(C,1),:);
            plant = ss(comb.A, comb.B, C, D);
            obj.LinearPlantModel = plant;

            obj.Nx = N_x;
            obj.Ny = N_y;
            obj.Nu = N_u;

            %% Weighting Matrices
            Q_omega = q_omega*eye(4,4);
            Q_p = q_p*eye(3,3);
            Q_v = q_v*eye(3,3);
            Q_phi = q_phi*eye(3,3);
            Q_b_omega = q_b_omega*eye(3,3);

            %% LQR
            Q = blkdiag(Q_omega, Q_p, Q_v, Q_phi, Q_b_omega);
            R = rho*eye(N_u);
            N = zeros(N_x, N_u);
            
            obj.K_lqr = lqr(plant,Q,R,N);

            %% LQI - Track Position Reference
            Q_I = q_I*eye(3);
            Q = blkdiag(Q,Q_I);

            obj.K_lqi = lqi(plant,Q,R);

            %% Integrator Anti-Windup
            obj.K_sat = k_sat.*ones(3, 4);
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
            
            if isValid(so)
                so = so.Data;
                t = so.tout;
                target_trans = get(so.yout, 'r_out').Values.Data;
                trans = get(so.yout, 'y_out').Values.Data(:, obj.QR.BM.I.x.p);
                
                err = target_trans - trans;
                norm_err = vecnorm(err, 2, 2);
                cum_err = trapz(t,norm_err);
            else
                cum_err = NaN;
                t = NaN;
                norm_err = NaN;
                err = NaN;
            end
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

        function Am = getClosedLoopMatrix(obj,K)
            A = obj.LinearPlantModel.A;
            B = obj.LinearPlantModel.B;
            Am = (A - B*K);
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

