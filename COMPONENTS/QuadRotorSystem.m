classdef QuadRotorSystem < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        Name char = 'QuadRotor_Simulink'
    end
    
    properties
        QR QuadRotor
        Wks Simulink.ModelWorkspace
        
        RefTraj ReferenceTrajectory3D
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
        
        
        function [t,y,r] = Simulate(obj)
            simOut = sim(obj.Name);
            t = simOut.tout;
            y = get(simOut.yout, 'y_out').Values.Data;
            r = get(simOut.yout, 'r_out').Values.Data;
        end
        
        function ax = plot(obj, varargin)
            ax = obj.QR.BM.plot(varargin{:}, 'RefTraj', obj.RefTraj);
        end
        
        function ax = animate(obj, varargin)
            ax = obj.QR.BM.animate(varargin{:}, 'RefTraj', obj.RefTraj);
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
    end
end

