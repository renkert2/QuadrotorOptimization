classdef PowerTrain_OptiController < handle
    %POWERTRAIN_OPTICONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        PT PowerTrain
    end
    
    properties(SetAccess = private)
        Optimizer
        
        N_t double % Number of time steps
        delta_t double % Length of time step
    end
    
    methods
        function obj = PowerTrain_OptiController(PT)
            %POWERTRAIN_OPTICONTROLLER Construct an instance of this class
            %   Detailed explanation goes here
            if nargin 
                obj.PT = PT;
            end
        end
        
        function formulate(obj, N_t, delta_t, opts)
            arguments
                obj
                N_t double
                delta_t double
                opts.SolverOpts cell = {'solver','quadprog'}
            end
            yalmip 'clear';
            obj.N_t = N_t;
            obj.delta_t = delta_t;
            
            %% Formulate Optimization (YALMIP)
            Nx = obj.PT.Model.Nx;
            Nu = obj.PT.Model.Nu;
            Ny = numel(obj.PT.SpeedOutputs_I);
            
            disp("Initializing Variables");
            % Constants
            C_d = double(obj.PT.Model.LinearModel.C_sym(obj.PT.SpeedOutputs_I, :));

            % Parameters
            A_d = sdpvar(Nx, Nx, 'full');
            B_d = sdpvar(Nx, Nu, 'full');
            X0 = sdpvar(Nx,1, 'full'); % Initial State
            q_f = sdpvar(1); % Min Final State of Charge

            % Inputs
            y_des = sdpvar(Ny, obj.N_t, 'full');
            
            % Decision Variables and Outputs
            x_ = sdpvar(Nx,obj.N_t,'full');
            u_ = sdpvar(Nu,obj.N_t - 1);
            y_ = C_d*x_;
            
            % Required and Output variables for Optimizer function call
            params = {A_d, B_d, X0, q_f, y_des};
            outputs = {x_, u_, y_};
            
            objs = 0;
            disp("Formulating Objectives")
            for k = 2:obj.N_t-1
                objs = objs + (1/2)*(C_d*x_(:,k) - y_des(:,k))'*(C_d*x_(:,k) - y_des(:,k));
            end
            
            % Initial Condition
            cons = [x_(:,1) == X0];
            disp("Formulating Constraints")
            for k = 1:obj.N_t-1
                % Dynamic Constraints
                cons = [cons, x_(:,k+1) == A_d*x_(:,k) + B_d*u_(:,k)];
                
                % Input Constraints
                cons = [cons, (zeros(Nu,1) <= u_(:,k) <= ones(Nu,1))];
            end
            
            % Final Condition
            cons = [cons, x_(1,end) >= q_f];

            % Make this an Optimizer object and store as a property in the future to avoid all the
            % overhead.
            opts = sdpsettings(opts.SolverOpts{:});
            disp("Building Optimizer")
            obj.Optimizer = optimizer(cons,objs,opts,params,outputs);
        end
        
        function [sol_x, sol_u, sol_y, errorcode] = Solve(obj, pt_state, y_des, opts)
            arguments
                obj
                pt_state PowerTrainState
                y_des double
                opts.X0 = []
                opts.MinFinalSOC = 0.2
            end
            Nx = obj.PT.Model.Nx;
            Nu = obj.PT.Model.Nu;
            Ny = numel(obj.PT.SpeedOutputs_I);
            %assert(all(size(y_des) == [N_y, obj.N_t]), 'Desired Input must be N_y x N_t');
            
            q_f = opts.MinFinalSOC;
            
            if isempty(opts.X0)
               X0 = [1; zeros(Nx-1,1)];
               X0(obj.PT.SpeedOutputs_I) = pt_state.RotorSpeed;
               X0(obj.PT.InductanceOutputs_I) = pt_state.MotorCurrent;
            else
                X0 = opts.X0;
            end
            
            [A,B,C,D] = CalcMatrices(obj.PT, pt_state);
            [A_d, B_d,~, ~] = LinearModel.Discretize(A,B,C,D,obj.delta_t);
            
            params = {A_d, B_d, X0, q_f, y_des};
            [sol, errorcode] = obj.Optimizer(params);
            sol_x = sol{1};
            sol_u = sol{2};
            sol_y = sol{3};
        end
    end
end

