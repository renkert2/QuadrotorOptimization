classdef Optimization < handle      
    properties
        QR QuadRotor
        
        OptiVars (:,1) optiVar
        X_prev double 
    end
    
    methods
        function obj = Optimization(qr)
            if nargin == 0
                load QROpt.mat QROpt;
                qr = QROpt;
            end
            obj.QR = qr;
            obj.init();
        end
        
        function init(obj)
            batt = obj.QR.Battery;
            prop = obj.QR.Propeller;
            motor = obj.QR.Motor;
            
            % Set Optimization Variables
            OV(1) = optiVar(prop.D, [], prop.Fit.Boundary.X_lb(1),prop.Fit.Boundary.X_ub(1));
            OV(2) = optiVar(prop.P, [], prop.Fit.Boundary.X_lb(2), prop.Fit.Boundary.X_ub(2));            
            OV(3) = optiVar(batt.N_s, [], batt.Fit.Boundary.X_lb(1),batt.Fit.Boundary.X_ub(1), 'Enabled', true); % Typically voltage is selected to highest possible value           
            OV(4) = optiVar(batt.Q, [], batt.Fit.Boundary.X_lb(2),batt.Fit.Boundary.X_ub(2));
            OV(5) = optiVar(motor.kV, [], motor.Fit.Boundary.X_lb(1), motor.Fit.Boundary.X_ub(1));
            OV(6) = optiVar(motor.Rm, [], motor.Fit.Boundary.X_lb(2), motor.Fit.Boundary.X_ub(2));

            obj.OptiVars = OV';
        end

        function [X_opt_s, F_opt, OO] = Optimize(obj, objective, r, opts)
            arguments
                obj
                objective OptimObjectives = "FlightTime"
                r = @(t) (t>0)
                opts.SimulationBased = false
                opts.DiffMinChange = 1e-4
                opts.SimulationOpts cell = {'InterpolateTime', false, 'Timeout', 30} % Options pertaining to the simulation-based flight time calculation
                opts.OptimizationOpts cell = {}
                opts.OptimizationOutput logical = true
                opts.InitializeFromValue logical = false
                opts.CaptureState logical = true
            end
            
            optimopts = optimoptions('fmincon', 'Algorithm', 'sqp');
            if opts.SimulationBased
                optimopts = optimoptions(optimopts, 'DiffMinChange', opts.DiffMinChange);
            end
            if opts.OptimizationOutput
                optimopts = optimoptions(optimopts, 'Display', 'iter-detailed', 'PlotFcn', {@optimplotx,@optimplotfval,@optimplotfirstorderopt});
            end
            optimopts = optimoptions(optimopts, opts.OptimizationOpts{:});
            
            if opts.InitializeFromValue
                x0 = .75*scale(obj.OptiVars) + .25*X0(obj.OptiVars);
            else
                x0 = X0(obj.OptiVars);
            end
            lb = LB(obj.OptiVars);
            ub = UB(obj.OptiVars);
            
            OO = OptimOutput();
            OO.Objective = objective;
            
            qr_con_in_cache = obj.QR.ConstrainInput; % Save previous state of QR.ConstrainInput
            obj.QR.ConstrainInput = false; % Hand input constraint to optimization solver
            
            [X_opt_s, f_opt, OO.exitflag, ~, OO.lambda, OO.grad, OO.hessian] = fmincon(@objfun ,x0, [], [], [], [], lb, ub, @(X_s) nlcon(X_s), optimopts);
            F_opt = processF(f_opt); % Transform objective function output to desired output
            OO.F_opt = F_opt;
            
            % Specify Descriptions for constraints
            f = fieldnames(OO.lambda)';
            f{2,1} = {};
            lambda_desc = struct();
            lambda_desc.ineqnonlin = ["Battery", "Propeller", "Motor", "Input"]; % Hardcoded for now.  Would be good to have Constraint objects at some point
            OO.lambdaDesc = lambda_desc;
            
            obj.QR.ConstrainInput = qr_con_in_cache; 
         
            % Set Current Values to Optimal Value in OptiVars
            setVals(obj.OptiVars, X_opt_s);
            obj.updateQR();
            OO.X_opt = unscale(obj.OptiVars);
            
            if opts.CaptureState
                OO.PerformanceData = obj.QR.PerformanceData;
                OO.DesignData = obj.QR.DesignData;
            end
            
            function f = objfun(X_s)
                setVals(obj.OptiVars, X_s);
                try
                    obj.updateQR();
                catch
                    f = NaN;
                    return
                end
                
                switch objective
                    case "FlightTime"
                        f = -flightTime();
                    case "Range"
                        f = -obj.QR.Range();
                end
                % implement objective function scaling at some point
            end
            
            function [c,ceq] = nlcon(X_s)
                ceq = [];
                setVals(obj.OptiVars, X_s);
                try
                    obj.updateQR();
                    
                    % QR Dependent Constraints
                    c_input = obj.QR.SS_QAve.u - 1;
                catch
                    c_input = NaN;
                end
                
                % Boundary Constraints
                batt = obj.QR.Battery;
                c_batt = distToBoundary(batt.Fit.Boundary, [batt.Fit.Inputs.Value]');
                
                prop = obj.QR.Propeller;
                c_prop = distToBoundary(prop.Fit.Boundary, [prop.Fit.Inputs.Value]');
                
                motor = obj.QR.Motor;
                c_motor = distToBoundary(motor.Fit.Boundary, [motor.Fit.Inputs.Value]');
                
                c = [c_batt; c_prop; c_motor; c_input];
            end
            
            function F = processF(f)
                % Implement objective function scaling at some point              
                switch objective
                    case "FlightTime"
                        F = -f;
                    case "Range"
                        F = -f;
                end
            end
            
            function ft = flightTime()
                if opts.SimulationBased
                    obj.QR.calcControllerGains;
                    ft = obj.QR.flightTime(r,'SimulationBased',true, opts.FlightTimeOpts{:});
                else
                    ft = obj.QR.flightTime('SimulationBased',false);
                end
            end
        end
        
        function so = sweep(obj, vars, n, opts)
            % X is the vector of design variable values being swept across.
            % - 1xn for 1 design var and nxnx2 meshgrid for 2 design vars
            % ft is a vector or grid of flight times
            % X_opt is the optimal design point, f_opt is the optimal
            % flight time
            % I are the indices 1xv or (i;j)xv corresponding to points where a
            % valid flight time was obtained. PD and DD are 1xv vectors of 
            % PerformanceData and DesignData objects evaluated at the valid
            % points
            arguments
                obj
                vars
                n double
                opts.Objective = "FlightTime"
                opts.ConstraintFunction = []
                opts.ReverseSearch logical = false
                opts.InitializeFromValue logical = true
            end
            
            N_vars = numel(vars);
            assert(N_vars <=2 && N_vars ~= 0, "Choose 1 or 2 optimization variables for sweep")
            if ~isa(vars, 'optiVar')
                try
                    vars = string(vars);
                    vars = obj.OptiVars.get(vars);
                catch
                    error('vars argument must be optiVar objects or convertable to strings');
                end
            end
            
            obj.OptiVars.reset();
            % Obtain baseline optimal point
            [~,F_opt] = Optimize(obj, opts.Objective, 'OptimizationOutput', false, 'OptimizationOpts', {'Display', 'none'});
            X_opt = vertcat(vars.Value);
            
            for i = 1:N_vars
                % Each variable in the sweep is fixed at a point and is 
                % no longer optimized
                vars(i).Enabled = false;
            end
            
           switch N_vars
               case 1
                   X = linspace(vars, n);
                   if opts.ReverseSearch
                       X = fliplr(X);
                   end
                   
                   F = NaN(size(X));
                   valid_cnt = 0;
                   for i = 1:numel(X)
                       vars.Value = X(i);
                       [F(i), pd, dd, OO(i)] = optiWrapper();
                       if ~isnan(F(i))
                           valid_cnt = valid_cnt + 1;
                           I(1,valid_cnt) = i;
                           PD(1,valid_cnt) = pd;
                           DD(1,valid_cnt) = dd;
                           msg = sprintf("Point: %f F: %f", X(i), F(i));
                       else
                           msg = sprintf("Point: %f F: NaN", X(i));
                       end
                       disp(msg)
                   end
                   
               case 2
                   x = linspace(vars(1),n);
                   y = linspace(vars(2),n);
                   if opts.ReverseSearch
                       x = fliplr(x);
                       y = fliplr(y);
                   end
                   if ~isempty(opts.ConstraintFunction)
                       const_fun_flag = true;
                   else
                       const_fun_flag = false;
                   end
         
                   F = NaN(numel(y), numel(x));
                   valid_cnt = 0;
                   for i = 1:numel(x)
                       for j = 1:numel(y)
                           if const_fun_flag
                               is_valid = (opts.ConstraintFunction([x(i);y(j)])<=0);
                           else
                               is_valid = true;
                           end
                           
                           if is_valid
                               vars(1).Value = x(i);
                               vars(2).Value = y(j);
                               [F(j,i), pd, dd, OO(j,i)] = optiWrapper();
                               if ~isnan(F(j,i))
                                   valid_cnt = valid_cnt + 1;
                                   I(:,valid_cnt) = [j;i];
                                   PD(1,valid_cnt) = pd;
                                   DD(1,valid_cnt) = dd;
                                   msg = sprintf("Point: (%f, %f) F: %f", x(i), y(j), F(j,i));
                               else
                                   msg = sprintf("Point: (%f, %f) F: NaN", x(i), y(j));
                               end
                           else
                               F(j,i) = NaN;
                               msg = sprintf("Point: (%f, %f) F: Invalid Point", x(i), y(j));
                           end
                           disp(msg)
                       end
                   end
                   [X(:,:,1), X(:,:,2)] = meshgrid(x,y);
           end
           
           % Export Sweep Object
           so = SweepData();
           so.Vars = vars;
           so.N_vars = N_vars;
           so.X = X;
           so.F = F;
           so.X_opt = X_opt;
           so.F_opt = F_opt;
           so.I = I;
           so.PD = PD;
           so.DD = DD;
           so.OO = OO;
           
           % Clean Up
           obj.OptiVars.reset();
           
            function [F, pd, dd, oo] = optiWrapper()
                try
                    [~,F,oo] = Optimize(obj, opts.Objective, 'OptimizationOutput', false, 'OptimizationOpts', {'Display', 'none'}, 'InitializeFromValue',opts.InitializeFromValue);
                    pd = oo.PerformanceData;
                    dd = oo.DesignData;
                catch
                    F = NaN;
                    oo = OptimOutput();
                    oo.exitflag = -4;
                    pd = NaN;
                    dd = NaN;
                end
            end
        end
        
        function updateQR(obj)
            X = [obj.OptiVars.Value];
            if ~isempty(obj.X_prev) && all(X == obj.X_prev)
                return % Only update variables if the design variables have changed value
            end
            
            obj.QR.update();
            obj.X_prev = X;
        end
    end
end

