classdef Optimization < handle  
    properties
        QR QuadRotor

        propMassFit propMassFit
        propAeroFit propAeroFit
        motorFit motorFit
        
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
            load PF_Aero.mat PF_Aero
            obj.propAeroFit = PF_Aero;
            
            load PF_Mass.mat PF_Mass
            obj.propMassFit = PF_Mass;
            
            load MF_KDE.mat MF_KDE
            obj.motorFit = MF_KDE;
            
            batt = obj.QR.Battery;
            prop = obj.QR.Propeller;
            motor = obj.QR.Motor;
            
            % Set Optimization Variables
            OV(1) = optiVar("D", obj.propAeroFit.Boundary.X_mean(1), obj.propAeroFit.Boundary.X_lb(1),obj.propAeroFit.Boundary.X_ub(1));
            OV(1).Description = "Diameter";
            OV(1).Unit = "m";
            OV(1).Parent = prop;
            
            OV(2) = optiVar("P", obj.propAeroFit.Boundary.X_mean(2), obj.propAeroFit.Boundary.X_lb(2), obj.propAeroFit.Boundary.X_ub(2));
            OV(2).Description = "Pitch";
            OV(2).Unit = "m";
            OV(2).Parent = prop;
            
            OV(3) = optiVar("N_s", 6, 0.1, 12, 'Enabled', false); % Typically voltage is selected to highest possible value
            OV(3).Description = "Series Cells";
            OV(3).Parent = batt;
            
            OV(4) = optiVar("N_p", 1, 0.1, 20);
            OV(4).Description = "Parallel Cells";
            OV(4).Parent = batt;
            
            OV(5) = optiVar("kV", obj.motorFit.Boundary.X_mean(1), obj.motorFit.Boundary.X_lb(1), obj.motorFit.Boundary.X_ub(1));
            OV(5).Description = "Speed Constant";
            OV(5).Unit = "RPM/V";
            OV(5).Parent = motor;
            
            OV(6) = optiVar("Rm", obj.motorFit.Boundary.X_mean(2), obj.motorFit.Boundary.X_lb(2), obj.motorFit.Boundary.X_ub(2));
            OV(6).Description = "Phase Resistance";
            OV(6).Unit = "Ohm";
            OV(6).Parent = motor;

            obj.OptiVars = OV';
        end

        function [X_opt_s, F_opt, OO] = Optimize(obj, objective, r, opts)
            arguments
                obj
                objective OptimObjectives 
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
            
            [X_opt_s, f_opt, OO.exitflag, ~, OO.lambda, OO.grad, OO.hessian] = fmincon(@objfun ,x0, [], [], [], [], lb, ub, @nlcon, optimopts);
            F_opt = processF(f_opt); % Transform objective function output to desired output
            OO.F_opt = F_opt;
            
            obj.QR.ConstrainInput = qr_con_in_cache; 
         
            % Set Current Values to Optimal Value in OptiVars
            setVals(obj.OptiVars, X_opt_s);
            
            % Ensure the QuadRotor gets updated to the correct sym param vals
            obj.updateParamVals(XAll(obj.OptiVars));
            OO.X_opt = unscale(obj.OptiVars);
            
            if opts.CaptureState
                OO.PerformanceData = obj.QR.PerformanceData;
                OO.DesignData = obj.QR.DesignData;
            end
            
            function f = objfun(X_s)
                X = XAll(obj.OptiVars,X_s); % Unscale and return all
                
                try
                    obj.updateParamVals(X);
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
                
                X = XAll(obj.OptiVars,X_s); % Unscale and return all values
                
                try
                    obj.updateParamVals(X);
                catch
                    c = NaN(3,1);
                    return
                end
                
                % Boundary Objectives
                c_prop = distToBoundary(obj.propAeroFit.Boundary, X(find(obj.OptiVars, ["D", "P"])));
                c_motor = distToBoundary(obj.motorFit.Boundary, X(find(obj.OptiVars, ["kV", "Rm"])));
                
                % QR Dependent Objectives
                c_input = obj.QR.SS_QAve.u - 1;
                
                c = [c_prop; c_motor; c_input];
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
        
        function resetParamVals(obj)
           obj.updateParamVals(vertcat(obj.OptiVars.x0)); 
        end
        
        function updateParamVals(obj,X)
            if nargin == 1
                X = vertcat(obj.OptiVars.Value);
            end
            if ~isempty(obj.X_prev) && all(X == obj.X_prev)
                return % Only update the quadrotor if the design variables have changed value
            end
                
            %X_prop = [D;P]
            X_prop = X(find(obj.OptiVars, ["D", "P"]));
            D_prop = X_prop(1);
            P_prop = X_prop(2);
            [k_P_prop, k_T_prop] = calcPropCoeffs(obj.propAeroFit, X_prop);
            [M_prop,J_prop] = calcMassProps(obj.propMassFit, D_prop);
                   
            %X_batt = [N_p; N_s]
            X_batt = X(find(obj.OptiVars, ["N_s", "N_p"]));
            N_s_batt = X_batt(1);
            N_p_batt = X_batt(2);
            
            %X_motor = [kV; Rm]
            X_motor = X(find(obj.OptiVars, ["kV", "Rm"]));
            kV_motor = X_motor(1);
            Rm_motor = X_motor(2);
            [M_motor, J_motor, D_motor] = calcMotorProps(obj.motorFit, X_motor);
            
            % sym_params:
            %    D_prop
            %    J_motor
            %    J_prop
            %    K_t_motor
            %    M_motor
            %    M_prop
            %    N_p_batt
            %    N_s_batt
            %    Rm_motor
            %    k_P_prop
            %    k_T_prop
            QR = obj.QR;
            
            % Battery
            QR.Battery.N_p.Value = N_p_batt;
            QR.Battery.N_s.Value = N_s_batt;
            
            % Prop
            QR.Propeller.D.Value = D_prop;
            QR.Propeller.P.Value = P_prop;
            QR.Propeller.J.Value = J_prop;
            QR.Propeller.M.Value = M_prop;
            QR.Propeller.k_P.Value = k_P_prop;
            QR.Propeller.k_T.Value = k_T_prop;
            
            % Motor
            QR.Motor.J.Value = J_motor;
            QR.Motor.kV.Value = kV_motor;
            QR.Motor.M.Value = M_motor;
            QR.Motor.Rm.Value = Rm_motor;
            QR.Motor.D.Value = D_motor;

            obj.QR.update();
            obj.QR.SS_QAve = obj.QR.calcSteadyState();
            obj.X_prev = X;
        end
    end
end

