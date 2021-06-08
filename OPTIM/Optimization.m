classdef Optimization < handle      
    properties
        QR QuadRotor 
        OptiVars (:,1) optiVar
        
        Objective OptimObjectives = "FlightTime" 
        
        CD ComponentData % ComponentDatabase
        
        SimulationBased logical = false;
        SimulationOpts cell = {'InterpolateTime', false, 'Timeout', 30} % Options pertaining to the simulation-based flight time calculation
        ReferenceTrajectory function_handle = @(t) (t>0) % Reference Trajectory used for Simulation-Based Optimization
        
    end
    
    properties (Hidden)
        X_prev double 
    end
    
    methods
        function obj = Optimization(qr, cd)
            if nargin == 0
                load QROpt.mat QROpt;
                qr = QROpt;
            end
            if nargin == 1
                load ComponentDatabase.mat ComponentDatabase;
                cd = ComponentDatabase;
            end
            
            obj.QR = qr;
            obj.CD = cd;
            obj.init();
        end
        
        function init(obj)
            batt = obj.QR.Battery;
            prop = obj.QR.Propeller;
            motor = obj.QR.Motor;
            
            % Set Optimization Variables
            OV(1) = optiVar(prop.D, prop.Fit.Boundary.X_lb(1),prop.Fit.Boundary.X_ub(1));
            OV(2) = optiVar(prop.P, prop.Fit.Boundary.X_lb(2), prop.Fit.Boundary.X_ub(2));            
            OV(3) = optiVar(batt.N_s, batt.Fit.Boundary.X_lb(1),batt.Fit.Boundary.X_ub(1), 'Enabled', true); % Typically voltage is selected to highest possible value           
            OV(4) = optiVar(batt.Q, batt.Fit.Boundary.X_lb(2),batt.Fit.Boundary.X_ub(2));
            OV(5) = optiVar(motor.kV, motor.Fit.Boundary.X_lb(1), motor.Fit.Boundary.X_ub(1));
            OV(6) = optiVar(motor.Rm, motor.Fit.Boundary.X_lb(2), motor.Fit.Boundary.X_ub(2));

            obj.OptiVars = OV';
            
            % Enable Dependencies
            batt.R_s.Dependent = true;
            batt.Mass.Dependent = true;
            motor.D.Dependent = true;
            motor.Mass.Dependent = true;
            prop.k_P.Dependent = true;
            prop.k_T.Dependent = true;
            prop.Mass.Dependent = true;
        end
        
        function main(obj)
            oo = obj.Optimize();
            N_max = struct('Battery', 3, 'PMSMMotor', 3, 'Propeller', 3);
            
%             batt_weights = struct('N_s', 1, 'Q', 1);
%             prop_weights = struct('D', 1, 'P', 1);
%             mot_weights = struct('kV', 1, 'Rm', 1);

            
            % Combine paramVals and G, assign to WeightStruct
            
            
           % WeightStruct = struct('Battery', batt_weights, 'PMSMMotor', mot_weights, 'Propeller', prop_weights);
            so = searchNearest(obj, oo, N_max, WeightStruct);
        end
        
        function [OO] = Optimize(obj, opts)
            arguments
                obj
                opts.DiffMinChange = 1e-4
                opts.OptimizationOpts cell = {}
                opts.OptimizationOutput logical = true
                opts.InitializeFromValue logical = false
                opts.CaptureState logical = true
                opts.CheckPrevious logical = true
                opts.RestoreDependencies logical = true % Ensure parameters with dependencies are reset to their default values before continuous optimization
            end
            
            optimopts = optimoptions('fmincon', 'Algorithm', 'sqp');
            if obj.SimulationBased
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
            
            if opts.RestoreDependencies
                restoreDependentDefault(obj.QR.Params);
            end
            
            lb = LB(obj.OptiVars);
            ub = UB(obj.OptiVars);
            
            OO = OptimOutput();
            OO.Objective = obj.Objective;
            
            qr_con_in_cache = obj.QR.ConstrainInput; % Save previous state of QR.ConstrainInput
            obj.QR.ConstrainInput = false; % Hand input constraint to optimization solver
            
            [X_opt_s, f_opt, OO.exitflag, ~, OO.lambda, grad_s, hessian_s] = fmincon(@objfun_local ,x0, [], [], [], [], lb, ub, @nlcon_local, optimopts);
            F_opt = obj.processF(f_opt); % Transform objective function output to desired output
            OO.F_opt = F_opt;
            
            s = 1./vertcat(obj.OptiVars.scaleFactors);
            S = diag(s);
            OO.grad = S*grad_s; % Denominator Layout Notation
            OO.hessian = S*hessian_s*S;
            
            % Specify Descriptions for constraints
            lambda_desc = struct();
            lambda_desc.ineqnonlin = ["Battery", "Propeller", "Motor", "Input"]; % Hardcoded for now.  Would be good to have Constraint objects at some point
            OO.lambdaDesc = lambda_desc;
                
            obj.QR.ConstrainInput = qr_con_in_cache; 
         
            % Set Current Values to Optimal Value in OptiVars
            setVals(obj.OptiVars, X_opt_s);
            obj.updateQR(true);
            OO.X_opt = unscale(obj.OptiVars);
            
            % Get optimal ParamVals to load later or for discrete search
            OO.ParamVals = getValues(vertcat(obj.OptiVars.Child));
            
            if opts.CaptureState
                OO.PerformanceData = obj.QR.PerformanceData;
                OO.DesignData = obj.QR.DesignData;
            end
            
            function f = objfun_local(X_s)
                setVals(obj.OptiVars, X_s);
                try
                    obj.updateQR(opts.CheckPrevious);
                    f = objfun(obj, true);
                catch
                    f = objfun(obj, false);
                end
            end
            
            function [c,ceq] = nlcon_local(X_s)
                setVals(obj.OptiVars, X_s);
                try
                    obj.updateQR(opts.CheckPrevious);
                    [c,ceq] = nlcon(obj, true);
                catch
                    [c,ceq] = nlcon(obj, false);
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
        
        function so = searchNearestBrute(obj, oo, N_max_search, N_max_comb, opts)
            % CAUTION: This only works if all components of the same Type
            % have the same parameter values.  Need to add additional 
            % Functionality if there are two components of the same type with 
            % different parameter values.  
            
            arguments
                obj
                oo OptimOutput
                N_max_search = inf
                N_max_comb (1,1) double = inf
                opts.EnforceBounds logical = true
                opts.Display logical = false
                opts.Plot logical = false
            end
            
            target = oo.ParamVals;

            % Get Initial component sets based on distance from optimal
            % point
            if opts.EnforceBounds
                lb = vertcat(obj.OptiVars.lb);
                ub = vertcat(obj.OptiVars.ub);
            else
                lb = [];
                ub = [];
            end
            [cd,~,comp_names] = filterNearest(obj.CD, target, N_max_search, 'LB', lb, 'UB', ub);

            % Cache QuadRotor Parameters
            loadValues(obj.QR.Params, target);
            oopv_cache = getValues(obj.QR.Params);
            
            d = cell(size(cd));
            cd_sorted = cell(size(cd));
            for i = 1:numel(d)
                comps = cd{i};
                N_comps = numel(comps);
                d_objective_comp = NaN(N_comps, 1);
                for j = 1:N_comps
                    % Evaluate Objective
                    [fval, ~] = evalCombination(obj, comps(j), opts.Display);
                    % Reset QR
                    loadValues(obj.QR.Params, oopv_cache);
                    d_objective_comp(j) = fval;
                end
                [d_objective_comp_sorted, I] = sort(d_objective_comp);
                d{i} = d_objective_comp_sorted;
                cd_sorted{i} = comps(I); % Reassign components to cd in the new order
            end
            
            % Apply second N_max here if necessary to reduce number of
            % combinations to evaluate
            
            [comb_array,comb_I] = combinations(cd_sorted{:});
            
            sz = size(comb_array);
            
            
            % comb_d: matrix of size comb_array corresponding to distance
            % of component from target
            comb_d = NaN(sz);
            for i_col = 1:sz(2)
                col = d{i_col};
                comb_d(:,i_col) = col(comb_I(:,i_col));
            end
            
            % Sort component combinations by mean objective function value
            comb_d_norm = mean(comb_d,2);
            [sorted_d_norm, I_d] = sort(comb_d_norm);
            
            comb_array = comb_array(I_d, :);
            comb_d = comb_d(I_d, :);
            comb_I = comb_I(I_d, :);
            
            % Apply N_max_comb to restrict total number of combinations
            N_combs = min(size(comb_array, 1), N_max_comb);
            R = 1:N_combs;
            comb_array = comb_array(R,:);
            comb_d = comb_d(R,:);
            comb_I = comb_I(R,:);
            
            
            if opts.Plot
                figure('Name', 'Nearest Neighbor Search')
                
                % Objective Function Plot
                ax_f = subplot(2,1,1);
                an_f = animatedline(ax_f);
                title("Objective Function")
                ylabel('f')
                
                ylim=get(ax_f,'ylim');
                xlim=get(ax_f,'xlim');
                opt_str = sprintf("Current Optimal:\n Iteration: %d \n F: %f",0,0);
                opt_text = text(xlim(1),ylim(2),opt_str, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
                
                hold on
                ax_optpoint = plot(ax_f, xlim(1), ylim(1), '.r', 'MarkerSize', 20);
                hold off
                
                % Distance Plot
                ax_d = subplot(2,1,2);
                co = colororder; % Gets default plot colors as rgb matrix
                for i = 1:sz(2)
                    color = co(i,:);
                    an_d(i) = animatedline(ax_d, 'DisplayName', comp_names(i), 'Color', color);
                end
                an_d_norm = animatedline(ax_d, 'DisplayName', "Average");
                
                title("Individual Objective Function Values")
                ylabel('d');
                xlabel("Iteration")
                legend
            end

            fvals = NaN(N_combs, 1);
            for i = 1:N_combs
               comb = comb_array(i,:); % Array of ComponentData objects
               
               if opts.Display
                    fprintf("Evaluating Configuration: %d\n", i)
               end
               
               [fvals(i,1), ~] = evalCombination(obj, comb, opts.Display);
               
               % update optimal configuration
               [opt_fval, opt_i] = min(fvals);  
               opt_comb = comb_array(opt_i,:);
                if opts.Display
                    fprintf("Current Optimal: I = %d, Objective Function: %f\n", opt_i, opt_fval);
                end 
               
               
               if opts.Plot
                   % Update Points
                   addpoints(an_f, i, fvals(i,1));
                   for j = 1:sz(2)
                       addpoints(an_d(j), i, comb_d(i,j))
                   end
                   addpoints(an_d_norm, i, sorted_d_norm(i));
                   
                   % Update Optimal Point
                   set(ax_optpoint, 'XData', opt_i, 'YData', opt_fval);
                   
                   % Update Optimal
                   ylim=get(ax_f,'ylim');
                   xlim=get(ax_f,'xlim');
                   set(opt_text, "Position", [xlim(1), ylim(2)]);
                   set(opt_text, "String", sprintf("Current Optimal:\n Iteration: %d \n F: %f",opt_i,opt_fval));
                   
                   drawnow
               end

            end 
            
            % Sort component configurations from best to worst
            [sorted_fvals,I] = sort(fvals, 'ascend');
            sorted_combs = comb_array(I,:); % Component combinations sorted by objective
            sorted_distances = comb_d(I,:);
            sorted_component_indices = comb_I(I,:);
            normalized_distances = sorted_d_norm(I,:);
            
            % Set QuadRotor to Optimal Configuration
            if opts.Display
                disp("Final Configuration: ")
            end
            [~,pmod] = evalCombination(obj, opt_comb, opts.Display);
            % All of pmod currently independent.
            
            sorted_FVals = processF(obj, sorted_fvals);
            
            % Package Output
            so = SearchOutput();
            so.Objective = obj.Objective;
            so.OptimalConfiguration = opt_comb;
            so.OptimalFVal = opt_fval;
            so.OptimalIteration = opt_i;
            so.SortedConfigurations = sorted_combs;
            so.SortedFVals = sorted_FVals;
            so.SortedDistances = sorted_distances;
            so.SortedComponentIndices = sorted_component_indices;
            so.NormalizedDistances = normalized_distances;
            so.ModifiedParameters = pmod;
            so.ComponentNames = comp_names;
            so.DistanceMode = "Norm";
            
            function [fval, pmod] = evalCombination(obj, comb, dispflag)
                cpv = vertcat(comb.Data);
                
                % Load Values into Parameters
                pmod = loadValues(obj.QR.Params, cpv);
                
                % Cache Dependency of Modified Parameters and make all
                % Independent
                setDependentTemporary(pmod, false); % Makes each element of pmod independent
                
                % Update QuadRotor
                try
                    updateQR(obj, false);
                    valid = true;
                catch
                    valid = false;
                end
                
                % Check Constraints
                if valid
                    [c,~] = nlcon(obj, valid);
                    if any(c > 0)
                        valid = false;
                    end
                end
                
                % Evaluate and Store Objective Function
                fval = objfun(obj, valid);
                
                               
               % Restore Dependency of Modified Parameters before returning
                restoreDependentDefault(pmod);
                
                if dispflag
                    disp("Configuration:");
                    disp(summaryTable(comb))
                    fprintf("Objective Function Value: %f\n\n", fval);
                end 
            end
        end
        
        function so = searchNearest(obj, oo, N_max, opts)
            % CAUTION: This only works if all components of the same Type
            % have the same parameter values.  Need to add additional 
            % Functionality if there are two components of the same type with 
            % different parameter values.  
            
            arguments
                obj
                oo OptimOutput
                N_max = inf
                opts.DistanceMode string = "Norm" % Options: "Norm", "WeightedNorm", "TaylorSeries"
                opts.Weights = []
                opts.Tolerance = []
                opts.UseGradient logical = true
                opts.UseHessian logical = true
                opts.Display logical = false
                opts.Plot logical = false
            end
            
            target = oo.ParamVals;
            
            grad = [];
            hessian = [];
            if opts.DistanceMode == "TaylorSeries"
                if opts.UseGradient
                    grad = oo.grad; % Scale the gradient by target values
                end
                if opts.UseHessian
                    hessian = oo.hessian; % Scale the hessian by target values
                end
            end
            
            [cd,d,comp_names] = filterNearest(obj.CD, target, N_max, 'DistanceMode', opts.DistanceMode, 'Weights', opts.Weights, 'Gradient', grad, 'Hessian', hessian, 'Tolerance', opts.Tolerance);

            [comb_array,comb_I] = combinations(cd{:});
            
            sz = size(comb_array);
            N_combs = sz(1);
            
            % comb_d: matrix of size comb_array corresponding to distance
            % of component from target
            comb_d = NaN(sz);
            for i_col = 1:sz(2)
                col = d{i_col};
                comb_d(:,i_col) = col(comb_I(:,i_col));
            end
            
            % Sort component combinations by norm of distances from target
            switch opts.DistanceMode
                case "Norm"
                    comb_d_norm = vecnorm(comb_d, 2, 2);
                case "WeightedNorm"
                    comb_d_norm = vecnorm(comb_d, 2, 2);
                case "TaylorSeries"
                    comb_d_norm = sum(comb_d, 2);
            end
            
            [sorted_d_norm, I_d] = sort(comb_d_norm);
            
            comb_array = comb_array(I_d, :);
            comb_d = comb_d(I_d, :);
            
            if opts.Plot
                figure('Name', 'Nearest Neighbor Search')
                
                % Objective Function Plot
                ax_f = subplot(2,1,1);
                an_f = animatedline(ax_f);
                title("Objective Function")
                ylabel('f')
                
                % Distance Plot
                ax_d = subplot(2,1,2);
                co = colororder; % Gets default plot colors as rgb matrix
                for i = 1:sz(2)
                    color = co(i,:);
                    an_d(i) = animatedline(ax_d, 'DisplayName', comp_names(i), 'Color', color);
                end
                an_d_norm = animatedline(ax_d, 'DisplayName', "Norm Distance");
                
                title("Component Distance")
                ylabel('d');
                xlabel("Iteration")
                legend
            end

            fvals = NaN(N_combs, 1);
            for i = 1:N_combs
               comb = comb_array(i,:); % Array of ComponentData objects
               
               if opts.Display
                    fprintf("Evaluating Configuration: %d\n", i)
               end
               
               [fvals(i,1), ~] = evalCombination(obj, comb, opts.Display);
               
               if opts.Plot
                   addpoints(an_f, i, fvals(i,1));
                   for j = 1:sz(2)
                       addpoints(an_d(j), i, comb_d(i,j))
                   end
                   addpoints(an_d_norm, i, sorted_d_norm(i));
                   drawnow
               end

            end 
            
            % Sort component configurations from best to worst
            [sorted_fvals,I] = sort(fvals, 'ascend');
            sorted_combs = comb_array(I,:); % Component combinations sorted by objective
            sorted_distances = comb_d(I,:);
            normalized_distances = sorted_d_norm(I_d,:);
            
            % Set QuadRotor to Optimal Configuration
            if opts.Display
                disp("Optimal Configuration: ")
            end
            [~,pmod] = evalCombination(obj, sorted_combs(1,:), opts.Display);
            % All of pmod currently independent.
            
            sorted_FVals = processF(obj, sorted_fvals);
            
            % Package Output
            so = SearchOutput();
            so.Objective = obj.Objective;
            so.SortedComponentSets = sorted_combs;
            so.SortedFVals = sorted_FVals;
            so.SortedDistances = sorted_distances;
            so.NormalizedDistances = normalized_distances;
            so.ModifiedParameters = pmod;
            so.ComponentNames = comp_names;
            so.DistanceMode = opts.DistanceMode;
            so.Weights = opts.Weights;
            
            function [fval, pmod] = evalCombination(obj, comb, dispflag)
                cpv = vertcat(comb.Data);
                
                % Load Values into Parameters
                pmod = loadValues(obj.QR.Params, cpv);
                
                % Cache Dependency of Modified Parameters and make all
                % Independent
                setDependentTemporary(pmod, false); % Makes each element of pmod independent
                
                % Update QuadRotor
                try
                    updateQR(obj, false);
                    valid = true;
                catch
                    valid = false;
                end
                
                % Check Constraints
                if valid
                    [c,~] = nlcon(obj, valid);
                    if any(c > 0)
                        valid = false;
                    end
                end
                
                % Evaluate and Store Objective Function
                fval = objfun(obj, valid);
                
                               
               % Restore Dependency of Modified Parameters before returning
                restoreDependentDefault(pmod);
                
                if dispflag
                    disp("Configuration:");
                    disp(summaryTable(comb))
                    fprintf("Objective Function Value: %f\n\n", fval);
                end 
            end
        end
        
        function f = objfun(obj, success_flag)
            if success_flag
                switch obj.Objective
                    case "FlightTime"
                        f = -flightTime();
                    case "Range"
                        f = -obj.QR.Range();
                end
            else
                f = NaN;
            end
            
            function ft = flightTime()
                if obj.SimulationBased
                    obj.QR.calcControllerGains;
                    ft = obj.QR.flightTime(obj.ReferenceTrajectory,'SimulationBased',true, obj.SimulationOpts{:});
                else
                    ft = obj.QR.flightTime('SimulationBased',false);
                end
            end
            % implement objective function scaling at some point
        end
        
        function F = processF(obj,f)
            % Processes objective function value and returns desired value
            % Implement objective function scaling at some point
            % Make sure this stays vectorizable
            switch obj.Objective
                case "FlightTime"
                    F = -f;
                case "Range"
                    F = -f;
            end
        end
        
        function [c,ceq] = nlcon(obj, success_flag)
            if success_flag
                c_input = obj.QR.SS_QAve.u - 1;
            else
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
            ceq = [];
        end
        
        function updateQR(obj, check_prev)
            if check_prev
                X = [obj.OptiVars.Value];
                if ~isempty(obj.X_prev) && all(X == obj.X_prev)
                    return % Only update variables if the design variables have changed value
                end
                obj.X_prev = X;
            end
            
            obj.QR.update();
        end
    end
end

