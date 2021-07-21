classdef Optimization < handle      
    properties
        QR QuadRotor 
        OptiVars (:,1) optiVar
        DependentParams (:,1) compParam
        
        Objective OptiFunctions.Parents.Function = OptiFunctions.FlightTime();
        Constraints OptiFunctions.Parents.Function = [OptiFunctions.BatteryBoundary; OptiFunctions.MotorBoundary; OptiFunctions.PropellerBoundary; OptiFunctions.InputConstraint]        
        CD ComponentData % ComponentDatabase 
    end
    
    properties (Hidden)
        X0Params compParamValue
        X_prev double 
    end
    
    methods
        function obj = Optimization(qr, cd)
            if nargin == 1
                load BatteryComponentData.mat BatteryComponentData;
                load PropellerComponentData.mat PropellerComponentData;
                load MotorComponentData.mat MotorComponentData;
                cd = [BatteryComponentData; PropellerComponentData; MotorComponentData];
            end
            
            obj.QR = qr;
            obj.X0Params = getValues(qr.Params);
            obj.CD = cd;

            obj.init();
        end
        
        function init(obj)
            batt = obj.QR.PT.Battery;
            prop = obj.QR.PT.Propeller;
            motor = obj.QR.PT.Motor;
            
            % Set Optimization Variables
            OV(1) = optiVar(prop.D, prop.Fit.Boundary.X_lb(1), min(prop.Fit.Boundary.X_ub(1), obj.QR.MaxPropDiameter));
            OV(2) = optiVar(prop.P, prop.Fit.Boundary.X_lb(2), prop.Fit.Boundary.X_ub(2));            
            OV(3) = optiVar(batt.N_s, batt.Fit.Boundary.X_lb(1),batt.Fit.Boundary.X_ub(1)); % Typically voltage is selected to highest possible value           
            OV(4) = optiVar(batt.Q, batt.Fit.Boundary.X_lb(2),batt.Fit.Boundary.X_ub(2));
            OV(5) = optiVar(motor.kV, motor.Fit.Boundary.X_lb(1), motor.Fit.Boundary.X_ub(1));
            OV(6) = optiVar(motor.Rm, motor.Fit.Boundary.X_lb(2), motor.Fit.Boundary.X_ub(2));

            obj.OptiVars = OV';
            
            % Enable Dependencies
            obj.DependentParams = [...
                batt.R_s;
                batt.Mass;
                batt.Price;
                motor.D;
                motor.Mass;
                motor.Price;
                prop.k_P;
                prop.k_T;
                prop.Mass;
                prop.Price;];
            storeDependentDefault(obj.DependentParams);
        end

        function [OO] = Optimize(obj, opts)
            arguments
                obj
                opts.SolverFunction = "fmincon"
                opts.OptimizationOpts cell = {}
                opts.OptimizationOutput logical = true
                opts.InitializeFromValue logical = false
                opts.CaptureState logical = false
                opts.CheckPrevious logical = true
                opts.Timer logical = true
            end
            
            if opts.Timer
                timer = tic;
            end
                        
            setDependent(obj.DependentParams, true);
            obj.updateQR(false);
            
            optimopts = optimoptions(opts.SolverFunction);
            switch opts.SolverFunction
                case "fmincon"
                    optimopts = optimoptions(optimopts, 'Algorithm', 'sqp');
                    if opts.OptimizationOutput
                        optimopts = optimoptions(optimopts, 'Display', 'iter-detailed', 'PlotFcn', {@optimplotx,@optimplotfval,@optimplotfirstorderopt});
                    end
                case "ga"
                    optimopts = optimoptions(optimopts, 'UseVectorized', false);
                    if opts.OptimizationOutput
                        optimopts = optimoptions(optimopts, 'Display', 'iter', 'PlotFcn', {@gaplotbestf,@gaplotbestindiv});
                    end
            end
            optimopts = optimoptions(optimopts, opts.OptimizationOpts{:});
            
            lb = LB(obj.OptiVars);
            ub = UB(obj.OptiVars);
            
            OO = OptimOutput();
            OO.Objective = class(obj.Objective);
            OO.SolverFunction = opts.SolverFunction;
            
            OO.F0 = obj.Objective.Value(obj.QR);
            qr_con_in_cache = obj.QR.PT.SimpleModel.ConstrainInput; % Save previous state of QR.ConstrainInput
            obj.QR.PT.SimpleModel.ConstrainInput = false; % Hand input constraint to optimization solver
            
            switch opts.SolverFunction
                case "fmincon"
                    if opts.InitializeFromValue
                        x0 = .75*scale(obj.OptiVars) + .25*X0(obj.OptiVars);
                    else
                        x0 = X0(obj.OptiVars);
                    end
                    OO.X0 = unscale(obj.OptiVars);
                    
                    [X_opt_s, f_opt, OO.exitflag, ~, OO.lambda, grad_s, hessian_s] = fmincon(@objfun_local ,x0, [], [], [], [], lb, ub, @nlcon_local, optimopts);
                    
                    s = 1./vertcat(obj.OptiVars.scaleFactors);
                    S = diag(s);
                    OO.grad = S*grad_s; % Denominator Layout Notation
                    OO.hessian = S*hessian_s*S;
                    
                    % Specify Descriptions for constraints
                    lambda_desc = struct();
                    lambda_desc.ineqnonlin = ["Battery", "Propeller", "Motor", "Input"]; % Hardcoded for now.  Would be good to have Constraint objects at some point
                    OO.lambdaDesc = lambda_desc;
                case "ga"
                    nvars = sum([obj.OptiVars.Enabled]);
                    [X_opt_s,f_opt,OO.exitflag,OO.output,OO.population,OO.scores] = ga(@objfun_local ,nvars,[],[],[],[],lb,ub,@nlcon_local,optimopts);
            end
            
            F_opt = obj.Objective.f2val(f_opt); % Transform objective function output to desired output
            OO.F_opt = F_opt;
                
            obj.QR.PT.SimpleModel.ConstrainInput = qr_con_in_cache; 
         
            % Set Current Values to Optimal Value in OptiVars
            setVals(obj.OptiVars, X_opt_s);
            obj.updateQR(true);
            OO.X_opt = unscale(obj.OptiVars);
            
            % Get optimal ParamVals to load later or for discrete search
            OO.ParamVals = getValues(filterEnabled(obj.OptiVars, "Child"));
            
            if opts.CaptureState
                OO.PerformanceData = obj.QR.PerformanceData;
                OO.DesignData = obj.QR.DesignData;
            end
            
            if opts.Timer
                OO.OptimTime = seconds(toc(timer));
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
                    [c,ceq] = nlcon(obj);
                catch
                    [c,ceq] = nlcon(obj);
                end
            end
        end
        
        function [grad, grad_s] = fdiff(obj, opts)
            arguments
                obj
                opts.Function = obj.Objective
                opts.Vars = obj.OptiVars(isEnabled(obj.OptiVars))
                opts.FiniteDifferenceStepSize = sqrt(eps)                
            end
            
            vars = opts.Vars;
            v = opts.FiniteDifferenceStepSize;
            F = opts.Function;
            
            param_cache = getValues(obj.QR.Params);
            dep_cache = setDependent(obj.DependentParams, true); % Need to use surrogate models for this
            
            x = vertcat(vars.Value);
            obj.updateQR(false);
            f = F.Value(obj.QR);
            
            typical_x = vertcat(vars.x0);
            delta = v.*sign(x).*max(abs(x),typical_x);
            
            N = numel(vars);
            grad = zeros(N,1);
            for i = 1:N
                var = vars(i);
                val_cache = var.Value;
                
                var.Value = var.Value + delta(i);
                obj.updateQR(false);
                f_step = F.Value(obj.QR);
                grad(i) = (f_step - f)/delta(i);
                
                var.Value = val_cache;
            end
            grad_s = x.*grad;
            
            % Cleanup
            setDependent(obj.DependentParams, dep_cache);
            loadValues(obj.QR.Params, param_cache);
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
                opts.ConstraintFunction = []
                opts.ReverseSearch logical = false
                opts.InitializeFromValue logical = false
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
            [OO_opt] = Optimize(obj, 'OptimizationOutput', false, 'OptimizationOpts', {'Display', 'none'});
            F_opt = OO_opt.F_opt;
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
                        [OO(i)] = optiWrapper();
                        F(i) = OO(i).F_opt;
                        if ~isnan(F(i))
                            valid_cnt = valid_cnt + 1;
                            I(1,valid_cnt) = i;
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
                               [OO(j,i)] = optiWrapper();
                               F(j,i) = OO(j,i).F_opt;
                               if ~isnan(F(j,i))
                                   valid_cnt = valid_cnt + 1;
                                   I(:,valid_cnt) = [j;i];
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
           so.OO = OO;
           so.OO_opt = OO_opt;
           
           % Clean Up
           obj.OptiVars.reset();
           
            function [oo] = optiWrapper()
                try
                    [oo] = Optimize(obj, 'OptimizationOutput', false, 'OptimizationOpts', {'Display', 'none'}, 'InitializeFromValue',opts.InitializeFromValue);
                catch
                    oo = OptimOutput();
                    oo.exitflag = -4;
                    oo.F_opt = NaN;
                end
            end
        end
        
        function [oo,range] = EpsilonConstraint(obj, constraint, property, range)
            con_cache = obj.Constraints;
            obj.Constraints = [obj.Constraints; constraint];
            
            N = numel(range);
            oo = OptimOutput.empty(N,0);
            for i = 1:N
                constraint.(property) = range(i);
                oo(i) = optiWrapper();
                disp(oo(i));
            end
            obj.Constraints = con_cache;
            
            function [oo] = optiWrapper()
                try
                    [oo] = Optimize(obj, 'InitializeFromValue', true, 'OptimizationOutput', false, 'OptimizationOpts', {'Display', 'none'});
                catch
                    oo = OptimOutput();
                    oo.exitflag = -4;
                    oo.F_opt = NaN;
                end
            end
        end
        
        function so = searchNearest(obj, target_arg, N_max_search, N_max_comb, opts)
            % CAUTION: This only works if all components of the same Type
            % have the same parameter values.  Need to add additional 
            % Functionality if there are two components of the same type with 
            % different parameter values.  
            
            arguments
                obj
                target_arg = []
                N_max_search = inf
                N_max_comb (1,1) double = inf
                opts.EnforceBounds logical = true
                opts.BruteResort logical = false
                opts.DistanceMode string = "Norm" % Options: "Norm", "WeightedNorm"
                opts.Weights = []
                opts.Display logical = false
                opts.Plot logical = false
                opts.Timer logical = true
            end
            
            if opts.Timer
                timer = tic;
            end
            
            if ~isempty(target_arg)
                if isa(target_arg, 'OptimOutput')
                    target = target_arg.ParamVals;
                elseif isa(target_arg, 'compParamVal')
                    target = target_arg;
                end
            else
                target = getValues(filterEnabled(obj.OptiVars, "Child"));
            end

            % Get Initial component sets based on distance from optimal
            % point
            if opts.EnforceBounds
                lb = filterEnabled(obj.OptiVars,"lb");
                ub = filterEnabled(obj.OptiVars,"ub");
            else
                lb = [];
                ub = [];
            end
            
            F0 = processF(obj, objfun(obj, true));
            
            [cd,d,comp_names] = filterNearest(obj.CD, target, N_max_search, 'LB', lb, 'UB', ub, 'DistanceMode', opts.DistanceMode, 'Weights', opts.Weights);
            
            setDependent(obj.DependentParams, true); % We want to make sure that any parameter values we don't suppy are handled by the surrogate models
            if opts.BruteResort
                loadValues(obj.QR.Params, target); % Set QR Parameters to the optimal target design
                oopv_cache = getValues(obj.QR.Params); % Cache QuadRotor Parameters
                d_sorted = cell(size(cd));
                cd_sorted = cell(size(cd));
                for i = 1:numel(d_sorted)
                    comps = cd{i};
                    N_comps = numel(comps);
                    d_objective_comp = NaN(N_comps, 1);
                    for j = 1:N_comps
                        % Evaluate Objective
                        [fval, ~] = evalCombination(obj, comps(j), opts.Display);
                        d_objective_comp(j) = fval;
                        % Reset QR
                        loadValues(obj.QR.Params, oopv_cache);
                    end
                    [d_objective_comp_sorted, I] = sort(d_objective_comp);
                    d_sorted{i} = d_objective_comp_sorted;
                    cd_sorted{i} = comps(I); % Reassign components to cd in the new order
                end
                
                d = d_sorted;
                cd = cd_sorted;
            end
            
            % Apply second N_max here if necessary to reduce number of
            % combinations to evaluate
            
            [comb_array,comb_I] = combinations(cd{:});
            sz = size(comb_array);
            
            % comb_d: matrix of size comb_array corresponding to distance
            % of component from target
            comb_d = NaN(sz);
            for i_col = 1:sz(2)
                col = d{i_col};
                comb_d(:,i_col) = col(comb_I(:,i_col));
            end
            
            if opts.BruteResort
                % Sort component combinations by mean objective function value
                comb_d_norm = mean(comb_d,2);
            else
                switch opts.DistanceMode
                    case "Norm"
                        comb_d_norm = vecnorm(comb_d, 2, 2);
                    case "WeightedNorm"
                        comb_d_norm = vecnorm(comb_d, 2, 2);
                end
            end
                       
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
                
                if opts.BruteResort
                    % Sort component combinations by mean objective function value
                    aggregate_label = "Average";
                    individual_label = "ObjFun";
                else
                    aggregate_label = "Norm";
                    individual_label = "d";
                end
                
                for i = 1:sz(2)
                    color = co(i,:);
                    an_d(i) = animatedline(ax_d, 'DisplayName', comp_names(i), 'Color', color);
                end
                
                an_d_norm = animatedline(ax_d, 'DisplayName', aggregate_label);
                
                title("Component Distance Metric")
                ylabel(individual_label);
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
            so.F0 = F0;
            so.F_opt = obj.processF(opt_fval);
            so.OptimalIteration = opt_i;
            so.SortedConfigurations = sorted_combs;
            so.SortedFVals = sorted_FVals;
            so.SortedDistances = sorted_distances;
            so.SortedComponentIndices = sorted_component_indices;
            so.NormalizedDistances = normalized_distances;
            so.ModifiedParameters = pmod;
            so.ComponentNames = comp_names;
            so.DistanceMode = "Norm";
            if opts.Timer
                so.SearchTime = seconds(toc(timer));
            end
            
            function [fval, pmod] = evalCombination(obj, comb, dispflag)
                cpv = vertcat(comb.Data);
                
                % Load Values into Parameters
                pmod = loadValues(obj.QR.Params, cpv);
                
                % Cache Dependency of Modified Parameters and make all
                % Independent
                dep_cache = setDependent(pmod, false); % Makes each element of pmod independent
                
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
                setDependent(pmod, dep_cache);
                
                if dispflag
                    disp("Configuration:");
                    disp(summaryTable(comb))
                    fprintf("Objective Function Value: %f\n\n", fval);
                end 
            end
        end

        function f_ = objfun(obj, success_flag)
            if success_flag
                f_ = f(obj.Objective, obj.QR);
            else
                f_ = Inf;
            end
            % implement objective function scaling at some point
        end

        function [c,ceq] = nlcon(obj)
            c = g(obj.Constraints, obj.QR);
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
        
        function resetQR(obj)
            restoreDependentDefault(obj.DependentParams);
            loadValues(obj.QR.Params, obj.X0Params);
            obj.QR.update();
        end
    end
end

