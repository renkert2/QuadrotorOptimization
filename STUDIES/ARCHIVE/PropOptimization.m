classdef PropOptimization < handle

    properties (Constant)
        disturbances = zeros(14,1);
    end
    
    properties
        quad_rotor
        prop
        
        stc
        nominal_q
        reqd_thrust
        
        propFit
        D_bounds
        D_init = 0.182
        P_bounds
        P_init = 0.097
        
        mission_thrust_factor (1,:) = 1; % mission thrust factor = 1 -> input will be calculated to maintain hover
        mission_thrust_times (1,:) = 0; % mission thrust times -> starting times corresponding to mission_thrust_factors
        input_sched;
    end
    
    methods
        function obj = PropOptimization()
            obj.init();
        end
        
        function init(obj)
            batt = Battery('Name', 'Battery'); % 4000mAh, 3S Default Battery, No Dynamics for now
            obj.prop = Propeller('Name', 'Propeller', 'k_P', symParam('k_P', 0.03), 'k_T', symParam('k_T', 0.05), 'D', symParam('D', 0.1270));
            obj.quad_rotor = QuadRotor('Battery', batt, 'Propeller', obj.prop);
            obj.quad_rotor.createModel;
            
            obj.stc = obj.prop.square_thrust_coeff; % rad/s
            
            nq = solve(batt.V_OCV_curve == 1);
            obj.nominal_q = double(nq(1));
           
            total_mass = obj.quad_rotor.extrinsicProps(1).Value;
            obj.reqd_thrust = (9.81*total_mass) / 4; % Assumes 4 Propellers, reqd_thrust per propeller
            
            load propFits.mat propFits
            obj.propFit = propFits;
            
            obj.D_bounds = [min(obj.propFit.Boundary.points(:,1)); max(obj.propFit.Boundary.points(:,1))];
            obj.P_bounds = [min(obj.propFit.Boundary.points(:,2)); max(obj.propFit.Boundary.points(:,2))];
        end
        
        function [x_ss, input_sol, reqd_speed, dyn_eqns] = calcSteadyState(obj, X, reqd_thrust)
            [k_P, k_T] = obj.calcPropCoeffs(X);
            
            reqd_speed = obj.prop.calcSpeed(reqd_thrust);
            reqd_speed = double(subs(reqd_speed, [sym('D'), sym('k_T')], [X(1), k_T]));
            
            subs_vars = [sym('x14'), sym('x1'), sym('k_P'), sym('k_T'), sym('D')];
            subs_vals = [reqd_speed, obj.nominal_q, k_P, k_T, X(1)];
            
            %dyn_eqns        
            %         2       "Motor"        "Inductance (i_q)"      Current        
            %         3       "Motor"        "Inertia (omega_m)"     AngularVelocity
            %         10       "Shaft"        "Torque (T)"            Torque         
            %         14      "Propeller"    "Inertia (omega)"       AngularVelocity
            
            digits(8)
            dyn_eqns = vpa(obj.quad_rotor.Model.f_sym([2,3,10,14]));
            dyn_eqns = subs(dyn_eqns, [sym('u2'), sym('u3'), sym('u4'), sym('x4'), sym('x6'), sym('x8')],[sym('u1'), sym('u1'), sym('u1'), sym('x2'), sym('x2'), sym('x2')]);
            dyn_eqns = subs(dyn_eqns, subs_vars, subs_vals);
            
            
            
            assume(0<sym('u1') & sym('u1')<1);
            
            x_ss = vpasolve(dyn_eqns == 0);
            
            input_sol = min(double(x_ss.u1));
            
            if isempty(input_sol)
                error('No Solution Found');
            end
        end

        function [t, y, p_f, errFlag] = Simulate(obj, X, input_sched, opts)
            arguments
                obj
                X
                input_sched = []
                opts.Timeout = Inf
            end
            
            if isempty(input_sched)
                input_sched = calcInputSchedule(obj, X);
            end
            
            [k_P, k_T] = obj.calcPropCoeffs(X);
            
            % System Simulation
            sim_opts = odeset('Events', @emptyBattery, 'OutputFcn', @odeFunc);
            
            errFlag = 0;
            
            tic
           
            [t,y, p_f] = Simulate(obj.quad_rotor.Model, input_sched, obj.disturbances, [X(1); k_P; k_T], [0 5e4], 'PlotStates', false, 'Solver', @ode23tb, 'SolverOpts', sim_opts);
            
            function [value, isterminal, direction] = emptyBattery(~,y)
                % emptyBattery() is an event that terminates the simulation
                % when the battery SOC drains to zero.
                value = y(1);
                isterminal = 1;
                direction = 0;
            end
            
            function status = odeFunc(~,~,flag)
                status = 0;
                if isempty(flag)
                    timer = opts.Timeout - toc;
                    if timer < 0
                        warning('Solver timed out')
                        status = 1;
                        errFlag=1;
                    end
                end
            end
        end
        
        function [d_vals, p_vals, valid_grid] = validDomain(obj, res)
            d_vals = linspace(obj.D_bounds(1), obj.D_bounds(2), res);
            p_vals = linspace(obj.P_bounds(1), obj.P_bounds(2), res);
            
            if nargout == 3
                N_d = numel(d_vals);
                N_p = numel(p_vals);
                valid_grid = zeros(N_d,N_p);
                parfor i = 1:N_d
                    d_val = d_vals(i);
                    for j = 1:N_p
                        p_val = p_vals(j);
                        valid_grid(j,i) = obj.propFit.Boundary.distance_func(d_val, p_val);
                    end
                end
            end
        end
        
        function [input_sched] = calcInputSchedule(obj,X)            
            inputs = zeros(size(obj.mission_thrust_factor));
            for i = 1:length(obj.mission_thrust_factor)
                [~,inputs(i)] = obj.calcSteadyState(X, obj.mission_thrust_factor(i)*obj.reqd_thrust);
            end
            
            if numel(inputs) == 1
                input_sched = repmat(inputs,4,1);
            else
                vals = [0 inputs];
                times = obj.mission_thrust_times;
                input_sched = @input_sched_func; 
            end
            
            obj.input_sched = input_sched;
            
            function input = input_sched_func(t)
                input = repmat(vals(sum(t>=times') + 1),4,1);
            end
        end
            
        function [flight_time] = flightTime(obj, X)
            try
                [t,~,~,errFlag] = Simulate(obj, X, 'Timeout', 5);
                if errFlag
                    flight_time = NaN;
                else
                    flight_time = t(end);
                end
            catch
                flight_time = NaN; 
            end
        end

        function [opt_X, opt_flight_time, output] = Optimize(obj,opts)
            arguments
                obj
                opts.OptimOptions = []
                opts.FixDiameter double = []
            end
           
            x0 = [obj.D_init; obj.P_init];
            %A = [-0.6684 1; 0.5842 -1];
            %b = [0.1022; 0.0342];
            lb = [obj.D_bounds(1); obj.P_bounds(1)];
            ub = [obj.D_bounds(2); obj.P_bounds(2)];
            
            nlcon = @(x) obj.propFit.Boundary.distance_func(x(1,:), x(2,:));
            
            if isempty(opts.FixDiameter)
                [opt_X, fval, ~, output] = fmincon(@(X) -obj.flightTime(X),x0, [], [], [], [], lb, ub, nlcon, opts.OptimOptions);
            else
                D = opts.FixDiameter;
                x0_mod = fmincon(@(x) nlcon([D;x]), x0(2));
                lb_mod = max(lb(2), fzero(@(x) nlcon([D;x]), lb(2)));
                ub_mod = min(ub(2), fzero(@(x) nlcon([D;x]), ub(2)));
                
                [opt_P, fval, ~, output] = fmincon(@(x)-obj.flightTime([D;x]),x0_mod, [], [], [], [], lb_mod, ub_mod, [], opts.OptimOptions);
                opt_X = [D; opt_P];
            end
            
            opt_flight_time = -fval;
        end
        
        function [k_P, k_T] = calcPropCoeffs(obj, X)
            % X = [D ; P]
            cp_fit = obj.propFit.Fit.cp;
            ct_fit = obj.propFit.Fit.ct;
            
            D = X(1,:);
            P = X(2,:);
            
            k_P = cp_fit(D,P);
            k_T = ct_fit(D,P);
        end
    end
end

