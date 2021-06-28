classdef QuadRotor < System
    properties
        rho double = 1.205 % Air Density - kg/m^3
        Height double = 0.1 % Approximate Height of the quadrotor in m, used to estimate drag
        DragCoefficient double = 1.2
        
        ConstrainInput logical = true % Prevents calcSteadyState from returning a solution with u < 0 or u > 1
    end
    
    properties (SetAccess = private)
        % Dependent Parameters - Properties that are functions of the QuadRotor
        % Parameters
        Mass extrinsicProp % Don't construct new one - already exists
        J compParam  = compParam("J");% Inertia Taken about COM
        
        HoverThrust function_handle % Thrust required to hover
        HoverSpeed function_handle  % Speed required to hover
        
        % Drag Model Sym Quantities
        ReferenceAreaVector
    end
    
    properties
        % Expensive calculations are cached
        SS_QAve QRState % Steady State at average battery voltage 
        flight_time double 
        range double 
    end
    
    % Easier Access to Components
    properties (SetAccess = private)
        Frame Frame
        PT PowerTrain
        
        BM BodyModel
    end
    
    properties (Dependent)
        PerformanceData
        DesignData
    end
    
    properties (Hidden)
        SteadyState_func function_handle % Used in solved for 0, i.e. solve(obj.SteadyState_func == 0)
        SteadyStateIO_func function_handle % Solved for 0 in calcSteadyStateIO
    end

    methods
        function obj = QuadRotor(p)
            arguments
                p.Frame Frame = Frame('Name', "Frame");
                
                p.Battery Battery = Battery('Name', "Battery");
                p.DCBus DCBus_CurrentEquivalence = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',4);
                p.PMSMInverter PMSMInverter = PMSMInverter('Name', "Inverter");
                p.PMSMMotor PMSMMotor = PMSMMotor('Name', "Motor");
                p.Propeller Propeller = Propeller('Name', "Propeller");
            end
            frame = p.Frame;
            pt = PowerTrain('Battery', p.Battery, 'DCBus', p.DCBus, 'PMSMInverter', p.PMSMInverter, 'PMSMMotor', p.PMSMMotor, 'Propeller', p.Propeller);
            qrcomps = [frame; pt];
            
            obj = obj@System("QuadRotor", qrcomps);
            obj.Frame = p.Frame;
            obj.PT = pt;
           
            
            %init_post(obj);
        end

        function init_post(obj)
            setCombinedParams(obj, 'FilterDirectDescendentExProps', true);
            setParamQuantities(obj);
            setSteadyStateFunc(obj);
            setSteadyStateIOFunc(obj);
            update(obj);
            calcControllerGains(obj);
        end

        function update(obj)
            obj.Params.update();
            obj.SS_QAve = calcSteadyState(obj);
            obj.flight_time = [];
            obj.range = [];
        end
        
        function setParamQuantities(obj)                       
            % Mass
            exps = obj.Params.extrinsicProps;
            masses = getProp(exps, 'Mass');
            obj.Mass = masses(end);
            
            % Inertia
            J = obj.J;
            J.Size = [3 3];
            J.Assumptions = "real";
            J.Description = "Inertia Tensor about COM";
            J.Unit = "kg*m^2";
            J.Parent = obj;
            J_fun = @(J_f,d,m_m, m_p) J_f + ((m_m + m_p)*d^2)*diag([2,2,4]);
            J_bkpts =  [obj.Frame.J_f, obj.Frame.d, obj.PT.Motor.Mass, obj.PT.Propeller.Mass];
            J.setDependency(J_fun, J_bkpts);
            J.Dependent = true;
            J.update();

            % Hover Thrust
            obj.HoverThrust = @() 9.81*obj.Mass.Value; % Total Thrust required to hover
            % Hover Speed
            obj.HoverSpeed = @() obj.PT.RotorSpeed(obj.HoverThrust());
            
            % Reference Area Vector for Drag Model (Weekly Report 5/3/21)
            h = obj.Height;
            r = obj.PT.Propeller.D*1;
            obj.ReferenceAreaVector = matlabFunction([obj.PT.Propeller.D], [2*r*h; 2*r*h; pi*r^2]);
        end
                
        function setSteadyStateFunc(obj)
            solve_vars = [sym('u1'); sym('x2')];
            subs_vars = [sym('x1'); sym('x3')];
            
            args = {solve_vars, subs_vars};
            
            obj.SteadyState_func = matlabFunction(obj.Params, obj.SimpleModel.f_sym(2:end), args);
        end
        
        function setSteadyStateIOFunc(obj)
            solve_vars = [sym('x2'); sym('x3')];
            subs_vars = [sym('u1'); sym('x1')];
            
            args = {solve_vars, subs_vars};
            
            obj.SteadyStateIO_func = matlabFunction(obj.Params, obj.SimpleModel.f_sym(2:end),args);
        end
        
        function qrss = calcSteadyState(obj, q_bar, T_reqd, opts)
            % Calculates steady-state values of the powertrain model at thrust T_reqd, returns QRSteadyState object
            % Default value for q_bar is the average battery soc
            % Default value fot T_reqd is the HoverThrust
            
            arguments
                obj
                q_bar double = [];
                T_reqd double = [];
                opts.SolverOpts struct = optimset('Display','off');
            end
            
            if isempty(q_bar)
                q_bar = obj.Battery.Averaged_SOC;
            end
            
            if isempty(T_reqd)
                rotor_speed = obj.HoverSpeed();
            else
                rotor_speed = obj.RotorSpeed(T_reqd);
            end
            
            
            x0 = [0.5; 1];
            [x_sol, ~, exit_flag] = fsolve(@(x) obj.SteadyState_func(x,[q_bar;rotor_speed]), x0, opts.SolverOpts);
            if exit_flag <= 0
                error('No solution found');
            end
            if obj.ConstrainInput
                if x_sol(1) > 1
                    error('No valid solution.  Required input exceeds 1');
                elseif x_sol(1) < 0
                    error('No valid solution.  Required input must be positive');
                end
            end
            
            qrss = QRState();
            qrss.q = q_bar;
            qrss.x = [q_bar; x_sol(2:end); rotor_speed];
            qrss.u = x_sol(1);
            qrss.y = obj.SimpleModel.CalcG(qrss.x, qrss.u, []);
            qrss.BatteryOCV = obj.V_OCV_pack(q_bar);
        end
        
        function qrss = calcSteadyStateIO(obj, u, q, opts)
            % Calculates steady-state thrust given input u, battery SOC q
            % Default value for q is average battery soc
            
            arguments
                obj
                u
                q double = []
                opts.SolverOpts = optimset('Display','off');
            end
            
            if isempty(q)
                q = getComponents(obj,'Battery').Averaged_SOC;
            end
            
            hover_speed = obj.HoverSpeed(); % Just used for initialization.
            
            x0 = [1; hover_speed];
            [x_sol, ~, exit_flag] = fsolve(@(x) obj.SteadyStateIO_func(x,[u;q]), x0, opts.SolverOpts);
            if exit_flag <= 0
                error('No solution found');
            end
            
            x_sol = [q; x_sol];
            u_sol = u;
            y_sol = obj.SimpleModel.CalcG(x_sol, u_sol, []);
            
            qrss = QRState();
            qrss.q = q;
            qrss.x = x_sol;
            qrss.u = u_sol;
            qrss.y = y_sol;
            qrss.BatteryOCV = obj.V_OCV_pack(q);
        end
        
        function u_bar_func = calcSteadyStateInputFunc(obj, opts)
            arguments
                obj
                opts.Resolution double = 0.01
            end
            
            solver_opts = optimset('Display','off');
            
            q_vals = 0:opts.Resolution:1;
            u0_vals = zeros(size(q_vals));
            for i = 1:numel(q_vals)
                qrss = calcSteadyState(obj, q_vals(i), 'SolverOpts', solver_opts);
                u0_vals(i) = qrss.u;
            end
            
            u_bar_func = @(q) interp1(q_vals, u0_vals, q, 'pchip');
        end
        
        function tr = calcThrustRatio(obj)
            qrsio = calcSteadyStateIO(obj, 1);
            T_max = qrsio.TotalThrust;
            T_hover = obj.HoverThrust();
            
            tr = T_max / T_hover;
        end
        
        function [t_out, y_out, q_out, errFlag, state_out] = Simulate(obj, opts)
            arguments
                obj
                opts.MaxClimbRateReference double = 20
                opts.MaxSimTime double = 5e4
                opts.Timeout double = inf
                opts.PlotResults logical = true
                opts.FeedForwardW logical = true
                opts.FeedForwardU logical = true
                opts.SolverOpts cell = {}
            end


        end
        
        function [flight_time] = flightTime(obj)
            arguments
                obj
            end
            
            cap = obj.Battery.Capacity.Value; % A*s
            ave_current = obj.SS_QAve.y(5);
            flight_time = cap/ave_current;
        end
        
        function [range, speed, flight_time, theta_0] = Range(obj, theta_0, opts)
            % Calculates Range as a function of Pitch angle theta_0 < 0
            % if no pitch angle is specified, the optimal pitch is 
            % calculated with fminbnd
            arguments
               obj
               theta_0 double = []
               opts.DisplayWarnings = false
               opts.SweepRange = false
               opts.SweepPoints = 100
               opts.Theta0Range = [-pi/2 0]
               opts.MaxPitch = false
            end
            
            theta_0_range = opts.Theta0Range; % Pitch angle can range from -90deg to 0deg
            
            if ~opts.MaxPitch
                if isempty(theta_0)
                    t_prev = []; % Used to prevent from evaluating f and ceq more than necessary
                    x0 = -0.1;
                    lb = theta_0_range(1);
                    ub = theta_0_range(2);
                    optimopts = optimoptions('fmincon', 'Display', 'none');
                    con_in_cache = obj.ConstrainInput;
                    obj.ConstrainInput = false;
                    [theta_0] = fmincon(@objfun, x0, [],[],[],[],lb,ub,@nonlcon,optimopts);
                    obj.ConstrainInput = con_in_cache;
                else
                    assert(theta_0 >= theta_0_range(1) && theta_0 <= theta_0_range(2), "Pitch angle can range from -90deg to 0deg");
                end
                [range, speed, flight_time] = calcRange(obj, theta_0, "PitchAngle", opts.DisplayWarnings);
            else
                qrsmax = calcSteadyStateIO(obj, 1);
                [range, speed, flight_time, theta_0] = calcRange(obj, qrsmax, "Thrust", opts.DisplayWarnings);
            end
            
            if opts.SweepRange % Sweep Range
                sweepRange();
            end
            
            function [range, speed, flight_time, theta_0_, u] = calcRange(obj, arg1, mode, warn_flag)
                T_hover = obj.HoverThrust(); % m*g
                rho = obj.rho;
                Cd = obj.DragCoefficient;
                S = obj.ReferenceAreaVector();
                
                switch mode
                    case "PitchAngle"
                        qrstate_arg_flag = false;
                        theta_0_ = arg1;
                        T_trim = T_hover*sec(theta_0_);
                    case "Thrust"
                        qrstate_arg_flag = isa(arg1, 'QRState');
                        if qrstate_arg_flag
                            T_trim = arg1.TotalThrust;
                        else
                            T_trim = arg1;
                        end
                        theta_0_ = -abs(asec(T_trim/T_hover));
                end
                
                u = [cos(theta_0_); 0; sin(theta_0_)]; % unit vector of relative velocity in the body frame
                
                speed = sqrt((-2*T_hover*tan(theta_0_)) / (rho * Cd * dot(S,abs(u))));
                
                try
                    if qrstate_arg_flag
                        qrs = arg1;
                    else
                        qrs = obj.calcSteadyState([], T_trim);
                    end
                    u = qrs.u;
                    cap = obj.Battery.Capacity.Value; % A*s
                    ave_current = qrs.BusCurrent;
                    flight_time = cap/ave_current;
                    
                    range = speed*flight_time;
                catch ME
                    if warn_flag
                        disp( getReport( ME, 'extended', 'hyperlinks', 'on' ) )
                    end
                    u = NaN;
                    flight_time = NaN;
                    range = NaN;
                end
            end
            
            function sweepRange()
                [nom_range, nom_speed, nom_flight_time] = deal(range, speed, flight_time);
                
                n = opts.SweepPoints;
                range = zeros(1,n);
                speed = zeros(1,n);
                flight_time = zeros(1,n);
                
                x = linspace(theta_0_range(1), theta_0_range(2), n);
                for i = 1:n
                    [range(i), speed(i), flight_time(i)] = calcRange(obj, x(i), "PitchAngle", false);
                end
                speed(isnan(range)) = NaN;
                
                figure('Name', 'Range Sweep')
                
                subplot(3,1,1)
                plot(x, range)
                xlim(theta_0_range)
                ylabel('Range (m)')
                set(gca,'XTickLabel',[]);
                hold on
                plot(theta_0, nom_range, '.r', 'MarkerSize',20);
                hold off
                
                subplot(3,1,2)
                plot(x, speed)
                xlim(theta_0_range)
                ylabel('Speed (m/s)')
                set(gca,'XTickLabel',[]);
                hold on
                plot(theta_0, nom_speed, '.r', 'MarkerSize',20);
                hold off
                
                subplot(3,1,3)
                plot(x, flight_time)
                xlim(theta_0_range)
                ylabel('Flight Time (s)')
                xlabel('$$\theta_0$$ (rad)', 'Interpreter', 'latex')
                hold on
                plot(theta_0, nom_flight_time, '.r', 'MarkerSize',20);
                hold off
                
                theta_0 = x;
            end
            
            function f = objfun(t)
                [r,~] = calcRangeOptWrapper(t);
                f = -r;
            end
            
            function [c,ceq] = nonlcon(t)
                [~,u] = calcRangeOptWrapper(t);
                c = u-1;
                ceq = [];
            end
            
            function [r,u] = calcRangeOptWrapper(t)
                persistent r_prev u_prev
                if ~isempty(t_prev) && t == t_prev
                    r = r_prev;
                    u = u_prev;
                    return
                end
                
                [r,~,~,~,u] = calcRange(obj, t, "PitchAngle", false);
                t_prev = t;
                r_prev = r;
                u_prev = u;
            end
        end
               
        %% Export PerformanceData and Design Data
        function pd = get.PerformanceData(obj)
            pd = PerformanceData();
            pd.FlightTime = obj.flight_time;
            pd.Range = obj.range;
            pd.ThrustRatio = calcThrustRatio(obj);
            pd.SteadyState = obj.SS_QAve;
        end
        
        function dd = get.DesignData(obj)
            dd = exportStruct(obj.Params);
        end
        
        %% Get Methods for cached properties
        function ss = get.SS_QAve(obj)
            if isempty(obj.SS_QAve)
                obj.SS_QAve = calcSteadyState(obj);
            end
            ss = obj.SS_QAve;
        end
        
        function ft = get.flight_time(obj)
            if isempty(obj.flight_time)
                obj.flight_time = obj.flightTime();
            end
            ft = obj.flight_time;
        end
        
        function r = get.range(obj)
            if isempty(obj.range)
                obj.range = obj.Range();
            end
            r = obj.range;
        end
    end
end