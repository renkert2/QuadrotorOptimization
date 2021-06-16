classdef QuadRotor < System
    properties
        K_PI_speed % Inner Rotor Speed Loop controller Gains
        K_PD_vel % Outer Height Loop controller Gains
        K_P_height % Outer Velocity Loop controller gains
        
        rho double = 1.205 % Air Density - kg/m^3
        Height double = 0.1 % Approximate Height of the quadrotor in m, used to estimate drag
        DragCoefficient double = 1.2
        
        ConstrainInput logical = true % Prevents calcSteadyState from returning a solution with u < 0 or u > 1
    end
    
    properties (SetAccess = private)
        SymVars SymVars
        
        SimpleModel Model
        LinearDynamicModel LinearModel
        
        % Dependent Parameters - Properties that are functions of the QuadRotor
        % Parameters
        Mass extrinsicProp
        J compParam % Inertia Taken about COM
        
        HoverThrust function_handle % Thrust required to hover
        HoverSpeed function_handle  % Speed required to hover
        
        RotorSpeed function_handle % Calculates rotor speed as a function of total required thrust
        V_OCV_pack function_handle % Open circuit voltage of the battery pack; function of Q.
        
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
        Battery Battery
        DCBus DCBus_CurrentEquivalence
        Inverter PMSMInverter
        Motor PMSMMotor
        Propeller Propeller
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
           
            pmsminverters = Replicate([p.PMSMInverter], 4);
            pmsminverters = pmsminverters{1};
            
            motorprop = MotorProp('PMSMMotor', p.PMSMMotor, 'Propeller', p.Propeller);
            motorprops = Replicate(motorprop, 4, 'RedefineChildren', false);
            motorprops = vertcat(motorprops{:});
           
            
            % I would love for this to work at some point.
            % motorprop = MotorProp('PMSMMotor', p.PMSMMotor, 'Propeller', p.Propeller);
            % pmsminverters = repmat(p.PMSMInverter, 1, 4);
            % motorprops = repmat(motorprop, 1, 4);
            
            Components = [p.Frame; p.Battery; p.DCBus; pmsminverters; motorprops];
            
            ConnectP = {[p.Battery.Ports(1) p.DCBus.Ports(1)]};
            
            for i = 1:4
                ConnectP{end+1,1} = [pmsminverters(i).Ports(1),p.DCBus.Ports(1+i)];
                ConnectP{end+1,1} = [pmsminverters(i).Ports(2), motorprops(i).Ports(1)];
            end
            
            obj = obj@System("Quad Rotor", Components, ConnectP);
            obj.Frame = p.Frame;
            obj.Battery = p.Battery;
            obj.DCBus = p.DCBus;
            obj.Inverter = p.PMSMInverter;
            obj.Motor = p.PMSMMotor;
            obj.Propeller = p.Propeller;
            
            warning('off', 'Control:combination:connect10') % Annoying message from calcControllerGains
            init_post(obj);
        end

        function init_post(obj)
            createModel(obj);
            setParamQuantities(obj);
            setSimpleModel(obj);
            setLinearDynamicModel(obj);
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
            p = obj.Params;
            PF = @(s) matlabFunction(p,s); % Wrapper function for brevity
                       
            % Mass
            exps = obj.Params.extrinsicProps;
            masses = getProp(exps, 'Mass');
            obj.Mass = masses(end);
            
            % Inertia
            J = compParam("J");
            J.Description = "Inertia Tensor about COM";
            J.Unit = "kg*m^2";
            J.Parent = obj;
            J_f = obj.Frame.J_f;
            d = obj.Frame.d;
            J_fun = @(m_m, m_p) J_f + ((m_m + m_p)*d^2)*diag([2,2,4]);
            J_bkpts =  get(obj.Params,["Mass__Motor", "Mass__Propeller"]);
            setDependency(J, J_fun, J_bkpts);
            J.Dependent = true;
            J.update();

            obj.J = J;
            obj.Params(end+1) = J;
            
            % Battery Pack V_OCV
            obj.V_OCV_pack = matlabFunction(p, obj.Battery.V_OCV_pack, {sym('q')});

            % Hover Thrust
            obj.HoverThrust = @() 9.81*obj.Mass.Value; % Total Thrust required to hover

            % Hover Speed
            prop = obj.Propeller;
            mass_pop = pop(obj.Mass);
            thrust_pop = mass_pop*9.81;
            hover_speed = prop.calcSpeed(thrust_pop/4);
            obj.HoverSpeed = PF(hover_speed);
            
            % Rotor Speed Function 
            % - Calculates required rotor speed as a function of total
            % thrust
            T_reqd = sym("T_reqd");
            rotor_speed = prop.calcSpeed(T_reqd/4);
            obj.RotorSpeed = matlabFunction(p, rotor_speed, {T_reqd});
            
            % Reference Area Vector for Drag Model (Weekly Report 5/3/21)
            h = obj.Height;
            r = obj.Propeller.D*1;
            obj.ReferenceAreaVector = PF([2*r*h; 2*r*h; pi*r^2]);
        end
        
        function setSimpleModel(obj)
            simple_model = Model();
            
            % STATES
            %dyn_eqns dot(x) = dyn_eqns(x)
            %1.         x1       "Battery"        "Battery SOC"           Abstract
            %2.         x2       "Motor"        "Inductance (i_q)"      Current
            %3.         x3       "Motor"        "Inertia (omega_m)"     AngularVelocity
            
            states = [1,2,3];
            simple_model.StateDescriptions = ["Battery SOC"; "Motor Current"; "Rotor Speed"];
            simple_model.Nx = numel(states);
            
            % INPUTS
            simple_model.Nu = 1;
            simple_model.InputDescriptions = ["Inverter Input"];
            
            
            % OUTPUTS:
            %1-3.  States 1-3
            %4.  Bus Voltage
            %5.  Bus Current
            %6.  Inverter Current (DC)
            %7.  Inverter Voltage (Q)
            %8. Torque
            %9. Thrust - Total
  
            outs = [states 10 11 12 13 21 20];
            simple_model.Ny = numel(outs);
            simple_model.OutputDescriptions = [simple_model.StateDescriptions; "Bus Voltage"; "Bus Current"; "Inverter Current (DC)"; "Inverter Voltage (q)"; "Torque"; "Total Thrust"];
            
            obj.SymVars = SymVars('Nx', simple_model.Nx, 'Nd', simple_model.Nd, 'Nu', simple_model.Nu);
            
            digits 8 % Set VPA Digits to 8
            
            % Set f_sym
            dyn_eqns = vpa(obj.Model.f_sym(states));
            simple_model.f_sym = combine(vpa(obj.convertSyms(dyn_eqns))); % combine makes sym expression a bit simpler
            
            % Set g_sym
            g_sym_mod = obj.convertSyms(vpa(obj.Model.g_sym(outs)));
            g_sym_mod(end) = g_sym_mod(end)*4; % Convert Thrust per Propeller to Total Thrust
            simple_model.g_sym = combine(g_sym_mod); % combine makes sym expression a bit simpler
            
            simple_model.Params = obj.Params;
            simple_model.SymVars = obj.SymVars;
            
            simple_model.init();
            
            obj.SimpleModel = simple_model;
        end
        
        function setLinearDynamicModel(obj)
            u_mod = obj.SymVars.u;
            
            % Cut Battery Dynamics out of Dynamic Model
            x_mod = obj.SymVars.x(2:end);
            f_sym_mod = obj.SimpleModel.f_sym(2:end);
            g_sym_mod = obj.SimpleModel.g_sym([3,end]);
            
            A = jacobian(f_sym_mod, x_mod);
            B = jacobian(f_sym_mod, u_mod);
            
            C = jacobian(g_sym_mod, x_mod);
            D = jacobian(g_sym_mod, u_mod);
            
            lm = LinearModel();
            lm.Nx = 2;
            lm.Nu = 1;
            lm.Nd = 1; % Treat battery SOC as disturbance
            lm.Ny = 2;
            lm.StateDescriptions = obj.SimpleModel.StateDescriptions(2:end);
            lm.InputDescriptions = obj.SimpleModel.InputDescriptions;
            lm.DisturbanceDescriptions = "Battery SOC";
            lm.OutputDescriptions = obj.SimpleModel.OutputDescriptions([3,end]);
            sv = obj.SimpleModel.SymVars;
            sv.x = sv.x(2:end);
            sv.d = sym('x1');
            lm.SymVars = sv;
            p = [obj.Params];
            lm.Params = p;
            
            lm.A_sym = A;
            lm.B_sym = B;
            lm.E_sym = zeros(lm.Nx,1);
            lm.C_sym = C;
            lm.D_sym = D;
            lm.H_sym = zeros(lm.Ny,1);
            
            lm.f0 = zeros(lm.Nx,1);
            lm.g0 = zeros(lm.Ny,1);
            
            lm.init();
            obj.LinearDynamicModel = lm;
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
        
        function [A,B,C,D] = calcLinearMatrices(obj, qrss)
            if nargin == 1
                qrss = obj.SS_QAve;
            end
            
            [A,B,~,C,D,~] = obj.LinearDynamicModel.CalcMatrices(qrss.x(2:end), qrss.u, qrss.x(1));
        end
        
        function [A,B,C,D] = calcBodyModel(obj)
            m = obj.Mass.Value;
            
            A = [0 1;0 0];
            B = [0; 1/m];
            C = [1 0];
            D = [0];
        end
        
        function [K_PI_speed, K_PD_vel, K_P_height] = calcControllerGains(obj, qrss)
            if nargin == 1
                qrss = obj.SS_QAve;
            end
            
            %% Body Model
            [Am,Bm,~,Dm] = calcBodyModel(obj);
            Cm = eye(2);
            
            B = ss(Am,Bm,Cm,Dm);
            B.InputName = {'T'};
            B.OutputName = {'Y','Ydot'};
            
            %% PowerTrain Model
            [Apt, Bpt, Cpt, Dpt] = calcLinearMatrices(obj, qrss);
            gain = Dpt-Cpt*(Apt\Bpt);
            dom_pole = min(abs(eigs(Apt)));
            den = [(1/dom_pole), 1];
            PT = [tf(gain(1), den); tf(gain(2), den)];
            PT.InputName = {'u'};
            PT.OutputName = {'W','T'};
            
            %% Inner Speed Loop
            opts = pidtuneOptions('PhaseMargin', 90);
            PI_speed = pidtune(PT(1),'PI', 10,opts);
            PI_speed.InputName = 'e_w';
            PI_speed.OutputName = 'u';
            
            sb_speed = sumblk('e_w = r_w - W');
            
            speed_loop = connect(sb_speed, PI_speed, PT, 'r_w', {'W', 'T'});          
            %% Velocity (ydot) Loop
            vel_plant = connect(speed_loop, B, 'r_w', 'Ydot');
            opts = pidtuneOptions('PhaseMargin', 90);
            PD_vel = pidtune(vel_plant,'PD', 5,opts);
            PD_vel.InputName = 'e_v';
            PD_vel.OutputName = 'r_w';
            
            sb_vel = sumblk('e_v = r_v - Ydot');
            vel_loop = connect(sb_vel, PD_vel, speed_loop, B, 'r_v',{'Y','Ydot'});
            %% Outer Height Loop
            P_height = pidtune(vel_loop(1),'P',1);
            
            %% Set Gains
            K_PI_speed = [PI_speed.Kp, PI_speed.Ki];
            obj.K_PI_speed = K_PI_speed;
            
            K_PD_vel = [PD_vel.Kp, PD_vel.Kd];
            obj.K_PD_vel = K_PD_vel;
            
            K_P_height = P_height.Kp;
            obj.K_P_height = K_P_height;
        end
        
        function [t_out, y_out, q_out, errFlag, state_out] = Simulate(obj, r, opts)
            arguments
                obj
                r function_handle = (@(t) t>=0)
                opts.ReferenceFilterTimeConstant double = 0.5
                opts.MaxClimbRateReference double = 20
                opts.MaxSimTime double = 5e4
                opts.Timeout double = inf
                opts.PlotResults logical = true
                opts.FeedForwardW logical = true
                opts.FeedForwardU logical = true
                opts.SolverOpts cell = {}
            end

            % init
            model = obj.SimpleModel;
            qrss = obj.calcSteadyState(1); % Calc steady state at full battery
            
            % Indices in composite x = [x_g; x_b; x_sc; x_hc; x_r]
            % - x_g: States of Power Train (Graph Model)
            % - x_b: States of Body Model, x_b = [y, dot(y)]
            % - x_sc: States of Rotor Speed Controller
            % - x_vc: States of Velocity Controller
            % - x_r: States of Reference Input Filter
            
            persistent i_x_g i_x_b i_x_sc i_x_vc i_x_r
            if isempty(i_x_g)
                i_x_g = 1:obj.SimpleModel.Nx;
                i_x_b = i_x_g(end) + (1:2);
                i_x_sc = i_x_b(end) + 1;
                i_x_vc = i_x_sc(end) + 1;
                i_x_r = i_x_vc(end) + 1;
            end
            N_states = i_x_r(end);
            
            % Graph Model (PowerTrain)
            f_g = @(x_g,u_g) model.CalcF(x_g, u_g, []);
            g_g = @(x_g,u_g) model.CalcG(x_g, u_g, []);
            
            % Body Model
            % Computes states [y diff(y)]
            [A_b,B_b,~,~] = calcBodyModel(obj);
            f_b = @(x_b,u_b) A_b*x_b + B_b*u_b;
            
            % Speed Controller
            % - Input: speed error signal
            % - Output: Inverter control ouptut
            f_sc = @(x_sc,u_sc) u_sc;
            g_sc = @(x_sc,u_sc) obj.K_PI_speed(1)*u_sc + obj.K_PI_speed(2)*x_sc;
            
            % Velocity Controller
            % - Input: velocity error signal
            % - Output: Propeler Speed reference input
            tau = 0.1; % Derivative filter time constant
            f_vc = @(x_vc, u_vc) -(1/tau)*x_vc + u_vc;
            g_vc = @(x_vc, u_vc) obj.K_PD_vel*([1/tau; -1/tau^2]*x_vc + [0; 1/tau]*u_vc);
            
            % Height Controller
            % - Input: [e_h]
            % - Output: Desired Climb Rate
            g_hc = @(u_hc) obj.K_P_height*u_hc;
            
            % Reference Filter
            % Input - reference signal r
            % Output: [r_filtered; dot_r_filtered]
            tau = opts.ReferenceFilterTimeConstant;
            f_r = @(x_r, u_r) -(1/tau)*x_r + u_r;
            g_r = @(x_r, u_r) [1/tau; -1/tau^2]*x_r + [0; 1/tau]*u_r;
            g_r_1 = @(x_r) 1/tau.*x_r;
            
            % Nominal Thrust
            T_bar = qrss.TotalThrust;
            
            if opts.FeedForwardW
                w_bar = qrss.RotorSpeed;
            else
                w_bar = 0;
            end
            
            % SS Input
            if opts.FeedForwardU
                u_bar = qrss.u;
            else
                u_bar = 0;
            end
            
            % Initial Values
            x_0 = zeros(N_states(end),1);
            x_0(i_x_g) = qrss.x;
            x_0(i_x_g(1)) = 1;
            
            % Run Simulation
            sim_opts = odeset('Events', @emptyBattery, 'OutputFcn', @odeFunc, 'RelTol', 1e-6, 'NonNegative', i_x_b(1), opts.SolverOpts{:}); 
            errFlag = 0;
            tic
            [t_out,x_out] = ode23tb(@(t,x) f_sys_cl(t,x), [0 opts.MaxSimTime], x_0, sim_opts);
            
            % Process Minimal Results
            x_b_out = x_out(:,i_x_b);
            y_out = x_b_out(:,1);
            q_out = x_out(:, i_x_g(1));
            
            if nargout > 4 || opts.PlotResults
                % Process additional results
                x_g_out = x_out(:,i_x_g);
                x_r_out = x_out(:,i_x_r);                
                
                % Calculate the input and model outputs at each time
                u_g = zeros(obj.SimpleModel.Nu, numel(t_out));
                y_state = zeros(obj.SimpleModel.Ny, numel(t_out));
                
                for i = 1:numel(t_out)
                    [~,u_g(:,i)] = f_sys_cl(t_out(i), x_out(i,:)');
                    y_state(:,i) = obj.SimpleModel.CalcG(x_g_out(i,:)', u_g(:,i), []);
                end

                state_out = QRState();
                state_out.x = x_g_out';
                state_out.u =  u_g;
                state_out.y = y_state;
                q = state_out.BatterySOC;
                state_out.BatteryOCV = arrayfun(@(x) obj.V_OCV_pack(x), q);
            end
            
            if opts.PlotResults
                plotResults()
            end
            
            function [x_dot, u_g] = f_sys_cl(t,x)
                x_g = x(i_x_g);
                x_b = x(i_x_b);
                x_sc = x(i_x_sc);
                x_vc = x(i_x_vc);
                x_r = x(i_x_r);
                
                dot_x_r = f_r(x_r,r(t));
                r_PD = g_r(x_r,r(t)); % Proportional and Derivative Terms of Filtered Reference Signal
                
                u_hc = sqrtControl(r_PD(1)-x_b(1)); % Could use sqrt controller here
                y_hc = g_hc(u_hc); % output of P height controller = speed reference signal
                
                %ff_vc = r_PD(2); % Need to implement feed-forward later
                ff_vc = 0;
                r_vc = y_hc + ff_vc;
                r_vc = min(max(r_vc,-opts.MaxClimbRateReference),opts.MaxClimbRateReference);
                u_vc = r_vc - x_b(2);
                dot_x_vc = f_vc(x_vc, u_vc);
                y_vc = g_vc(x_vc, u_vc);
                
                u_sc = (y_vc - x_g(3)) + w_bar; % Propeller Speed error signal, essentially an acceleration command
                dot_x_sc = f_sc(x_sc, u_sc);
                y_sc = g_sc(x_sc, u_sc); % Output of PI Speed Controller is input to Graph (PowerTrain) Model
                
                u_g = y_sc + u_bar; 
                u_g = min(max(u_g,0),1); % Modify u_g such that 0 < u_g <1
                dot_x_g = f_g(x_g,u_g);
                y_g = g_g(x_g,u_g);
                
                T = y_g(end) - T_bar;
                u_b = T;
                dot_x_b = f_b(x_b, u_b);
                
                x_dot = [dot_x_g; dot_x_b; dot_x_sc; dot_x_vc; dot_x_r];
            end
            
            function [value, isterminal, direction] = emptyBattery(~,y)
                % emptyBattery() is an event that terminates the simulation
                % when the battery SOC drains to zero.
                value = y(i_x_g(1));
                isterminal = 1;
                direction = 0;
            end
            
            function status = odeFunc(~,~,flag)
                % Output function that stops the simulation if it takes too long.
                % Could also use this for fun visualizations :)
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
            
            function plotResults()
                f = figure;
                f.Position = [207.4000 311.4000 1.1016e+03 384.8000];
                subplot(1,2,1)
                plot(t_out, [r(t_out), g_r_1(x_r_out), y_out]);
                xlim([0 25])
                lg = legend(["Reference", "Filtered Reference", "Response"]);
                lg.Location = 'southeast';
                xlabel("t")
                ylabel("Height (m)")
                
                subplot(1,2,2)
                plot(t_out, [r(t_out), g_r_1(x_r_out),y_out]);
                lg = legend(["Reference", "Filtered Reference", "Response"]);
                lg.Location = 'southeast';
                xlabel("t")
                ylabel("Height (m)")
                
                sgtitle("Tracking Performance");
            end
            function x = sqrtControl(x)
                if abs(x) > 1
                    x = sign(x)*(sqrt(abs(4*x)) - 1);
                end
            end
        end
        
        function [flight_time] = flightTime(obj, r, opts)
            arguments
                obj
                r function_handle = (@(t) t>=0)
                opts.InterpolateTime logical = false % This didn't help with the smoothness of the response at all.
                opts.Timeout double = 30
                opts.SimulationBased logical = false
                opts.SimulationOpts cell = {}
            end
            
            if opts.SimulationBased
                [t, ~, q, errFlag] = Simulate(obj, r, 'Timeout', opts.Timeout, 'PlotResults', false, opts.SimulationOpts{:});
                if errFlag
                    flight_time = NaN;
                else
                    if opts.InterpolateTime
                        q_ = q(end-1:end);
                        t_ = t(end-1:end);
                        a = -q_(1)/(q_(2)-q_(1));
                        flight_time = (1-a)*t_(1) + a*t_(2);
                    else
                        flight_time = t(end);
                    end
                end
            else
                cap = obj.Battery.Capacity.Value; % A*s
                ave_current = obj.SS_QAve.y(5);
                flight_time = cap/ave_current;
            end
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
    
    %% Helper Functions
    methods (Access = private)
        function syms_out = convertSyms(obj, syms_in)
            syms_out = convertToBalanced(obj,syms_in);
            syms_out = convertStates(obj,syms_out);
        end
        
        function sym_bal = convertToBalanced(obj, sym_unbal)
            % Replaces independent rotor variables with the same variable so that each
            % rotor has the same symbolic variable
            persistent from to
            if isempty(from) || isempty(to)
                from = {};
                to = {};
                
                from{1} = [sym('u2'), sym('u3'), sym('u4')];
                to{1} = repmat(sym('u1'),1,3);
                
                from{2} = [sym('x4'), sym('x6'), sym('x8')];
                to{2} = repmat(sym('x2'),1,3);
                
                from{3} = [sym('x5'), sym('x7'), sym('x9')];
                to{3} = repmat(sym('x3'),1,3);
            end
            
            sym_bal = subs(sym_unbal,[from{:}], [to{:}]);
        end
        
        function sym_out = convertStates(obj, sym_in)
            from = [sym('x1'); sym('x2'); sym('x3')];
            to = obj.SymVars.x;
            sym_out = subs(sym_in, from,to);
        end
    end
end