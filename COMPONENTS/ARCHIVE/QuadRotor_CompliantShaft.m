classdef QuadRotor < System
    properties
        StandardEmptyWeight = extrinsicProp('Mass', 0.284 - 0.080 - 4*0.008 - 4*0.04) % Mass Not Including Battery, Propellers, and Motors.  Those are added later
        
        K_PI_speed % Inner Rotor Speed Loop controller Gains
        K_PD_vel % Outer Height Loop controller Gains
        K_P_height % Outer Velocity Loop controller gains
    end
    
    properties (SetAccess = private)
        SymVars SymVars
        SymParams SymParams
        
        SimpleModel Model
        LinearDynamicModel LinearModel
        
        BattCap symQuantity
        Mass symQuantity
        HoverThrust symQuantity % Thrust required to hover
        HoverSpeed symQuantity  % Speed required to hover

        sym_param_vals double
        SS_QAve QRSteadyState % Steady State at average battery voltage
    end
    
    properties (Hidden)
        SteadyState_func function_handle % Used in solved for 0, i.e. solve(obj.SteadyState_func == 0)
        SteadyStateIO_func function_handle % Solved for 0 in calcSteadyStateIO
    end

    methods
        function obj = QuadRotor(p)
            arguments
                p.Battery Battery = Battery('Name', "Battery");
                p.DCBus DCBus_CurrentEquivalence = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',4);
                p.PMSMInverter PMSMInverter = PMSMInverter('Name', "Inverter");
                p.PMSMMotor PMSMMotor = PMSMMotor('Name', "Motor");
                p.ShaftCoupler ShaftCoupler = ShaftCoupler('Name', "Shaft");
                p.Propeller Propeller = Propeller('Name', "Propeller");
            end
            
            replicated_comps = Replicate([p.PMSMInverter, p.PMSMMotor, p.ShaftCoupler, p.Propeller], 4);
            pmsminverters = replicated_comps{1};
            pmsmmotors = replicated_comps{2};
            shaftcouplers = replicated_comps{3};
            propellers = replicated_comps{4};
            
            components = [p.Battery, p.DCBus, replicated_comps{:}];
            
            ConnectP = {[p.Battery.Ports(1) p.DCBus.Ports(1)]};
            
            for i = 1:4
                ConnectP{end+1,1} = [pmsminverters(i).Ports(1),p.DCBus.Ports(1+i)];
                ConnectP{end+1,1} = [pmsminverters(i).Ports(2), pmsmmotors(i).Ports(1)];
                ConnectP{end+1,1} = [pmsmmotors(i).Ports(2), shaftcouplers(i).Ports(1)];
                ConnectP{end+1,1} = [shaftcouplers(i).Ports(2), propellers(i).Ports(1)];
            end
            
            obj = obj@System(components, ConnectP);
            obj.extrinsicProps = Combine([obj.extrinsicProps,obj.StandardEmptyWeight]);
            init_post(obj);
        end
        
        function init_post(obj)
            createModel(obj);
            obj.SymParams = obj.Graph.SymParams;
            setSymQuantities(obj);
            setSimpleModel(obj);
            setLinearDynamicModel(obj);
            setSteadyStateFunc(obj);
            setSteadyStateIOFunc(obj);
            updateSymParamVals(obj);
            calcControllerGains(obj);
        end
        
        function updateSymParamVals(obj, sym_param_vals)
            if nargin == 2
                assert(numel(sym_param_vals) == obj.SymParams.N, 'Incorrect number of sym param vals');
                obj.sym_param_vals = sym_param_vals;
            else
                obj.sym_param_vals = obj.SymParams.Vals;
            end
            obj.SS_QAve = calcSteadyState(obj);
        end
        
        function setSymQuantities(obj)
            sp = obj.SymParams;
            SQ = @(s) symQuantity(s,sp); % Wrapper function for brevity
            
            % Battery Capacity
            batt = obj.Components(1);
            obj.BattCap = SQ(batt.Capacity); % A*s
            
            % Mass
            mass = getProp(obj.extrinsicProps, 'Mass');
            obj.Mass = SQ(mass);
            
            % Hover Thrust
            hover_thrust = 9.81*mass;
            obj.HoverThrust = SQ(hover_thrust); % Total Thrust required to hover
            
            % Hover Speed
            prop_i = find(arrayfun(@(x) isa(x,'Propeller'), obj.Components),1);
            prop = obj.Components(prop_i);
            hover_speed = prop.calcSpeed(hover_thrust/4);
            obj.HoverSpeed = SQ(hover_speed);
        end
        
        function setSimpleModel(obj)
            simple_model = Model();
            
            % STATES
            %dyn_eqns dot(x) = dyn_eqns(x)
            %1.         x1       "Battery"        "Battery SOC"           Abstract
            %2.         x2       "Motor"        "Inductance (i_q)"      Current
            %3.         x3       "Motor"        "Inertia (omega_m)"     AngularVelocity
            %4.         x10      "Shaft"        "Torque (T)"            Torque
            %5.         x14      "Propeller"    "Inertia (omega)"       AngularVelocity
            states = [1,2,3,10,14];
            simple_model.StateDescriptions = ["Battery SOC"; "Motor Current"; "Motor Velocity"; "Shaft Torque"; "Propeller Speed"];
            simple_model.Nx = numel(states);
            
            % INPUTS
            simple_model.Nu = 1;
            simple_model.InputDescriptions = ["Inverter Input"];
            
            
            % OUTPUTS:
            %1-5.  States 1-5
            %6.  Bus Voltage
            %7.  Bus Current
            %8.  Inverter Current (DC)
            %9.  Inverter Voltage (Q)
            %10.  Thrust - Total
            outs = [states 18 19 20 21 28];
            simple_model.Ny = numel(outs);
            simple_model.OutputDescriptions = [simple_model.StateDescriptions; "Bus Voltage"; "Bus Current"; "Inverter Current (DC)"; "Inverter Voltage (q)"; "Total Thrust"];
            
            obj.SymVars = SymVars('Nx', simple_model.Nx, 'Nd', simple_model.Nd, 'Nu', simple_model.Nu);
            
            digits 8 % Set VPA Digits to 8
            
            % Set f_sym
            dyn_eqns = vpa(obj.Model.f_sym(states));
            simple_model.f_sym = combine(vpa(obj.convertSyms(dyn_eqns))); % combine makes sym expression a bit simpler
            
            % Set g_sym
            g_sym_mod = obj.convertSyms(vpa(obj.Model.g_sym(outs)));
            g_sym_mod(end) = g_sym_mod(end)*4; % Convert Thrust per Propeller to Total Thrust
            simple_model.g_sym = combine(g_sym_mod); % combine makes sym expression a bit simpler
            
            simple_model.SymParams = obj.SymParams;
            simple_model.SymVars = obj.SymVars;
            
            simple_model.init();
            
            obj.SimpleModel = simple_model;
        end
        
        function setLinearDynamicModel(obj)
            u_mod = obj.SymVars.u;
            
            % Cut Battery Dynamics out of Dynamic Model
            x_mod = obj.SymVars.x(2:end);
            f_sym_mod = obj.SimpleModel.f_sym(2:end);
            g_sym_mod = obj.SimpleModel.g_sym([5,end]);
            
            A = jacobian(f_sym_mod, x_mod);
            B = jacobian(f_sym_mod, u_mod);
            
            C = jacobian(g_sym_mod, x_mod);
            D = jacobian(g_sym_mod, u_mod);
            
            lm = LinearModel();
            lm.Nx = 4;
            lm.Nu = 1;
            lm.Nd = 0;
            lm.Ny = 1;
            lm.StateDescriptions = obj.SimpleModel.StateDescriptions(2:end);
            lm.InputDescriptions = obj.SimpleModel.InputDescriptions;
            lm.OutputDescriptions = obj.SimpleModel.OutputDescriptions([5,end]);
            sp = copy(obj.SimpleModel.SymParams);
            sp.prepend(symParam('x1',1));
            lm.SymParams = sp;
            sv = obj.SimpleModel.SymVars;
            sv.x = sv.x(2:end);
            lm.SymVars = sv;
            
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
            solve_vars = [sym('u1'); sym('x2'); sym('x3'); sym('x4')];
            subs_vars = [sym('x1'); sym('x5')];
            sp_vars = obj.SymParams.Syms;
            
            vars = {solve_vars, subs_vars, sp_vars};
            
            obj.SteadyState_func = matlabFunction(obj.SimpleModel.f_sym(2:end),'Vars',vars);
        end
        
        function setSteadyStateIOFunc(obj)
            solve_vars = [sym('x2'); sym('x3'); sym('x4'); sym('x5')];
            subs_vars = [sym('u1'); sym('x1')];
            sp_vars = obj.SimpleModel.SymParams.Syms;
            
            vars = {solve_vars, subs_vars, sp_vars};
            
            obj.SteadyStateIO_func = matlabFunction(obj.SimpleModel.f_sym(2:end),'Vars',vars);
        end
        
        function qrss = calcSteadyState(obj, q_bar, opts)
            % Calculates steady-state values at hover, returns QRSteadyState object
            % Default value for q_bar is the average battery soc
            
            arguments
                obj
                q_bar double = [];
                opts.SolverOpts struct = optimset('Display','off');
            end
            
            if isempty(q_bar)
                q_bar = obj.Components(1).Averaged_SOC;
            end
            
            sym_param_vals = obj.sym_param_vals;
            
            hover_speed = double(obj.HoverSpeed, sym_param_vals);
            
            x0 = [0.5; 1; hover_speed; 0.01];
            [x_sol, ~, exit_flag] = fsolve(@(x) obj.SteadyState_func(x,[q_bar;hover_speed],sym_param_vals), x0, opts.SolverOpts);
            if exit_flag <= 0
                error('No solution found');
            elseif x_sol(1) > 1
                error('No valid solution.  Required input exceeds 1');
            elseif x_sol(1) < 0
                error('No valid solution.  Required input must be positive');
            end
            
            qrss = QRSteadyState();
            qrss.q = q_bar;
            qrss.x = [q_bar; x_sol(2:end); hover_speed];
            qrss.u = x_sol(1);
            qrss.y = obj.SimpleModel.CalcG(qrss.x, qrss.u, [], sym_param_vals);
        end
        
        function [T,x_sol,y_sol] = calcSteadyStateIO(obj,u,q, opts)
            % Calculates steady-state thrust given input u, battery SOC q
            % Default value for q is average battery soc
            
            arguments
                obj
                u
                q double = []
                opts.SolverOpts = optimset('Display','off');
            end
            
            if isempty(q)
                q = obj.Components(1).Averaged_SOC;
            end
            
            sym_param_vals = obj.sym_param_vals;
            
            hover_speed = double(obj.HoverSpeed, sym_param_vals);
            
            x0 = [1; hover_speed; 0.01; hover_speed];
            [x_sol, ~, exit_flag] = fsolve(@(x) obj.SteadyStateIO_func(x,[u;q],sym_param_vals), x0, opts.SolverOpts);
            if exit_flag <= 0
                error('No solution found');
            end
            
            x_sol = [q; x_sol];
            u_sol = u;
            y_sol = obj.SimpleModel.CalcG(x_sol, u_sol, [], sym_param_vals);
            T = y_sol(end);
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
            T_max = calcSteadyStateIO(obj, 1);
            T_hover = double(obj.HoverThrust, obj.sym_param_vals);
            
            tr = T_max / T_hover;
        end
        
        function [eta,P] = calcEfficiency(obj,u,q)
            % Define x_power as the power output from that component
            % input_power is chemical power from battery
            
            arguments
                obj
                u double = []
                q double = []
            end
            
            if isempty(u)
                y = obj.SS_QAve.y;
            else
                [~,~,y] = obj.calcSteadyStateIO(u,q);
            end

            bus_power = y(6).*y(7); % bus voltage (DC) * bus current (DC)
            inv_power = 4*y(9).*y(2); % inverter voltage (q) * motor_current (q)
            motor_power = 4*y(3).*y(4);
            
            inv_eta = inv_power ./ bus_power;
            motor_eta = motor_power ./ inv_power;
            
            sys_eta= inv_eta*motor_eta;
            
            eta = struct();
            eta.Inverter = inv_eta;
            eta.Motor = motor_eta;
            eta.Sys = sys_eta;
            
            P = struct();
            P.Bus = bus_power;
            P.Inverter = inv_power;
            P.Motor = motor_power;
        end
        
        function [A,B,C,D] = calcLinearMatrices(obj, qrss)
            if nargin == 1
                qrss = obj.SS_QAve;
            end
            
            [A,B,~,C,D,~] = CalcMatrices(obj.LinearDynamicModel,qrss.x(2:end),qrss.u,[],[qrss.x(1); obj.sym_param_vals]);
        end
        
        function [A,B,C,D] = calcBodyModel(obj)
            m = double(obj.Mass, obj.sym_param_vals);
            
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
        
        function [t_out, y_out, x_g_out, errFlag] = Simulate(obj, r, opts)
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
            sym_param_vals = obj.sym_param_vals;
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
            f_g = @(x_g,u_g) model.CalcF(x_g, u_g, [], sym_param_vals);
            g_g = @(x_g,u_g) model.CalcG(x_g, u_g, [], sym_param_vals);
            
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
            T_bar = qrss.y(end);
            
            if opts.FeedForwardW
                w_bar = qrss.y(5);
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
            
            % Process Results
            x_g_out = x_out(:,i_x_g);
            x_b_out = x_out(:,i_x_b);
            x_r_out = x_out(:,i_x_r);
            
            y_out = x_b_out(:,1);
            
            if opts.PlotResults
                plotResults()
            end
            
            function x_dot = f_sys_cl(t,x)
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
                
                u_sc = (y_vc - x_g(5)) + w_bar; % Propeller Speed error signal, essentially an acceleration command
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
                plot(t_out, [r(t_out), g_r_1(x_r_out),y_out]);
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
                opts.Timeout double = 5
                opts.SimulationBased logical = false
                opts.SimulationOpts cell = {}
            end
            
            if opts.SimulationBased
                [t,~,x_g,errFlag] = Simulate(obj, r, 'Timeout', opts.Timeout, 'PlotResults', false, opts.SimulationOpts{:});
                if errFlag
                    flight_time = NaN;
                else
                    if opts.InterpolateTime
                        q_ = x_g(end-1:end,1);
                        t_ = t(end-1:end);
                        a = -q_(1)/(q_(2)-q_(1));
                        flight_time = (1-a)*t_(1) + a*t_(2);
                    else
                        flight_time = t(end);
                    end
                end
            else
                cap = double(obj.BattCap,obj.sym_param_vals); % A*s
                ave_current = obj.SS_QAve.y(7);
                flight_time = cap/ave_current;
            end    
        end
        %% Helper Functions
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
                
                from{3} = [sym('x11'), sym('x12'), sym('x13')];
                to{3} = repmat(sym('x10'),1,3);
                
                from{4} = [sym('x15'), sym('x16'), sym('x17')];
                to{4} = repmat(sym('x14'),1,3);
            end
            
            sym_bal = subs(sym_unbal,[from{:}], [to{:}]);
        end
        
        function sym_out = convertStates(obj, sym_in)
            from = [sym('x1'); sym('x2'); sym('x3'); sym('x10'); sym('x14')];
            to = obj.SymVars.x;
            sym_out = subs(sym_in, from,to);
        end
    end
end