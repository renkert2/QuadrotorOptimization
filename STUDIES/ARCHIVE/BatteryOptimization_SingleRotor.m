classdef BatteryOptimization_SingleRotor < handle

    properties (Constant)
        vehicle_mass = 0.284 - 0.080; % kg
        mass_per_cell = 0.08/3;
        disturbances = zeros(5,1);
    end
    
    properties
        single_rotor
        single_rotor_model
        comps
        
        stc
        
        N_p_bounds = [0.3,50];
        
        mission_thrust_factor = 1; % mission thrust factor = 1 -> input will be calculated to maintain hover
        mission_thrust_times = 0; % mission thrust times -> starting times corresponding to mission_thrust_factors
        input_sched;
    end
    
    methods
        function obj = BatteryOptimization()
            obj.init();
        end
        
        function init(obj)
            batt = Battery('Name', "Symbolic Battery", 'N_p', symParam('N_p', 1));
            obj.single_rotor = SingleRotor('Battery', batt);
            obj.single_rotor.createModel;

            prop = obj.single_rotor.Components(end);
            
            obj.stc = prop.square_thrust_coeff;
        end
        
        function [x_ss, input_sol] = calcSteadyState(obj, N_p, reqd_thrust)
            reqd_speed = sqrt((reqd_thrust/obj.stc));
            
            subs_vars = [sym('N_p'), sym('x7')];
            subs_vals = [N_p, reqd_speed];
            
            dyn_eqns = subs(obj.single_rotor.Model.f_sym(2:7), subs_vars, subs_vals);
            
            assume(0<sym('u1') & sym('u1')<2);
            
            x_ss = solve(dyn_eqns == 0);
            
            input_sol = double(x_ss.u1(1));
        end

        function [t,y] = Simulate(obj, N_p, input_sched)
            persistent N_p_last
            
            if nargin == 2
                if ~isempty(N_p_last)
                    go_flag = ~(N_p == N_p_last);
                else
                    go_flag = true;
                end
                
                if isempty(obj.input_sched) || go_flag
                    input_sched = calcInputSchedule(obj, N_p);
                else
                    input_sched = obj.input_sched;
                end
            end
            % System Simulation
            sim_opts = odeset('Events', @emptyBattery);
            [t,y] = Simulate(obj.single_rotor.Model, input_sched, obj.disturbances, N_p, [0 5e4], 'PlotStates', false, 'Solver', @ode23tb, 'SolverOpts', sim_opts);
            N_p_last = N_p;
            
            function [value, isterminal, direction] = emptyBattery(~,y)
                % emptyBattery() is an event that terminates the simulation
                % when the battery SOC drains to zero.
                
                value = y(1);
                isterminal = 1;
                direction = 0;
            end
        end
        
        function [input_sched, reqd_thrust, total_mass] = calcInputSchedule(obj,N_p)
            battery_mass = 3.*N_p.*obj.mass_per_cell;
            total_mass = obj.vehicle_mass + battery_mass;
            reqd_thrust = (9.81*total_mass) / 4; % Assumes 4 Propellers
            
            inputs = zeros(size(obj.mission_thrust_factor));
            
            for i = 1:length(obj.mission_thrust_factor)
                [~,inputs(i)] = obj.calcSteadyState(N_p, obj.mission_thrust_factor(i)*reqd_thrust);
            end
            
            if numel(inputs) == 1
                input_sched = inputs;
            else
                vals = [0 inputs];
                times = obj.mission_thrust_times;
                input_sched = @input_sched_func; 
            end
            
            obj.input_sched = input_sched;
            
            function input = input_sched_func(t)
                input = vals(sum(t>=times') + 1);
            end
        end
            
        function [flight_time] = flightTime(obj, N_p)
            [t,~] = Simulate(obj, N_p);
            flight_time = t(end);
        end

        function [opt_N_p, opt_flight_time] = Optimize(obj)
            [opt_N_p, fval] = fminbnd(@(N_p) -obj.flightTime(N_p),obj.N_p_bounds(1), obj.N_p_bounds(2));
            opt_flight_time = -fval;
        end
    end
end

