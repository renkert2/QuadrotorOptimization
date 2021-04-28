classdef BatteryOptimization < handle

    properties (Constant)
        vehicle_mass = 0.284 - 0.080; % kg
        mass_per_cell = 0.08/3;
        disturbances = zeros(14,1);
    end
    
    properties
        quad_rotor
        quad_rotor_model
        comps
        
        stc
        
        N_p_bounds = [1.8,20];
        nominal_q
        
        mission_thrust_factor (1,:) = 1; % mission thrust factor = 1 -> input will be calculated to maintain hover
        mission_thrust_times (1,:) = 0; % mission thrust times -> starting times corresponding to mission_thrust_factors
        input_sched;
    end
    
    methods
        function obj = BatteryOptimization()
            obj.init();
        end
        
        function init(obj)
            batt = Battery('Name', "Symbolic Battery", 'N_p', symParam('N_p', 1));
            obj.quad_rotor = QuadRotor('Battery', batt);
            obj.quad_rotor.createModel;

            prop = obj.quad_rotor.Components(end);
            
            obj.stc = prop.square_thrust_coeff;
            
            nq = solve(batt.V_OCV_curve == 1);
            obj.nominal_q = double(nq(1));
        end
        
        function [x_ss, input_sol] = calcSteadyState(obj, N_p, reqd_thrust)
            reqd_speed = sqrt((reqd_thrust/obj.stc));
            
            subs_vars = [sym('N_p'), sym('x16'), sym('x1')];
            subs_vals = [N_p, reqd_speed, obj.nominal_q];
            
            %dyn_eqns
            %         2       "Battery"      "Capacitance 1"         Voltage        
            %         3       "Battery"      "Capacitance 2"         Voltage        
            %         4       "Motor"        "Inductance (i_q)"      Current        
            %         5       "Motor"        "Inertia (omega_m)"     AngularVelocity
            %         12       "Shaft"        "Torque (T)"            Torque         
            %         16      "Propeller"    "Inertia (omega)"       AngularVelocity
            
            dyn_eqns = obj.quad_rotor.Model.f_sym([2,3,4,5,12,16]);
            dyn_eqns = subs(dyn_eqns, [sym('u2'), sym('u3'), sym('u4'), sym('x6'), sym('x8'), sym('x10')],[sym('u1'), sym('u1'), sym('u1'), sym('x4'), sym('x4'), sym('x4')]);
            dyn_eqns = subs(dyn_eqns, subs_vars, subs_vals);
            
            
            
            assume(0<sym('u1') & sym('u1')<2);
            
            x_ss = solve(dyn_eqns == 0);
            
            input_sol = double(x_ss.u1(1));
        end

        function [t, y, p_f] = Simulate(obj, N_p, input_sched)
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
            [t,y, p_f] = Simulate(obj.quad_rotor.Model, input_sched, obj.disturbances, N_p, [0 5e4], 'PlotStates', false, 'Solver', @ode23tb, 'SolverOpts', sim_opts);
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

