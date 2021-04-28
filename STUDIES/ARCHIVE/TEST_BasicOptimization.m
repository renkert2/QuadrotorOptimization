function [opt_N_p, opt_flight_time] = TEST_BasicOptimization()
%% Question: Optimal Battery Size (N_p) For Maximum Flight 

% Design Variables
% - N_s
% - N_p (Maybe)

% Cost Function 
% - Maximize Time spent in Hover = time @ q <= 0
% -- Use odeset('Events', @myEvent)

% Requirements
% - Mass model that depends on component properties (must compatible with symParams)
% -- Exists totally outside of GraphModel
% -- Combining Components -> New Component with mass of combined components being equal 
% -- -> This could be a whole class of Extrinsic Component Properties: (Mass, Cost, Volume, Etc), "extrinsicProps"
% - Controller to get appropriate amount of thrust

%% Graph Model
persistent batt prop single_rotor_model comps
if isempty(single_rotor_model)
    batt = Battery('Name', "Symbolic Battery", 'N_p', symParam('N_p', 1));
    [single_rotor_model, comps] = makeSingleRotor('Battery', batt);
    prop = comps(end);
end

%% Initialize Constants
vehicle_mass = 0.284 - 0.080; % kg
mass_per_cell = 0.08/3;
stc = prop.square_thrust_coeff;
subs_vars = [sym('N_p'), sym('x7')];
assume(0<sym('u1') & sym('u1')<2);

disturbances = zeros(5,1);

sim_opts = odeset('Events', @emptyBattery);
%% Optimization Problem

[opt_N_p, fval] = fminbnd(@(N_p) -flightTime(N_p),0.5,50);
opt_flight_time = -fval;

%% Calculate Steady-State Input Required for Hover
% Goal: Solve for steady-state d_1 given N_p
    function flight_time = flightTime(N_p)
        % Calculate Required Inputs for Hover
        
        battery_mass = 3.*N_p.*mass_per_cell;
        total_mass = vehicle_mass + battery_mass;
        reqd_thrust = 9.81*total_mass;
        reqd_speed = sqrt((reqd_thrust/stc));

        subs_vals = [N_p, reqd_speed];
        dyn_eqns = subs(single_rotor_model.f_sym(2:7), subs_vars, subs_vals);

        [input_sol,x_2,x_3,x_4,x_5,x_6] = solve(dyn_eqns == 0);
        input_sol = double(input_sol(1));

        % System Simulation
        [t,~] = Simulate(single_rotor_model, input_sol, disturbances, N_p, [0 1e6], 'PlotStates', false, 'Solver', @ode23tb, 'SolverOpts', sim_opts);
        
        flight_time = t(end);
    end

    function [value, isterminal, direction] = emptyBattery(~,y)
        % emptyBattery() is an event that terminates the simulation
        % when the battery SOC drains to zero.
        
        value = y(1);
        isterminal = 1;
        direction = 0;
    end
end

