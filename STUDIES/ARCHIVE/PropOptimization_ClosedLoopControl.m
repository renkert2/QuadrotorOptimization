classdef PropOptimization_ClosedLoopControl < handle  
    properties
        quad_rotor

        propMassFit propMassFit
        propAeroFit propAeroFit
        
        r_sym symfun % Symbolic reference signal - height
    end
    
    properties (Access = private)
        r_ matlabFunction
        int_r_ matlabFunction
        dot_r_ matlabFunction
    end
    
    methods
        function obj = PropOptimization_ClosedLoopControl()
            obj.init();
        end
        
        function init(obj)
            batt = Battery('Name', 'Battery'); % 4000mAh, 3S Default Battery, No Dynamics for now
            prop = Propeller('Name', 'Propeller', 'k_P', symParam('k_P', 0.03), 'k_T', symParam('k_T', 0.05), 'D', symParam('D', 0.1270));
            obj.quad_rotor = QuadRotor('Battery', batt, 'Propeller', prop);
            
            load PF_Aero.mat PF_Aero
            obj.propAeroFit = PR_Aero;
            
            load PF_Mass.mat PF_Mass
            obj.propMassFit = PF_Mass;
            
            syms r(t);
            r(t) = 1-exp(-t);
            obj.r_sym = r;   
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
        
        function set.r_sym(obj, r)
            obj.r_sym = r;
            obj.r_ = matlabFunction(r);
            
            int_r = int(r);
            obj.int_r_ = matlabFunction(int_r);
            
            dot_r = diff(r);
            obj.dot_r_ = matlabFunction(dot_r);
        end
    end
end

