classdef Optimization_BattNp < handle  
    properties
        quad_rotor
        
        X0 double
        lb double
        ub double
    end
    
    properties (Constant)
        X_desc = ["N_p"]
    end
    
    methods
        function obj = Optimization_BattNp()
            obj.init();
        end
        
        function init(obj)
            batt = Battery('Name', 'Battery', 'N_p', symParam('N_p',1)); % 4000mAh, 3S Default Battery, No Dynamics for now
            obj.quad_rotor = QuadRotor('Battery', batt);
            
            obj.X0 = [1];
            
            obj.lb = [0.1];
            obj.ub = [20];
        end

        function [opt_X, opt_flight_time, output] = Optimize(obj, opts)
            arguments
                obj
                opts.SimulationBased = false
                opts.DiffMinChange = 1e-4
                opts.FlightTimeOpts cell = {}
                opts.OptimizationOpts cell = {}
            end
            
            optimopts = optimoptions('fmincon', 'Algorithm', 'sqp','Display', 'iter-detailed', 'PlotFcn', {@optimplotx,@optimplotfval,@optimplotfirstorderopt});
            if opts.SimulationBased
                optimopts = optimoptions(optimopts, 'DiffMinChange', opts.DiffMinChange);
            end
            optimopts = optimoptions(optimopts, opts.OptimizationOpts{:});

            [opt_X, fval, ~, output] = fmincon(@(X) -obj.flightTime(X, 'SimulationBased', opts.SimulationBased, opts.FlightTimeOpts{:}),obj.X0, [], [], [], [], obj.lb, obj.ub, [], optimopts);

            
            opt_flight_time = -fval;
        end
        
        function ft = flightTime(obj,X,opts)
            arguments
                obj
                X
                opts.SimulationBased = false
                opts.RecomputeControlGains = true
                opts.InterpolateTime = true
                opts.Scaled = false
                opts.ScalingFactor = 1500
            end
            
           sym_param_vals = [X];
           
           try
               obj.quad_rotor.updateSymParamVals(sym_param_vals);
               if opts.SimulationBased
                   if opts.RecomputeControlGains
                       obj.quad_rotor.calcControllerGains;
                   end
                   ft = obj.quad_rotor.flightTime('SimulationBased',true, 'InterpolateTime', opts.InterpolateTime);
               else
                   ft = obj.quad_rotor.flightTime('SimulationBased',false);
               end
           catch
               ft = NaN;
           end
           
           if opts.Scaled
               ft = ft/opts.ScalingFactor;
           end
        end
    end
end

