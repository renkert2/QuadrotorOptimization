classdef Optimization_Prop < handle  
    properties
        quad_rotor

        propMassFit propMassFit
        propAeroFit propAeroFit
    end
    
    methods
        function obj = Optimization_Prop()
            obj.init();
        end
        
        function init(obj)
            batt = Battery('Name', 'Battery'); % 4000mAh, 3S Default Battery, No Dynamics for now
            prop = Propeller('Name', 'Propeller', 'k_P', symParam('k_P', 0.03), 'k_T', symParam('k_T', 0.05), 'D', symParam('D', 0.1270), 'M', symParam('M', 0.008), 'J', symParam('J', 2.1075e-05));
            obj.quad_rotor = QuadRotor('Battery', batt, 'Propeller', prop);
            
            load PF_Aero.mat PF_Aero
            obj.propAeroFit = PF_Aero;
            
            load PF_Mass.mat PF_Mass
            obj.propMassFit = PF_Mass;  
        end

        function [opt_X, opt_flight_time, output] = Optimize(obj,opts)
            arguments
                obj
                opts.FixDiameter double = []
                opts.SimulationBased = false
                opts.DiffMinChange = 1e-4
                opts.FlightTimeOpts cell = {}
                opts.OptimizationOpts cell = {}
            end
           
            x0 = obj.propAeroFit.Boundary.X_mean;

            lb = obj.propAeroFit.Boundary.X_lb;
            ub = obj.propAeroFit.Boundary.X_ub;
            
            optimopts = optimoptions('fmincon', 'Algorithm', 'sqp', 'PlotFcn', @boundaryPlotFun, 'Display', 'iter-detailed');
            if opts.SimulationBased
                optimopts = optimoptions(optimopts, 'DiffMinChange', opts.DiffMinChange);
            end
            optimopts = optimoptions(optimopts, opts.OptimizationOpts{:});
            
            if isempty(opts.FixDiameter)
                [opt_X, fval, ~, output] = fmincon(@(X) -obj.flightTime(X,'SimulationBased', opts.SimulationBased, opts.FlightTimeOpts{:}),x0, [], [], [], [], lb, ub, @(x) nlconBoundary(obj,x), optimopts);
            else
                D = opts.FixDiameter;
                x0_mod = fmincon(@(x) obj.nlconBoundary([D;x]), x0(2));
                lb_mod = max(lb(2), fzero(@(x) nlcon([D;x]), lb(2)));
                ub_mod = min(ub(2), fzero(@(x) nlcon([D;x]), ub(2)));
                
                [opt_P, fval, ~, output] = fmincon(@(x) -obj.flightTime_Simple([D;x]),x0_mod, [], [], [], [], lb_mod, ub_mod, [], optimopts);
                opt_X = [D; opt_P];
            end
            
            opt_flight_time = -fval;
            
            function stop = boundaryPlotFun(x,optimValues,state)
                stop = false;
                switch state
                    case 'init'
                        obj.propAeroFit.Boundary.plot
                        hold on
                    case 'iter'
                        plot(x(1),x(2),'.r', 'MarkerSize', 20);
                    case 'done'
                        hold off
                end 
            end
        end
        
        function carpetPlot(obj, opts)
            arguments
                obj
                opts.SimulationBased = false
            end
            
            [d_vals, p_vals] = obj.propAeroFit.Boundary.createGrid(50);
            N_d_vals = numel(d_vals);
            N_p_vals = numel(p_vals);

            flight_times = zeros(N_d_vals, N_p_vals);
            for i = 1:N_d_vals
                d_val = d_vals(i);
                for j = 1:N_p_vals
                    p_val = p_vals(j);
                    x = [d_val; p_val];
                    if obj.propAeroFit.Boundary.isInBoundary(x)
                        flight_times(j,i) =  flightTime(obj, x, 'SimulationBased', opts.SimulationBased);
                    else
                        flight_times(j,i) = NaN;
                    end
                end
            end
            
            surf(d_vals, p_vals, flight_times)
            title('Flight Time')
            zlabel('$$t$$', 'Interpreter', 'latex')
            xlabel('$$D$$', 'Interpreter', 'latex')
            ylabel('$$P$$', 'Interpreter', 'latex')
        end
        
        function ft = flightTime(obj,X, opts)
            arguments
                obj
                X
                opts.SimulationBased = false
                opts.RecomputeControlGains = true
                opts.InterpolateTime = true
                opts.Scaled = false
                opts.ScalingFactor = 1500
            end
            D = X(1);
            [k_P, k_T] = calcPropCoeffs(obj.propAeroFit, X);
            [M,J] = calcMassProps(obj.propMassFit, D);
            
            %   sym_params: [D;J;M;k_P;k_T]
            sym_param_vals = [D;J;M;k_P;k_T];
            
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
        
        function [c,ceq] = nlconBoundary(obj,x)
            c = distToBoundary(obj.propAeroFit.Boundary, x);
            ceq = [];
        end
    end
end

