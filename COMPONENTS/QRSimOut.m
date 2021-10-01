classdef QRSimOut
    %QRSIMOUT Wraps Simulink.SimulationOutput for easier access to data
    properties (Constant)
        I_x_p = [1 2 3]; % Position indices from Body Model
    end
    properties
        Data Simulink.SimulationOutput
        Desc struct % Description of various signal names
    end
    
    methods
        function obj = QRSimOut(data,desc)
            obj.Data = data;
            obj.Desc = desc;
        end
        
        function p = plot(obj, arg1, arg2)
            if nargin == 2
                p = plot_(arg1);
            elseif nargin == 3
                s = arg1;
                f = arg2;
                p = plot_(s,f);
            end
            
            function p = plot_(s,f)
                ts = getTS(obj, s);
                t = seconds(ts.Time);
                x = ts.Data;
                x = permute(x,[3 1 2]); % Reshape Time Series array
                x = x(:,:); % Reject third dimension
                desc = obj.Desc.(s);
                if nargin == 2
                    if isstring(f)
                        I = find(obj, s, f);
                        if ~any(I)
                            error("%s not found in %s", f, s);
                        end
                    else
                        I = f;
                    end
                    x = x(:,I);
                    desc = desc(I);
                end
                p = plot(t,x);
                for i = 1:numel(p)
                    p(i).DisplayName = desc(i);
                end
                legend('-DynamicLegend')
            end
        end

        function ax = plotTrajectory(obj_array, BM, opts)
            arguments
                obj_array
                BM BodyModel = BodyModel.empty()
                opts.LegendNames string = string.empty()
                opts.PlotOpts cell = {}
            end
            
            so = obj_array(1);
            y = get(so.Data.yout, 'y_out').Values.Data;
            r = get(so.Data.yout, 'r_out').Values.Data;
            
            if numel(obj_array) > 1
                arg = cell(1,numel(obj_array));
                arg{1} = y;
                for i = 2:numel(obj_array)
                    y_i = get(obj_array(i).Data.yout, 'y_out').Values.Data;
                    arg{i} = y_i;
                end
            else
                arg = y;
            end
            
            ax = BM.plot(arg, 'RefTraj', r, 'TrajectoryNames', opts.LegendNames, opts.PlotOpts{:});
        end

        function plotTrackingError(obj_array, opts)
            arguments
                obj_array
                opts.LegendNames string = string.empty()
                opts.PlotOpts cell = {}
            end

            tiledlayout(2,1);
            nexttile
           
            cum_errors = zeros(numel(obj_array), 1);
            for i = 1:numel(obj_array)
                [cum_err, t, norm_err, ~] = trackingError(obj_array(i));
                cum_errors(i) = cum_err;
                plot(t,norm_err);
                hold on
            end
            hold off
            
            legend(opts.LegendNames)
            title("Distance Error Over Time")
            xlabel("Time $$t$$", 'Interpreter', 'latex')
            ylabel("Norm of Error (m)", 'Interpreter', 'latex')
            
            nexttile
            
            barh(cum_errors)
            title("Integrated Error")
            yticklabels(opts.LegendNames)
            xlabel("Accumulated Error ($$m*s$$)",'Interpreter', 'latex');
        end
        
        function [I] = find(obj, signal, desc)
            sdesc = obj.Desc.(signal);
            I = false(size(sdesc));
            for i = 1:numel(desc)
                I = I | contains(sdesc, desc(i));
            end
        end
        
        function [ts,t,x] = getTS(obj, signal, f)
            s = get(obj.Data.yout, signal);
            ts = s.Values;
            t = seconds(ts.Time);
            x = ts.Data;
            if nargin == 3
                if isstring(f)
                    I = find(obj, signal, f);
                else
                    I = f;
                end
                x = x(:,I);
            end     
        end
    
        function [cum_err, t, norm_err, err] = trackingError(obj)
            so = obj.Data;
            t = so.tout;
            target_trans = get(so.yout, 'r_out').Values.Data;
            trans = get(so.yout, 'y_out').Values.Data(:, obj.I_x_p);
            
            err = target_trans - trans;
            norm_err = vecnorm(err, 2, 2);
            cum_err = trapz(t,norm_err);
        end
    end
end

