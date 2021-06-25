classdef ReferenceTrajectory3D < handle
    %REFERENCETRAJECTORIES Summary of this class goes here
    %   Detailed explanation goes here
    properties
        Speed double
    end
    
    properties (SetAccess = private)
        Type string
        
        % Parametrized by u
        Period double
        x function_handle
        y function_handle
        z function_handle
        
        dx function_handle
        dy function_handle
        dz function_handle
        
        % Parametrized by Arc Length
        s double
        x_s double
        y_s double
        z_s double
        
        % Parametrized by Time
        t double
        TimeSeries timeseries
        
    end
    
    methods
        function obj = ReferenceTrajectory3D(traj)
            if nargin
                if isa(traj, "string")
                    switch traj
                        case "Lemniscate"
                        case "AsymetricLemniscate"
                    end
                end
                init(obj);
            end
        end
        
        function line = plot(obj, opts)
            arguments
                obj
                opts.NumPoints double = 100
                opts.ParentAxes matlab.graphics.axis.Axes = matlab.graphics.axis.Axes.empty()
                opts.PlotOpts cell = {}
            end
            pnts = opts.NumPoints;
            u = linspace(0, obj.Period, pnts);
            if ~isempty(opts.ParentAxes)
                label_flag = false;
                ax = opts.ParentAxes;
            else
                label_flag = true;
                f = figure();
                ax = axes(f);
            end
            line = plot3(ax, obj.x(u), -obj.y(u), -obj.z(u), opts.PlotOpts{:});
            line.DisplayName = "Reference Trajectory";
            
            if label_flag
                title(sprintf('Reference Trajectory: %s', obj.Type));
                xlabel('$$x$$', 'Interpreter', 'latex')
                ylabel('$$-y$$', 'Interpreter', 'latex')
                zlabel('$$-z$$', 'Interpreter', 'latex')
                daspect([1 1 1])
                grid on
            end
        end
    end
    
    methods
        function Lemniscate(obj, opts)
            arguments
                obj
                opts.a = 1
                opts.HeightOffset = 0
                opts.PotatoChipHeight = 0
            end
            
            obj.Type = "Lemniscate";
            a = opts.a;
            height_offset = opts.HeightOffset;
            beta = opts.PotatoChipHeight;
            
            x = @(u) a.*sin(u);
            y = @(u) a.*sin(u).*cos(u);
            z = @(u) (height_offset + beta.*(cos(u).^2.*sin(u).^2 + sin(u).^2));
            setXYZ(obj,x,y,z);
            
            obj.Period = 2*pi;
        end
        
        function init(obj)
            setDerivatives(obj);
            setInterpolation(obj);
        end
        
        function ts = setTimeSeries(obj, cycles)
            arguments
                obj
                cycles double = 1
            end
            t_single = obj.s/obj.Speed;
            delta = mean(diff(t_single));
            t = t_single;
            for i = 2:cycles
                t_last = t(end);
                t = [t, (t_last + delta + t_single)];
            end

            data = repmat([obj.x_s; obj.y_s; obj.z_s], [1 cycles]);
            
            ts = timeseries(data, t);
            obj.t = t;
            obj.TimeSeries = ts;
        end
        
        function setXYZ(obj,x,y,z)
            obj.x = x;
            obj.y = @(u) -y(u);
            obj.z = @(u) -z(u);
        end
        
        function setDerivatives(obj)
            u = sym('u');
            r = obj.R(u);
            dr = diff(r);
            
            obj.dx = obj.makeMatlabFunction(dr(1), u);
            obj.dy = obj.makeMatlabFunction(dr(2), u);
            obj.dz = obj.makeMatlabFunction(dr(3), u);
        end
        
        function setInterpolation(obj, res)
            arguments
                obj
                res double = 0.1
            end
            
            obj.s = 0:res:arcLength(obj,0,obj.Period);
            r_points = ArcLengthParametrization(obj, obj.s);
            obj.x_s = r_points(1,:);
            obj.y_s = r_points(2,:);
            obj.z_s = r_points(3,:);
        end
        
        function [r,x,y,z] = R_s(obj,s_query)
            % Uses interpolated data to get position as function of arc
            % length
            method = 'pchip';
            s_query = mod(s_query, obj.s(end));
            x = interp1(obj.s, obj.x_s, s_query, method);
            y = interp1(obj.s, obj.y_s, s_query, method);
            z = interp1(obj.s, obj.z_s, s_query, method);
            
            r = [x;y;z];
        end
        
        function [r,x,y,z] = R_t(obj, t_query)
            method = 'pchip';
            t_query = mod(t_query, obj.t(end));
            x = interp1(obj.t, obj.x_s, t_query, method);
            y = interp1(obj.t, obj.y_s, t_query, method);
            z = interp1(obj.t, obj.z_s, t_query, method);
            
            r = [x;y;z];
        end
            
        
        function [r] =  ArcLengthParametrization(obj,s)
            % S is distance along path
            dist_fun = @(u) arcLength(obj, 0, u);
            r = zeros(3,numel(s));
            for i = 1:numel(s)
                usol = fzero(@(u) dist_fun(u) - s(i), s(i));
                r(:,i) = obj.R(usol);
            end
        end
        
        function r = R(obj, u)
            r = [obj.x(u); obj.y(u); obj.z(u)];
        end
        
        function dr = dR(obj, u)
            dr = [obj.dx(u); obj.dy(u); obj.dz(u)];
        end
        
        function ds = dS(obj, u)
            ds = vecnorm(obj.dR(u),2,1);
        end
        
        function s = arcLength(obj, a,b)
            s = integral(@(u) dS(obj,u), a, b);
        end
    end
    methods(Static)
        function f = makeMatlabFunction(sym,var)
            sv = symvar(sym);
            if isempty(sv) % Constant Function
                f = @(u) repmat(double(sym), size(u));
            else
                f = matlabFunction(sym, 'Vars', var);
            end
        end
    end
end

