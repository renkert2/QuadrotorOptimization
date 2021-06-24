classdef ReferenceTrajectory3D < handle
    %REFERENCETRAJECTORIES Summary of this class goes here
    %   Detailed explanation goes here
    properties
        Speed double
    end
    
    properties (SetAccess = private)
        Type string
        
        % Parametrized by Time
        Period double
        x function_handle
        y function_handle
        z function_handle
        
        dx function_handle
        dy function_handle
        dz function_handle
        
        % Parametrized by Arc Length
        s double
        x_interp double
        y_interp double
        z_interp double
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
            end
            init(obj);
        end
        
        function plot(obj, t, varargin)
            plot3(obj.x(t), -obj.y(t), -obj.z(t), varargin{:});
            title(sprintf('Reference Trajectory: %s', obj.Type));
            xlabel('x')
            ylabel('-y')
            zlabel('-z')
            daspect([1 1 1])
            grid on
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
            
            x = @(t) a.*sin(t);
            y = @(t) a.*sin(t).*cos(t);
            z = @(t) (height_offset + beta.*(cos(t).^2.*sin(t).^2 + sin(t).^2));
            setXYZ(obj,x,y,z);
            
            obj.Period = 2*pi;
        end
        
        function init(obj)
            
        end
        
        function setXYZ(obj,x,y,z)
            obj.x = x;
            obj.y = @(t) -y(t);
            obj.z = @(t) -z(t);
            
            setDerivatives(obj)
        end
        
        function setDerivatives(obj)
            t = sym('t');
            r = obj.R(t);
            dr = diff(r);
            
            obj.dx = obj.makeMatlabFunction(dr(1), t);
            obj.dy = obj.makeMatlabFunction(dr(2), t);
            obj.dz = obj.makeMatlabFunction(dr(3), t);
        end
        
        function setInterpolation(obj, res)
            arguments
                obj
                res double = 0.1
            end
            
            obj.s = 0:res:arcLength(obj,0,obj.Period);
            r_points = ArcLengthParametrization(obj, obj.s);
            obj.x_interp = r_points(1,:);
            obj.y_interp = r_points(2,:);
            obj.z_interp = r_points(3,:);
        end
        
        function [r,x,y,z] = R_s(obj,s_query)
            % Uses interpolated data to get position as function of arc
            % length
            method = 'pchip';
            s_query = mod(s_query, obj.s(end));
            x = interp1(obj.s, obj.x_interp, s_query, method);
            y = interp1(obj.s, obj.y_interp, s_query, method);
            z = interp1(obj.s, obj.z_interp, s_query, method);
            
            r = [x;y;z];
        end
        
        function [r,x,y,z] = R_t(obj, t_query)
            s_query = obj.Speed*t_query;
            [r,x,y,z] = R_s(obj, s_query);
        end
            
        
        function [r] =  ArcLengthParametrization(obj,s)
            % S is distance along path
            dist_fun = @(t) arcLength(obj, 0, t);
            r = zeros(3,numel(s));
            for i = 1:numel(s)
                tsol = fzero(@(t) dist_fun(t) - s(i), s(i));
                r(:,i) = obj.R(tsol);
            end
        end
        
        function r = R(obj, t)
            r = [obj.x(t); obj.y(t); obj.z(t)];
        end
        
        function dr = dR(obj, t)
            dr = [obj.dx(t); obj.dy(t); obj.dz(t)];
        end
        
        function ds = dS(obj, t)
            ds = vecnorm(obj.dR(t),2,1);
        end
        
        function s = arcLength(obj, a,b)
            s = integral(@(u) dS(obj,u), a, b);
        end
    end
    methods(Static)
        function f = makeMatlabFunction(sym,var)
            sv = symvar(sym);
            if isempty(sv) % Constant Function
                f = @(t) repmat(double(sym), size(t));
            else
                f = matlabFunction(sym, 'Vars', var);
            end
        end
    end
end

