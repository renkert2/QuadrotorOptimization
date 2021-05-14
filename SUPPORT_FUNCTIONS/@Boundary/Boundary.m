classdef Boundary
    %BOUNDARY Used to create a 2D boundary around experimental data and evaluate the distance of any 
    % point to the nearest point on the boundary.  Useful for optimization problems in which you 
    % want to constrain the design variables into a polygonal region.
    %TODO
    % - Add smoothing to remove the sharp creases in the Distance function that may give fmincon trouble
    %   Detailed explanation goes here
    
    properties
        Data (:,:) double
        BoundaryPoints (:,:) double
    end
    
    properties (SetAccess = private)
        X_lb
        X_ub
        X_mean
    end
    
    properties (Access = private)
        N_vars
    end
    
    methods
        function obj = Boundary(data)
            % - data: each variable is a column, i.e. [X], [X Y], or [X Y Z]
            N_vars = size(data,2);
            obj.N_vars = N_vars;
            
            obj.Data = data;
            obj.X_lb = min(obj.Data)';
            obj.X_ub = max(obj.Data)';
            obj.X_mean = mean(obj.Data)';
            
            bound_flag = false;
            switch N_vars
                case 2
                    bound = boundary(obj.Data(:,1), obj.Data(:,2), 0);
                    bound_flag = true;
                case 3
                    bound = boundary(obj.Data(:,1), obj.Data(:,2), obj.Data(:,3), 0);
                    bound_flag = true;
            end
            
            if bound_flag
                obj.BoundaryPoints = obj.Data(bound,:);
            end
        end
        
        function d = distToBoundary(obj,varargin)
            switch obj.N_Vars
                case 1
                    X = varargin{1};
                    in_bounds = (obj.X_lb <= X) && (X <= obj.X_ub);
                    
                    d = min(abs(X - [obj.X_lb obj.X_ub]));
                    if in_bounds
                        d = -d;
                    end
                case 2
                    if nargin == 2
                        X = varargin{1};
                        X1 = X(1,:);
                        X2 = X(2,:);
                    elseif nargin == 3
                        X1 = varargin{1};
                        X2 = varargin{2};
                    end
                    
                    d = obj.p_poly_dist(X1,X2,obj.BoundaryPoints(:,1), obj.BoundaryPoints(:,2),true)';
                otherwise
                    d = NaN;
            end
        end
        
        function l = isInBoundary(obj,varargin)
            switch obj.N_vars
                case 1
                    l = (obj.X_lb <= X) && (X <= obj.X_ub);
                case 2
                    d = distToBoundary(obj,varargin{:});
                    l = (d <= 0);
            end
        end
        
        function [x1_vals, x2_vals] = createGrid(obj, res)
            % Creates grid of res number of points along x_1 and x_2 dimensions.
            % Useful for surf plots over valid domain
            x1_vals = linspace(obj.X_lb(1), obj.X_ub(1), res);
            x2_vals = linspace(obj.X_lb(2), obj.X_ub(2), res);
        end
        
        function plot(obj, x1, x2)
            scatter(obj.Data(:,1), obj.Data(:,2))
            hold on
            plot(obj.BoundaryPoints(:,1), obj.BoundaryPoints(:,2))
            hold off
            
            if nargin == 3
                hold on
                plot(x1,x2, '.r', 'MarkerSize', 20)
                hold off
            end
            
            
            title("Boundary")
            xlabel('X_1')
            ylabel('X_2')
        end
    end
    
    methods (Static)
        [d_min, varargout] = p_poly_dist(xp, yp, xv, yv, varargin);
    end
end

