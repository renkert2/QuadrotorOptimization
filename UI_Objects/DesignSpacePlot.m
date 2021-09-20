classdef DesignSpacePlot < handle
    %BOUNDARYPLOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        QR QuadRotor
        Parent

    end
    properties (SetAccess = private)
        T matlab.graphics.layout.TiledChartLayout
        DataChildren matlab.graphics.chart.primitive.Line
        DataMarkers matlab.graphics.chart.primitive.Line
        StartMarker matlab.graphics.chart.primitive.Line
        EndMarker matlab.graphics.chart.primitive.Line
        Fits
    end
    
    methods
        function obj = DesignSpacePlot(QR, Parent)
            obj.QR = QR;
            if nargin == 1
                obj.Parent = figure();
            else
                obj.Parent = Parent;
            end
            init(obj);
        end
        
        function init(obj)
            obj.T = tiledlayout(obj.Parent, 1,3);
            comps = ["Battery", "Motor", "Propeller"];
            fits = arrayfun(@(c) obj.QR.PT.(c).Fit, comps);
            obj.Fits = fits;
            for i = 1:numel(fits)
                nexttile(obj.T,i)
                [obj.DataChildren(i), b, obj.StartMarker(i), obj.EndMarker(i)] = plotFit(fits(i));
                title(comps(i));
            end
            lg = legend([obj.DataChildren(1); b; obj.StartMarker(1); obj.EndMarker(1)], 'Orientation', 'Horizontal');
            lg.Layout.Tile = 'South';
            
            function [p,b,s,f] = plotFit(fit)
                ax = plotBoundary(fit, fit.Inputs.Value);
                
                p = ax.Children(1);
                p.LineStyle = '-';
                p.Color = 'black';
                p.Marker = '.';
                p.MarkerSize = 5;
                p.DisplayName = 'Iterations';
                
                b = ax.Children(2);
                b.DisplayName = 'Constraint';
                
                s = matlab.graphics.chart.primitive.Line;
                s.Parent = ax;
                s.LineStyle = 'none';
                s.Marker = 'o';
                s.MarkerSize = 5;
                s.MarkerEdgeColor = [1 165/255 0];
                s.LineWidth = 2;
                s.DisplayName = 'Initial Point';
                
                f = matlab.graphics.chart.primitive.Line;
                f.Parent = ax;
                f.LineStyle = 'none';
                f.Marker = 'o';
                f.MarkerSize = 5;
                f.MarkerEdgeColor = 'green';
                f.LineWidth = 2;
                f.DisplayName = 'Optimal Point';
            end
        end
        
        function update(obj)
            fields = ["XData", "YData", "ZData"];
            
            for i = 1:numel(obj.DataChildren)
                fit = obj.Fits(i);
                child = obj.DataChildren(i);
                new_vals = [fit.Inputs.Value];
                for j = 1:numel(new_vals)
                    child.(fields(j)) = [child.(fields(j)) new_vals(j)];
                end
            end
        end
        
        function addStartMarker(obj)
            fields = ["XData", "YData", "ZData"];
            
            for i = 1:numel(obj.StartMarker)
                fit = obj.Fits(i);
                child = obj.StartMarker(i);
                new_vals = [fit.Inputs.Value];
                for j = 1:numel(new_vals)
                    child.(fields(j)) = [child.(fields(j)) new_vals(j)];
                end
            end
        end 
        
        function addEndMarker(obj)
            fields = ["XData", "YData", "ZData"];
            
            for i = 1:numel(obj.EndMarker)
                fit = obj.Fits(i);
                child = obj.EndMarker(i);
                new_vals = [fit.Inputs.Value];
                for j = 1:numel(new_vals)
                    child.(fields(j)) = [child.(fields(j)) new_vals(j)];
                end
            end
        end 
    end
end

