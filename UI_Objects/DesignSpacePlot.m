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
                obj.DataChildren(i) = plotFit(fits(i));
                title(comps(i));
            end
            
            function p = plotFit(fit)
                ax = plotBoundary(fit, fit.Inputs.Value);
                p = ax.Children(1);
                p.LineStyle = '-';
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
                
    end
end

