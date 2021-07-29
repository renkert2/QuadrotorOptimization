classdef PropellerComponentData < handle
    % Wrapper for props in ComponentData that allow for filtering by
    % product line
    properties (Constant)
        AllProductLines = ["E","F","MR","SF"]
    end
    properties
        ProductLines = ["E"]
        AllCD ComponentData
    end
    properties (Access = private)
        SKUs
    end
    properties (Dependent)
        FilteredCD ComponentData
    end
    
    methods
        function obj = PropellerComponentData()
            addpath('APC_DATA')
            PropellerComponentData = ComponentData.importFromJSON('PropellerComponentDatabase.json');
            obj.AllCD = PropellerComponentData;
            obj.SKUs = vertcat(obj.AllCD.SKU);
        end
        
        function cd = get.FilteredCD(obj)
            cd = obj.getFilteredCD();
        end
        
        function cd = getFilteredCD(obj, pl)
            if nargin == 1
                pl = obj.ProductLines;
            end
            I = false(size(obj.SKUs));
            for i = 1:numel(pl)
                I = I | matches(obj.SKUs,PropellerComponentData.plPattern(pl(i)));
            end
            cd = obj.AllCD(I);
        end
        
        function save(obj)
            PropellerComponentData = obj;
            save PropellerComponentData.mat PropellerComponentData
        end
        
        function ProductLinePlot(obj, outs)
            if nargin == 1
                outs = ["k_P", "k_T", "Mass", "Price"];
            end
            t = tiledlayout(1,numel(outs));
            colors = colororder;
            lines = obj.AllProductLines;
            
            for i = 1:numel(obj.AllProductLines)
                tbls{i} = table(getFilteredCD(obj, lines(i)));
            end
            
            for j = 1:numel(outs)
                nexttile(t,j);
                for i = 1:numel(lines)
                    tbl = tbls{i};
                    plot3(tbl.D, tbl.P, tbl.(outs(j)),'.','Color',colors(i,:));
                    zlabel(outs(j))
                    hold on
                end
                xlabel("D (m)", 'Interpreter', 'latex')
                ylabel("P (m)", 'Interpreter', 'latex')
                legend(lines)
                hold off
            end
        end
    end
    
    methods (Static)
        function p = plPattern(pl)
            p = lettersPattern()+digitsPattern(5)+pl;
        end
    end
end

