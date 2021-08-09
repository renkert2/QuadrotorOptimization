classdef SearchOutput < DiscreteOutput
    %SEARCHOUTPUT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        OptimalIteration (1,1) uint16
        OptimalCounterState (1,1) CounterState
        SortedConfigurations (:,:) ComponentData
        SortedFVals (:,1)
        SortedDistances (:,:)
        SortedComponentIndices (:,:) double
        NormalizedDistances(:,1)
        Weights
        DistanceMode string
        SortMode string
        ComponentNames string
    end
    methods
        function plot(obj)
            figure('Name', 'Nearest Neighbor Search Output')
            
            % Objective Function Plot
            ax_f = subplot(2,1,1);
            plot(obj.SortedFVals);
            title("Processed Objective Function")
            ylabel(string(obj.Objective))
            
            % Distance Plot
            ax_d = subplot(2,1,2);
            sz = size(obj.SortedComponentSets);
            hold on
            co = colororder; % Gets default plot colors as rgb matrix
            for i = 1:sz(2)
                color = co(i,:);
                plot(obj.SortedDistances(:,i), 'Color', color, 'DisplayName', obj.ComponentNames(i));
            end
            
            plot(obj.NormalizedDistances, 'DisplayName', "Norm Distance", 'Color','k');
            hold off
            
            title("Component Distance")
            ylabel('d');
            xlabel("Iteration")
            legend
            %legend([obj.ComponentNames, "Norm Distance"]);
        end
    end
end

