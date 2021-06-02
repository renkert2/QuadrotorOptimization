classdef SearchOutput
    %SEARCHOUTPUT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Objective (1,1) OptimObjectives
        SortedComponentSets (:,:) ComponentData
        SortedFVals (:,1) double
        SortedDistances (:,:) double
        NormalizedDistances(:,1) double
        Weights
        DistanceMode string
        ModifiedParameters (:,1) compParam
        ComponentNames string
    end
    
    properties (Dependent)
        OptimalComponentSet (1,:) ComponentData
        OptimalFVal (1,1) double
    end
    
    methods
        function v = get.OptimalComponentSet(obj)
            v = obj.SortedComponentSets(1,:);
        end
        function v = get.OptimalFVal(obj)
            v = obj.SortedFVals(1);
        end
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

