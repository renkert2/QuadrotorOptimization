classdef GAOutput < DiscreteOutput
    %GAOUTPUT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties        
        exitflag
        output
        population
        scores
    end
    
    methods
        function h = counterHistogram(obj_array)
            counters = [obj_array.Counter];
            updates = [counters.update_Counter];
            found_opt = [obj_array.exitflag] == -1;
            updates_found_opt = updates(found_opt);
            h = histogram(updates_found_opt,12);
            
            title("GA Function Evaluations Histogram (Successful Trials)")
            xlabel("Function Evaluations")
            ylabel("Count")
        end
        
        function s = stats(obj_array)
            counters = [obj_array.Counter];
            update_cnts = double([counters.update_Counter]);
            
            s.N_Trials = numel(obj_array);
            found_opt = [obj_array.exitflag] == -1;
            s.N_FoundGlobalOpt = sum(found_opt);
            s.N_ReachedMaxGenerations = sum([obj_array.exitflag] == 0);
            
            s.Mean = mean(update_cnts(found_opt));
            s.StandardDeviation = std(update_cnts(found_opt));
            s.Median = median(update_cnts(found_opt));
            s.Max = max(update_cnts(found_opt));
            s.Min = min(update_cnts(found_opt));
        end
    end
end

