classdef Counter < CounterParent & handle
    %COUNTER Handle class Counter object
    methods
        function increment(obj, cntr)
            for i = 1:numel(cntr)
                prop = cntr(i)+"_Counter";
                obj.(prop) = obj.(prop) + 1;
            end
        end
        
        function s = getState(obj)
            props = string(properties("CounterParent"))';
            s = CounterState();
            for p = props
                s.(p) = obj.(p);
            end
        end
    end
end

