classdef QRSimOut
    %QRSIMOUT Wraps Simulink.SimulationOutput for easier access to data
    
    properties
        Data Simulink.SimulationOutput
        Desc struct % Description of various signal names
    end
    
    methods
        function obj = QRSimOut(data,desc)
            obj.Data = data;
            obj.Desc = desc;
        end
        
        function p = plot(obj, arg1, arg2)
            if nargin == 2
                p = plot_(arg1);
            elseif nargin == 3
                s = arg1;
                f = arg2;
                p = plot_(s,f);
            end
            
            function p = plot_(s,f)
                ts = getTS(obj, s);
                t = seconds(ts.Time);
                x = ts.Data;
                desc = obj.Desc.(s);
                if nargin == 2
                    if isstring(f)
                        I = find(obj, s, f);
                        if ~any(I)
                            error("%s not found in %s", f, s);
                        end
                    else
                        I = f;
                    end
                    x = x(:,I);
                    desc = desc(I);
                end
                p = plot(t,x);
                for i = 1:numel(p)
                    p(i).DisplayName = desc(i);
                end
                legend('-DynamicLegend')
            end
        end
        
        function [I] = find(obj, signal, desc)
            sdesc = obj.Desc.(signal);
            I = false(size(sdesc));
            for i = 1:numel(desc)
                I = I | contains(sdesc, desc(i));
            end
        end
        
        function ts = getTS(obj, signal)
            s = get(obj.Data.yout, signal);
            ts = s.Values;
        end
    end
end

