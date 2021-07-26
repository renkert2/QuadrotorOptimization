classdef PropellerParamFit < handle
    properties
        CD PropellerComponentData
        Fit paramFit
    end
    properties
        ProductLines
        FitClass string = "loess"
        LoessSpan double = 0.5
    end
    
    methods
        function obj = PropellerParamFit(cd)
            if nargin ==1
                obj.CD = cd;
            else
                obj.CD = PropellerComponentData();
            end
            
            % Construct paramFit object
            obj.Fit = paramFit(2,4);
            obj.setFitTypesOpts();
            obj.updateFitData();
        end
        
        function p = get.ProductLines(obj)
            p = obj.CD.ProductLines;
        end

        function set.ProductLines(obj, p)
            obj.CD.ProductLines = p;
            obj.updateFitData();
        end
        
        function set.FitClass(obj, c)
            obj.FitClass = c;
            obj.setFitTypesOpts();
            obj.updateFitModels();
        end
        
        function set.LoessSpan(obj,s)
            if ~isempty(obj.Fit)
                obj.Fit.setSpan(s);
            end
            obj.LoessSpan = s;
        end
        
        function setFitTypesOpts(obj)
            switch obj.FitClass
                case "loess"
                    default_span = obj.LoessSpan;
                    ft = fittype( 'loess' );
                    opts = fitoptions( 'Method', 'LowessFit' );
                    opts.Normalize = 'on';
                    opts.Robust = 'Bisquare';
                    opts.Span = default_span;
                case "interp"
                    ft = fittype('thinplateinterp');
                    opts = fitoptions(ft);
                    opts.Normalize = 'on';
            end
            
            ft_price = fittype( 'poly21' );
            opts_price = fitoptions('Method', 'LinearLeastSquares');
            
            obj.Fit.FitTypes = {ft, ft, ft, ft_price};
            obj.Fit.FitOpts = {opts, opts, opts, opts_price};
        end
        
        function updateFitData(obj)
            [t,~] = table(obj.CD.FilteredCD);
            input_data = [t.D, t.P];
            output_data = [t.k_P, t.k_T, t.Mass, t.Price];
            obj.Fit.setData(input_data, output_data);
            obj.Fit.setBoundary();
            
            updateFitModels(obj);
        end
        function updateFitModels(obj)
            obj.Fit.setModels();
            if ~isempty(obj.Fit.Outputs)
               setOutputDependency(obj.Fit);
           end
        end 
        function save(obj)
            PropellerFit = obj;
            save PropellerFit.mat PropellerFit;
        end
    end
end

