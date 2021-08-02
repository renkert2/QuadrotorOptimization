classdef PropellerSurrogate < handle
    properties
        CD PropellerComponentData
        Fit paramFit
    end
    properties
        ProductLines
    end

    methods
        function obj = PropellerSurrogate(cd)
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
            obj.setFitTypesOpts;
            obj.updateFitData();
        end
 
        function setFitTypesOpts(obj)
            PL = obj.ProductLines;
            if all(PL == "E") || numel(intersect(["E" "F"], PL)) == 2
                ft = fittype( 'loess' );
                opts = fitoptions( 'Method', 'LowessFit' );
                opts.Normalize = 'on';
                opts.Robust = 'Bisquare';
                opts.Span = 0.8;
            elseif all(PL == "MR")
%                 ft = fittype('thinplateinterp');
%                 opts = fitoptions(ft);
                  % opts.Normalize = 'on';
                ft = 'cubicinterp';
                opts = fitoptions(ft); 
                opts.Normalize = 'on';
            elseif all(PL == "SF")
                ft = fittype( 'poly22' );
                opts = fitoptions( ft );
                opts.Normalize = 'on';
            else
                ft = fittype('loess');
                opts = fitoptions( 'Method', 'LowessFit' );
                opts.Normalize = 'on';
                opts.Robust = 'Bisquare';
                opts.Span = 0.8;
            end
            
%             ft_price = fittype( 'poly21' );
%             opts_price = fitoptions('Method', 'LinearLeastSquares');
%             
            obj.Fit.FitTypes = {ft, ft, ft, ft};
            obj.Fit.FitOpts = {opts, opts, opts, opts};
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

