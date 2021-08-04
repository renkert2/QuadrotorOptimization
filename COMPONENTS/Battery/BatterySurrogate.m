classdef BatterySurrogate < handle
    properties
        CD ComponentData
        Fit paramFit
        
        N_s_min double = 0
    end
    properties (Dependent)
        FilteredCD ComponentData
    end
    properties (SetAccess = private)
        CDTable table
    end

    methods
        function obj = BatterySurrogate(cd)
            if nargin ==1
                obj.CD = cd;
            else
                load("BatteryComponentData.mat", "BatteryComponentData"); 
                obj.CD = BatteryComponentData;
            end
            obj.CDTable = table(obj.CD);
            
            % Construct paramFit object
            obj.Fit = paramFit(2,3);
            obj.setFitTypesOpts();
            obj.updateFitData();
        end
        
        function fcd = get.FilteredCD(obj)
            N_s_vals = obj.CDTable.N_s;
            I = N_s_vals >= obj.N_s_min;
            fcd = obj.CD(I);
        end
        
        function set.N_s_min(obj,val)
            obj.N_s_min = val;
            obj.updateFitData();
        end
        
        function setFitTypesOpts(obj)
            ftRs =  fittype( 'a/(y+b)+c*x^(d/y) +k', 'independent', {'x', 'y'}, 'dependent', 'z' );
            foRs = fitoptions( 'Method', 'NonlinearLeastSquares' );
            foRs.Algorithm = 'Levenberg-Marquardt';
            foRs.Display = 'Off';
            foRs.Robust = 'Bisquare';
            foRs.StartPoint = [0.932734068002066 0.597116865382136 0.78528762506439 0.546881519204984 0.278498218867048];

            f = @(a, b, x, y) a*x.*y + b;
            ftM = fittype( f , 'independent', {'x','y'}, 'dependent', {'z'});
            foM = fitoptions('Method', 'NonlinearLeastSquares');
            foM.Robust = 'Bisquare';
            foM.StartPoint = [0.619415251298952 0.613880978184139];
            
            ftP = fittype( 'poly33' );
            foP = fitoptions( 'Method', 'LinearLeastSquares' );
            foP.Normalize = 'on';

%             
%             ft = fittype( 'loess' );
%             opts = fitoptions( 'Method', 'LowessFit' );
%             opts.Normalize = 'on';
%             opts.Robust = 'Bisquare';
%             opts.Span = 0.8;
            
            obj.Fit.FitTypes = {ftRs, ftM, ftP};
            obj.Fit.FitOpts = {foRs, foM, foP};
        end
        
        function updateFitData(obj)
            [t,~] = table(obj.FilteredCD);
            input_data = [t.N_s, t.Q];
            output_data = [t.R_s,t.Mass, t.Price];
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
            BatterySurrogate = obj;
            save BatterySurrogate.mat BatterySurrogate;
        end
    end
end

