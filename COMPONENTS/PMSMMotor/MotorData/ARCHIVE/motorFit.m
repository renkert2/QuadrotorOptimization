classdef motorFit < handle
    %MOTORFIT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Data struct
        DataTable table
        X_desc string = ["kV";"Rm"]
        Y_desc string = ["Weight"; "Diameter"]
        Y_min double
        Y_max double
        Fit struct
        Boundary Boundary
    end
    
    properties
       x_data
       weight_data
       diam_data
    end
    
    methods % Init Methods
        function obj = motorFit(data_struct)
            obj.Data = data_struct;
        end
        
        function init(obj)
            getData(obj)
            createFits(obj)
        end
        
        function getData(obj)
            constants = [obj.Data.CONSTANTS];
            specs = [obj.Data.SPECS];
            
            kV = [constants.kV]';
            kV(kV > 5000 | kV < 1) = NaN;
            
            Rm = [constants.Rm]';
            Rm(Rm > 1 | Rm < 0) = NaN;
            
            Io = [constants.Io]';
            Io(Io > 10 | Io < 0) = NaN;
            
            Weight = [specs.Weight]';
            Weight(Weight > 10 | Weight < 0) = NaN;
            obj.Y_min(1,1) = min(Weight);
            obj.Y_max(1,1) = max(Weight);
            
            Diameter = [specs.Diameter]';
            Diameter(Diameter > 0.3 | Diameter < 0) = NaN;
            obj.Y_min(2,1) = min(Diameter);
            obj.Y_max(2,1) = max(Diameter);
            
            Length = [specs.Length]';
            Length(Length > 0.3 | Length < 0) = NaN;
            
            obj.DataTable = table(Diameter,Length,Weight,kV,Rm,Io);
        end
        
        function createFits(obj, opts)
            
            x_data = [obj.DataTable.kV, obj.DataTable.Rm];
            x_data_filt = x_data(~any(isnan(x_data),2),:);
            obj.x_data = x_data_filt;
            obj.Boundary = Boundary(x_data_filt);
            
             % Weight Fit
            data = [x_data obj.DataTable.Weight];
            data = data(~any(isnan(data),2),:);
            obj.weight_data = data;
            
%             ft = fittype( 'loess' );
%             opts = fitoptions( 'Method', 'LowessFit' );
%             opts.Normalize = 'on';
%             opts.Robust = 'Bisquare';
%             opts.Span = 0.4;

            ft = fittype( '(a/(x+f))^(d) + (b/(y+g))^(e) + c', 'independent', {'x', 'y'}, 'dependent', 'z' );
            opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
            opts.Display = 'Off';
            opts.Lower = [0 0 0 0 0 0 0];
            opts.Robust = 'Bisquare';
            opts.StartPoint = [0.781043314488573 0.978966203552797 0.845317915312609 0.547008892286345 0.296320805607773 0.7447 0.188955015032545];
            opts.Upper = [Inf Inf Inf 3 3 0.1 0.1];
            
            weight_fit = fit(data(:,1:2),data(:,3), ft,opts);
            
            % Diam Fit
             data = [x_data obj.DataTable.Diameter];
             data = data(~any(isnan(data),2),:);
             obj.diam_data = data;
             
%             ft = fittype( 'loess' );
%             opts = fitoptions( 'Method', 'LowessFit' );
%             opts.Normalize = 'on';
%             opts.Robust = 'Bisquare';
%             opts.Span = 0.75;

            ft = fittype( '(a/(x+f))^(d) + (b/(y+g))^(e) + c', 'independent', {'x', 'y'}, 'dependent', 'z' );
            opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
            opts.Display = 'Off';
            opts.Lower = [0 0 0 0 0 0 0];
            opts.Robust = 'Bisquare';
            opts.StartPoint = [0.817627708322262 0.794831416883453 0.644318130193692 0.378609382660268 0.811580458282477 0.532825588799455 0.350727103576883];
            opts.Upper = [Inf Inf Inf 3 3 1000 1];

            diam_fit = fit(data(:,1:2),data(:,3), ft,opts);
            
            fits = struct();
            fits.Weight = weight_fit;
            fits.Diameter = diam_fit;
            
            obj.Fit = fits;
        end     
    end
    
    methods % Working Methods
        function [M,J,D] = calcMotorProps(obj,varargin)
            if nargin == 2
                X = varargin{1};
                kV = X(1,:);
                Rm = X(2,:);
            elseif nargin == 3
                kV = varargin{1};
                Rm = varargin{2};
                
                if iscolumn(kV)
                    kV = kV';
                end
                if iscolumn(Rm)
                    Rm = Rm';
                end
            end
            
            in_bounds = obj.Boundary.isInBoundary(kV,Rm);
            if any(~in_bounds)
                out_i = find(~in_bounds);
                out_str = num2str(out_i, '%d, ');
                warning("Points %s Outside motorFit Boundary", out_str)
            end
            
            M = obj.Fit.Weight(kV,Rm);
            M = max(min(M,obj.Y_max(1)),obj.Y_min(1));
            
            D = obj.Fit.Diameter(kV,Rm);
            D = max(min(D,obj.Y_max(2)),obj.Y_min(2));
            
            J = obj.calcInertia(M,D);
        end
        
        function plotFits(obj)
            figure
            plot(obj.Fit.Weight, obj.weight_data(:,1:2), obj.weight_data(:,3))
            zlim([0 max(obj.weight_data(:,3))])
            title('Motor Mass')
            xlabel('kV (rpm/V)')
            ylabel('Rm (ohms)')
            zlabel('M (kg)')
            
            
            figure
            plot(obj.Fit.Diameter, obj.diam_data(:,1:2), obj.diam_data(:,3))
            zlim([0 max(obj.diam_data(:,3))])
            title('Motor Diameter')
            xlabel('kV (rpm/V)')
            ylabel('Rm (ohms)')
            zlabel('D (m)')
        end
    end
    
    methods (Static)
        function K_t = kVToKt(kV)
            % Convert kV in rpm/V to torque constant Kt in N*m/A = V/(rad/s)
            kV_radps = kV*(2*pi)/60;
            K_t = 1/kV_radps;
        end
        
        function J = calcInertia(M,D)
            % Estimate mass of rotor as M/2;
            % R = D/2;
            % J = MR^2 (Hoop moment of inertia)
            
            J = (M/2).*(D/2).^2;
        end
    end
end

