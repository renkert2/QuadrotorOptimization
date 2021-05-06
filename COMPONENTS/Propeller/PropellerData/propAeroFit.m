classdef propAeroFit < handle
    %PROPFIT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Data struct
        Interp struct
        Fit struct
        
        Boundary Boundary
        
        PropStruct struct
    end
    
    methods
        function obj = propAeroFit(prop_struct)
            if nargin == 1
                obj.PropStruct = prop_struct;
            else
                load 'propStruct.mat' 'propStruct'
                obj.PropStruct = propStruct;
            end
            obj.init();
        end
        
        function init(obj)
            getData(obj);
            createFits(obj);
        end
        
        function getData(obj)
            propStruct = obj.PropStruct;
            ps_filtered = propStruct([propStruct.Diameter] < 1 & [propStruct.Pitch] < 1);
            diameter = [ps_filtered.Diameter]';
            pitch = [ps_filtered.Pitch]';
            ct = [ps_filtered.meanCt]';
            cp = [ps_filtered.meanCp]';  
            
            bound = Boundary([diameter pitch]);
            obj.Boundary = bound;
            
            Data = struct();
            Data.Diameter = diameter;
            Data.Pitch = pitch;
            Data.ct = ct;
            Data.ct_range = [min(ct) max(ct)];
            Data.cp = cp;
            Data.cp_range = [min(cp) max(cp)];
            obj.Data = Data;                       
        end
        
        function createFits(obj)
            diameter = obj.Data.Diameter;
            pitch = obj.Data.Pitch;
            cp = obj.Data.cp;
            ct = obj.Data.ct;
            
            ct_interp = scatteredInterpolant(diameter,pitch,ct, 'natural');
            ct_fit = fit([diameter pitch], ct, 'Poly33');
            
            cp_interp = scatteredInterpolant(diameter,pitch,cp, 'natural');
            cp_fit = fit([diameter pitch], cp, 'Poly33');
            
            Fit.ct = ct_fit;
            Fit.cp = cp_fit;
            obj.Fit = Fit;
            
            Interp.ct = ct_interp;
            Interp.cp = cp_interp;
            obj.Interp = Interp;
        end
        
        function [k_P, k_T] = calcPropCoeffs(obj, varargin)
            % X = [D ; P]
            cp_fit = obj.Fit.cp;
            ct_fit = obj.Fit.ct;
            
            cp_range = obj.Data.cp_range;
            ct_range = obj.Data.ct_range;
            
            if nargin == 2
                X = varargin{1};
                D = X(1,:);
                P = X(2,:);
            elseif nargin == 3
                D = varargin{1};
                P = varargin{2};
                
                % Ensure D and P are row vectors
                if iscolumn(D)
                    D = D';
                end
                if iscolumn(P)
                    P = P';
                end
            end
            
            in_bounds = obj.Boundary.isInBoundary(D,P);
            if any(~in_bounds)
                out_i = find(~in_bounds);
                out_str = num2str(out_i, '%d, ');
                warning("Points %s Outside propFit Boundary", out_str)
            end
            
            k_P = cp_fit(D,P);
            k_P = max(min(k_P, cp_range(2)),cp_range(1));
            k_T = ct_fit(D,P);
            k_T = max(min(k_T, ct_range(2)),ct_range(1));
        end
        
        function plot(obj, D,P)
            figure(1)
            plot(obj.Fit.ct, [obj.Data.Diameter, obj.Data.Pitch], obj.Data.ct)
            if nargin == 3
                hold on
                plot3(D, P, obj.Fit.ct(D,P),'.r','MarkerSize',20)
                hold off
            end
            zlim([0 max(obj.Data.ct)])
            title('Propeller Thrust Coefficient')
            xlabel('Diameter (m)')
            ylabel('Pitch (m)')
            zlabel('C_T')
            
            
            figure(2)
            plot(obj.Fit.cp, [obj.Data.Diameter, obj.Data.Pitch], obj.Data.cp)
            if nargin == 3
                hold on
                plot3(D, P, obj.Fit.cp(D,P),'.r','MarkerSize',20)
                hold off
            end
            zlim([0 max(obj.Data.cp)])
            title('Propeller Power Coefficient')
            xlabel('Diameter (m)')
            ylabel('Pitch (m)')
            zlabel('C_P')
        end
    end
end

