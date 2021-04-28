classdef propAeroFit
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
        function [k_P, k_T] = calcPropCoeffs(obj, varargin)
            % X = [D ; P]
            cp_fit = obj.Fit.cp;
            ct_fit = obj.Fit.ct;
            
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
            k_T = ct_fit(D,P);
        end
    end
end

