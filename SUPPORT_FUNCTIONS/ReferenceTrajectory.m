classdef ReferenceTrajectory < handle
    %REFERENCETRAJECTORIES Summary of this class goes here
    %   Detailed explanation goes here
    properties
        Speed double
        R function_handle
    end
    
    methods
        function obj = ReferenceTrajectory(traj)
            if nargin
                if isa(traj, "string")
                    switch traj
                        case "ConstantStep"
                            obj.R = obj.genConstantStep();
                        case "Pulses"
                            obj.R = obj.genPulses();
                        case "Lemniscate"
                        case "AsymetricLemniscate"
                    end
                end
            end
        end
        
        function plot(obj, t_range, varargin)
            fplot(obj.R,  t_range, varargin{:})
            title('Reference Trajectory')
           xlabel('t')
           ylabel('Reference Height')
        end
    end
    
    methods
        function r = genConstantStep(obj,height)
            arguments
                obj
                height double = 1
            end
            
            r = @(t) (t>0).*height;
            obj.R = r;
        end
        
        function r = genPulses(obj,h,t_high,t_low,N,offset)
            arguments
                obj
                h double = 29
                t_high double = 2*60
                t_low double = 60
                N double = 6
                offset double = 1
            end
            P = t_high + t_low;
            r = @(t) (t>0).*(h*(mod(t,P)<t_high).*(t<=P*N) + offset);
            obj.R = r;
        end
        
        function r = genLemniscate(obj, opts)
            arguments
                obj
                opts.a = 1
                opts.HeightOffset = 0
                opts.PotatoChipFactor = 0
            end
            
            a = opts.a;
            height_offset = opts.HeightOffset;
            beta = opts.PotatoChipFactor;
            r = @(t) [a*sin(t), a*sin(t)*cos(t), -(height_offset + beta*sqrt(a^2*cos(t)^2*sin(t)^2 + a^2*sin(t)^2))]';
            
            obj.R = r;
        end
    end
end

