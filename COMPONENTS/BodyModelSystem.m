classdef BodyModelSystem < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    properties (Constant)
        Name char = 'BodyModel_Simulink'
    end
    
    properties
        BM BodyModel
        LM LinearModel
        Wks Simulink.ModelWorkspace
        
        x_ss double
        u_ss double
    end
    
    methods
        function obj = BodyModelSystem(arg)
            if isa(arg, 'BodyModel')
                obj.BM = arg;
            elseif isa(arg, 'QuadRotor')
                obj.BM = BodyModel(arg.Params);
            elseif isa(arg, 'compParam')
                obj.BM = BodyModel(arg);
            end
        end
        
        function sim_h = init(obj)
            obj.LM = getLinearModel(obj.BM);
            
            sim_h = makeSimulinkModel(obj.BM, obj.Name);
            obj.Wks = get_param(sim_h, 'ModelWorkspace');
            
            obj.x_ss = zeros(12,1);
            obj.u_ss = obj.BM.calcSteadyStateInput(obj.x_ss, [], repmat(580,4,1));
            
            setLowPass(obj);
            setLQR(obj);
            setP_des(obj);
        end
        
        function [t,y] = Simulate(obj)
            simOut = sim(obj.Name);
            t = simOut.tout;
            y = simOut.yout;
        end
        
        function setP_des(obj, p_des)
            arguments
                obj
                p_des double = [10;10;-10]
            end
            
            x_des  = zeros(12,1);
            x_des(obj.BM.I.x.p) = p_des;
            
            assignin(obj.Wks, 'x_des', x_des)
        end
        
        function setLQR(obj, rho)
            arguments 
                obj
                rho (1,1) double = 0.01
            end
            
            [A,B] = CalcMatrices(obj.LM,obj.x_ss,obj.u_ss,[]);
            
            Q = eye(obj.BM.Nx);
            R = rho*eye(obj.BM.Nu);
            
            N = [0];
            
            K_lqr = lqr(A,B,Q,R,N);
            assignin(obj.Wks, 'K_lqr', K_lqr);
        end
        
        function setLowPass(obj)
            k = 1e6;
            A_m = -k.*eye(12);
            B_m = k.*eye(12);
            C_m = eye(12);
            D_m = zeros(12);
            assignin(obj.Wks, 'A_m', A_m);
            assignin(obj.Wks, 'B_m', B_m);
            assignin(obj.Wks, 'C_m', C_m);
            assignin(obj.Wks, 'D_m', D_m);
        end
        
        function set.u_ss(obj, val)
            obj.u_ss = val;
            if ~isempty(obj.Wks)
                assignin(obj.Wks, 'u_ss', obj.u_ss);
            end
        end   
    end
end

