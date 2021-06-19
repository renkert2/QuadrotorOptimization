classdef BodyModel < Model
    % Wraps equations to simulate 3D dynamics of the quadrotor.  Refer to weekly report 6/15/2021 for the development.
    properties (Constant)
        g double = 9.8067
    end
    
    properties (SetAccess = private)
        StateSyms string
        InputSyms string
        I struct % State and Input Indices
    end
    
    properties % Parameters for Body Model
        m compParam
        J compParam
        J_r compParam
        K_T compParam
        K_Q compParam
        l compParam
    end
    
    methods
        function obj = BodyModel(params)
            obj.Nx = 12;
            obj.Nu = 4;
            obj.Nd = 0;
            obj.Ny = 12; % Full State Feedback
            
            obj.I = struct();
            obj.I.x.p = 1:3;
            obj.I.x.v = 4:6;
            obj.I.x.Theta = 7:9;
            obj.I.x.omega = 10:12;
            
            obj.I.u.omega_tilde = 1:4;
            
            obj.StateSyms = ["x","y","z","v_x","v_y","v_z","phi","theta","psi","omega_x","omega_y","omega_z"];
            obj.StateDescriptions = ["X Position", "Y Position", "Z Position", "X Velocity", "Y Velocity", "Z Velocity", "Roll", "Pitch", "Yaw", "X Angular Velocity", "Y Angular Velocity", "Z Angular Velocity"];
            obj.InputSyms = ["omega_tilde_1", "omega_tilde_2", "omega_tilde_3", "omega_tilde_4"];
            obj.InputDescriptions = ["Rotor Speed 1", "Rotor Speed 2", "Rotor Speed 3", "Rotor Speed 4"];
            obj.DisturbanceDescriptions = [];
            obj.OutputDescriptions = obj.StateDescriptions;
            
            if nargin
                obj.m = get(params, "Mass", "Quad Rotor"); % Must be set before calling genMatlabFunctions
                obj.J = get(params, "J", "Quad Rotor");
                
                obj.J_r = get(params, "J_r", "MotorProp");

                obj.K_T = get(params, "K_T", "Propeller");
                obj.K_Q = get(params, "K_Q", "Propeller");
                
                obj.l = get(params, "l", "Frame");
                
                obj.Params = compParam.gatherObjectParams(obj);
                obj.Params.update();
            end
        end
        
        function init(obj)
            obj.setSymVars();
            setF(obj)
            setG(obj)
            init@Model(obj);
        end
        
        function setF(obj)
            x = obj.SymVars.x;
            p = x(obj.I.x.p);
            v = x(obj.I.x.v);
            
            Theta = x(obj.I.x.Theta);
            phi = Theta(1);
            theta = Theta(2);
            psi = Theta(3);
            
            omega = x(obj.I.x.omega);
            
            u = obj.SymVars.u;
            omega_tilde = u;
            
            % Params: J,m,J_r,K_T,K_Q,l
            J = obj.J.Sym_;
            m = obj.m.Sym_;
            J_r = obj.J_r.Sym_;
            K_T = obj.K_T.Sym_;
            K_Q = obj.K_Q.Sym_;
            l = obj.l.Sym_;
            
            e_3 = [0;0;1];
            W = obj.W(phi,theta);
            Rbe = obj.Rbe(phi,theta,psi);
            
            g = obj.g;
            
            [f,tau] = obj.controlEffectiveness(omega_tilde, K_T, K_Q, l);

            G_a = J_r*([-1 1 -1 1]*omega_tilde)*(cross(e_3, omega));

            obj.f_sym = [v;...
                W*omega;...
                g*e_3 + (1/m)*Rbe*(-f*e_3);...
                inv(J)*(-cross(omega, J*omega) + tau + G_a)];   
        end
        
        function update(obj)
        end
    end
    methods (Static)
        function w = W(phi,theta)
            w = [1 tan(theta)*sin(phi) tan(theta)*cos(phi);...
                0 cos(phi) -sin(phi);...
                0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
        end
        
        function rz = Rz(psi)
            rz = [cos(psi) sin(psi) 0;...
                -sin(psi) cos(psi) 0;...
                0 0 1];
        end
        
        function ry = Ry(theta)
                ry = [cos(theta) 0 -sin(theta);...
                0 1 0;...
                sin(theta) 0 cos(theta)];
        end
        
        function rx = Rx(phi)
            rx = [1 0 0;...
                0 cos(phi) sin(phi);...
                0 -sin(phi) cos(phi)];
        end
        
        function rbe = Rbe(phi,theta,psi)
            rbe = BodyModel.Rx(phi)*BodyModel.Ry(theta)*BodyModel.Rz(psi);
        end
        
        function [f,tau] = controlEffectiveness(omega_tilde, K_T, K_Q, l)
            x = [K_T K_T K_T K_T;...
                l*K_T -l*K_T -l*K_T l*K_T;...
                l*K_T l*K_T -l*K_T -l*K_T;...
                K_Q -K_Q K_Q -K_Q]*omega_tilde.^2;
            f = x(1);
            tau = x(2:4);
        end
    end
end

