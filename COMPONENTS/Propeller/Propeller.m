classdef Propeller < Component
    %PROPELLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        D compParam = compParam('D', 0.1780, 'Unit', "m") % Propeller Diameter - m
        P compParam =  compParam('P', 0.0673, 'Unit', "m") % Propeller Pitch - m
        
        % k_Q and k_T Parameters from:
        % Illinois Volume 2 Data
        % Static Test for C_t and C_p 
        % Note: C_q (drag coeff) = C_p (power coeff) / (2 * pi)
        % - https://m-selig.ae.illinois.edu/props/volume-2/data/da4002_5x2.65_static_1126rd.txt
        
        % Nominal Parameters from "magf_7x4" in propStruct
        k_P compParam = compParam('k_P',  0.0411) % Power coefficient - k_P = 2*pi*k_Q, speed in rev/s
        k_T compParam = compParam('k_T', 0.0819) % Thrust coefficient - N/(s^2*kg*m^2), speed in rev/s.
        
        Mass extrinsicProp = extrinsicProp('Mass', 0.008, 'Unit', "kg")
        J compParam = compParam('J', 2.1075e-05,'Unit', "kg*m^2") % Rotational Inertia - kg*m^2 from "Stabilization and Control of Unmanned Quadcopter (Jiinec)
        
        rho compParam = compParam('rho', 1.205, 'Unit', "kg/m^3", 'Description', "Air Density") % Air Density - kg/m^3
    end
    
    properties (SetAccess = private) % Dependent compParams
        k_Q compParam = compParam('k_Q', NaN, 'Unit', "N/(s*kg*m)", 'Description', "Drag Torque Coefficient", 'Dependent', true) % Drag Torque Coefficient - N/(s*kg*m)=1/s^4, speed in rev/s.
        K_Q compParam = compParam('K_Q', NaN, 'Unit', "N*m/(rad/s)^2", 'Description', "Lumped Drag Coefficient", 'Dependent', true)%square_drag_coeff %coefficient in front of speed^2 term, N*m/(rad/s)^2.
        K_T compParam = compParam('K_T', NaN, 'Unit', "N/(rad/s)^2", 'Description', "Lumped Thrust Coefficient", 'Dependent', true)%square_thrust_coeff %coefficient in front of speed^2 term, N/(rad/s)^2. 
    end
    
    properties (SetAccess = private)
       Fit paramFit
    end
    
    methods
        function k_Q = lumpedToTorqueCoeff(obj, lumped_torque_coeff)
            k_Q = lumped_torque_coeff/(obj.rho*obj.D.^5); % rev/s
        end
        
        function k_T = lumpedToThrustCoeff(obj, lumped_thrust_coeff)
            k_T = lumped_thrust_coeff/(obj.rho*obj.D.^4); % rev/s
        end
        
        function thrust = calcThrust(obj, speed)
            thrust = obj.K_T*(speed.^2);
        end
        
        function speed = calcSpeed(obj, thrust)
            speed = sqrt(thrust/obj.K_T);
        end
        
        function torque = calcTorque(obj, speed)
            torque = obj.K_Q*(speed.^2);
        end
        
        function init(obj)
            load PropellerFit.mat PropellerFit;
            PropellerFit.Inputs = [obj.D, obj.P];
            PropellerFit.Outputs = [obj.k_P, obj.k_T, obj.Mass];
            PropellerFit.setOutputDependency;
            obj.Fit = PropellerFit;
            
            obj.J.setDependency(@Propeller.calcInertia, [obj.Mass, obj.D]);
            
            obj.k_Q.setDependency(@Propeller.calcTorqueCoefficient, [obj.k_P]);
            obj.K_Q.setDependency(@Propeller.calcLumpedTorqueCoefficient, [obj.k_Q, obj.rho, obj.D]);
            obj.K_T.setDependency(@Propeller.calcLumpedThrustCoefficient, [obj.k_T, obj.rho, obj.D]);
        end
    end
    
    methods (Static)
        function k_rad_per_s = convCoeffToRadPerSec(k_rev_per_s)
            % Converts lumped coefficients of speed^2 term from rev/s to rad/s
            % w' = speed in rad/s
            % w = speed in rev/s
            % Thrust = k_rev_per_s * w^2 = k_rad_per_s * (w')^2
            
            k_rad_per_s = k_rev_per_s / (2*pi)^2;
        end
        
        function k_rev_per_s = convCoeffToRevPerS(k_rad_per_s)
            % Converts lumped coefficients of speed^2 term from rev/s to rad/s
            % w' = speed in rad/s
            % w = speed in rev/s
            % Thrust = k_rev_per_s * w^2 = k_rad_per_s * (w')^2
            
            k_rev_per_s = k_rad_per_s * (2*pi)^2;
        end
        
        function J = calcInertia(M,D)
            J = 1/12*M.*D.^2;
        end
        
        function k_Q = calcTorqueCoefficient(k_P)
            k_Q = k_P / (2*pi);
        end
        
        function K_Q = calcLumpedTorqueCoefficient(k_Q, rho, D)
            sdc = k_Q*rho*D^5; % rev/s
            K_Q = Propeller.convCoeffToRadPerSec(sdc); % rad/s
        end
        
        function K_T = calcLumpedThrustCoefficient(k_T, rho, D)
            stc = k_T*rho*D^4; % rev/s
            K_T = Propeller.convCoeffToRadPerSec(stc); % rad/s
        end
    end
    
    methods (Access = protected)        
        function DefineComponent(obj)
            % Capacitance Types
            C(1) = Type_Capacitance('x');
            
            % PowerFlow Types
            P(1) = Type_PowerFlow('xt*xh');
            P(2) = Type_PowerFlow('xt^3');
            
            % Vertices
            V(1) = GraphVertex_Internal('Description', "Inertia (omega)",...
                'Capacitance', C(1),...
                'Coefficient', 1*obj.J,...
                'VertexType', 'AngularVelocity');
            
            V(2) = GraphVertex_External('Description', "Input Torque (T_m)",'VertexType','Torque');
            V(3) = GraphVertex_External('Description', "Drag Sink",'VertexType','Abstract');
            
            % Inputs
            
            % Edges
            E(1) = GraphEdge_Internal(...
                'PowerFlow',P(1),...
                'Coefficient',1,...
                'TailVertex',V(2),...
                'HeadVertex',V(1));
            
            E(2) = GraphEdge_Internal(...
                'PowerFlow',P(2),...
                'Coefficient',pop(obj.K_Q),...
                'TailVertex',V(1),...
                'HeadVertex',V(3));
            
            g = Graph(V, E);
            obj.Graph = g;
            
            % Ouputs
            syms omega
            Thrust = obj.calcThrust(omega);
            Thrust_Fun = symfun(Thrust, omega);
            O(1) = GraphOutput('Description', "Thrust", 'Function', Thrust_Fun, 'Breakpoints', {V(1)});
            
            obj.Graph.Outputs = O;
                          
            % Ports
            p(1) = ComponentPort('Description',"Torque Input",'Element',E(1));
            obj.Ports = p;
        end
    end
end

