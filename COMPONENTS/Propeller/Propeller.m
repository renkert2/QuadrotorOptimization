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
        
        rho {mustBeParam} = 1.205 % Air Density - kg/m^3
    end
    
    properties (SetAccess = private)
       Fit paramFit
    end

    
    properties (Dependent)
        square_drag_coeff %coefficient in front of speed^2 term, N*m/(rad/s)^2.
        square_thrust_coeff %coefficient in front of speed^2 term, N/(rad/s)^2.
        
        k_Q % Drag Torque Coefficient - N/(s*kg*m)=1/s^4, speed in rev/s.
    end
    
    methods
        function k_Q = get.k_Q(obj)
            k_Q = obj.k_P / (2*pi);
        end
        
        function sdc = get.square_drag_coeff(obj)
            sdc = obj.k_Q*obj.rho*obj.D^5; % rev/s
            sdc = obj.convCoeffToRadPerSec(sdc); % rad/s
        end
        
        function stc = get.square_thrust_coeff(obj)
            stc = obj.k_T*obj.rho*obj.D^4; % rev/s
            stc = obj.convCoeffToRadPerSec(stc); % rad/s
        end
        
        function k_Q = lumpedToTorqueCoeff(obj, lumped_torque_coeff)
            k_Q = lumped_torque_coeff/(obj.rho*obj.D.^5); % rev/s
        end
        
        function k_T = lumpedToThrustCoeff(obj, lumped_thrust_coeff)
            k_T = lumped_thrust_coeff/(obj.rho*obj.D.^4); % rev/s
        end
        
        function thrust = calcThrust(obj, speed)
            thrust = obj.square_thrust_coeff*(speed.^2);
        end
        
        function speed = calcSpeed(obj, thrust)
            speed = sqrt(thrust/obj.square_thrust_coeff);
        end
        
        function torque = calcTorque(obj, speed)
            torque = obj.square_drag_coeff*(speed.^2);
        end
        
        function init(obj)
            load PropellerFit.mat PropellerFit;
            PropellerFit.Inputs = [obj.D, obj.P];
            PropellerFit.Outputs = [obj.k_P, obj.k_T, obj.Mass];
            PropellerFit.setOutputDependency;
            obj.Fit = PropellerFit;
            
            obj.J.setDependency(@Propeller.calcInertia, [obj.Mass, obj.D]);
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
                'Coefficient',obj.square_drag_coeff,...
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

