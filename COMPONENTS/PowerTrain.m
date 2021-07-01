classdef PowerTrain < System
    %POWERTRAINMODEL Summary of this class goes here
    %   Detailed explanation goes here
    properties (Constant)
        SpeedOutputs_I double = [3;5;7;9];
    end
    
    properties
        SimpleModel PowerTrain_SimpleModel
    end
    
    properties (SetAccess = private)
        Battery Battery
        DCBus DCBus_CurrentEquivalence
        Inverter PMSMInverter
        Motor PMSMMotor
        Propeller Propeller
        MotorProp MotorProp
    end
    
    methods
        function obj = PowerTrain(p)
            arguments
                p.Battery Battery = Battery('Name', "Battery");
                p.DCBus DCBus_CurrentEquivalence = DCBus_CurrentEquivalence('Name', 'Bus', 'N_inputs',1,'N_outputs',4);
                p.PMSMInverter PMSMInverter = PMSMInverter('Name', "Inverter");
                p.PMSMMotor PMSMMotor = PMSMMotor('Name', "Motor");
                p.Propeller Propeller = Propeller('Name', "Propeller");
            end
            
            pmsminverters = Replicate([p.PMSMInverter], 4);
            pmsminverters = pmsminverters{1};
            
            motorprop = MotorProp('PMSMMotor', p.PMSMMotor, 'Propeller', p.Propeller);
            motorprops = Replicate(motorprop, 4, 'RedefineParams', false, 'RedefineElement', true, 'RedefineChildren', false);
            motorprops = vertcat(motorprops{:});
            
            
            % I would love for this to work at some point.
            % motorprop = MotorProp('PMSMMotor', p.PMSMMotor, 'Propeller', p.Propeller);
            % pmsminverters = repmat(p.PMSMInverter, 1, 4);
            % motorprops = repmat(motorprop, 1, 4);
            
            Components = [p.Battery; p.DCBus; pmsminverters; motorprops];
            
            ConnectP = {[p.Battery.Ports(1) p.DCBus.Ports(1)]};
            
            for i = 1:4
                ConnectP{end+1,1} = [pmsminverters(i).Ports(1),p.DCBus.Ports(1+i)];
                ConnectP{end+1,1} = [pmsminverters(i).Ports(2), motorprops(i).Ports(1)];
            end
            
            obj = obj@System("PowerTrain", Components, ConnectP);
            obj.Battery = p.Battery;
            obj.DCBus = p.DCBus;
            obj.Inverter = p.PMSMInverter;
            obj.Motor = p.PMSMMotor;
            obj.Propeller = p.Propeller;
            obj.MotorProp = motorprop;
            
            init_post(obj);
        end
        
        function init_post(obj)
            createModel(obj);
            obj.Model.Name = "QuadrotorPowerTrainModel";
            obj.Model.x0 = [1 0 0 0 0 0 0 0 0].';
            
            obj.SimpleModel = PowerTrain_SimpleModel(obj.Model); % Name set inside this subclass
        end
        
        function ptss = calcSteadyState(obj, rotor_speed, q_bar)
            % Calculates steady-state values of the powertrain model at thrust T_reqd, returns QRSteadyState object
            % Default value for q_bar is the average battery soc
            % Default value fot T_reqd is the HoverThrust
            % Overrides built-in Model method
            
            arguments
                obj 
                rotor_speed double
                q_bar double = []
            end
            
            if isempty(q_bar)
                q_bar = obj.Battery.Averaged_SOC;
            end
            
            x0 = [0.5; 1];
            
            [x_bar, u_bar, y_bar] = calcSteadyState(obj.SimpleModel, rotor_speed, q_bar, x0);
            
            ptss = PowerTrainState();
            ptss.q = q_bar;
            ptss.x = x_bar;
            ptss.u = u_bar;
            ptss.y = y_bar;
            ptss.BatteryOCV = obj.Battery.V_OCV_pack(q_bar);
        end
        
        function ptss = calcSteadyStateIO(obj, u, rotor_speed_init, q_bar)
            arguments
                obj
                u double
                rotor_speed_init double
                q_bar double = []
            end
            
            if isempty(q_bar)
                q_bar = obj.Battery.Averaged_SOC;
            end

            x0 = [1; rotor_speed_init];
            [x_bar, u_bar, y_bar] = calcSteadyStateIO(obj.SimpleModel, u, q_bar, x0);
                        
            ptss = PowerTrainState();
            ptss.q = q_bar;
            ptss.x = x_bar;
            ptss.u = u_bar;
            ptss.y = y_bar;
            ptss.BatteryOCV = obj.Battery.V_OCV_pack(q_bar);
        end
        
        function [Apt, Bpt, Cpt, Dpt] = CalcMatrices(obj, ptstate)
            x = vertcat(ptstate.u, repmat([ptstate.MotorCurrent; ptstate.RotorSpeed],4,1));
            u = repmat(ptstate.u, obj.Model.Nu, 1);
            d = zeros(obj.Model.Nd,1);
            [Apt, Bpt, ~, Cpt, Dpt, ~] = CalcMatrices(obj.Model.LinearModel, x, u, d);
            
            % We only want Speed outputs
            Cpt = Cpt(obj.SpeedOutputs_I,:);
            Dpt = Dpt(obj.SpeedOutputs_I,:); 
        end
        
        function PT_simple_ss = getFirstOrderSS(obj, ptstate)
            [Apt,Bpt,Cpt,Dpt] = CalcMatrices(obj, ptstate);
            
            % Omit Battery Dynamics
            Aptmod = Apt(2:end,2:end);
            Bptmod = Bpt(2:end,:);
            Cptmod = Cpt(:,2:end);
            Dptmod = Dpt(:,:);
            
            gain = Dptmod-Cptmod*(Aptmod\Bptmod);
            dom_pole = min(abs(eigs(Apt)));
            
            Asimple = -dom_pole.*eye(4);
            Bsimple = -Asimple*gain;
            Csimple = eye(4);
            Dsimple = zeros(4);
            PT_simple_ss = ss(Asimple, Bsimple, Csimple, Dsimple);
        end
        
        function rs = RotorSpeed(obj, T_req)
            rs = obj.Propeller.RotorSpeed(T_req/4);
        end
    end
end

