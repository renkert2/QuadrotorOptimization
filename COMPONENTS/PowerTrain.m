classdef PowerTrain < System
    %POWERTRAINMODEL Summary of this class goes here
    %   Detailed explanation goes here
    
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
            obj.SimpleModel = PowerTrain_SimpleModel(obj.Model);
            setParamQuantities(obj);
        end
        
        function setParamQuantities(obj)
            % Rotor Speed Function
            % - Calculates required rotor speed as a function of total
            % thrust
            
        end
        
        function rs = RotorSpeed(obj, T_req)
            rs = obj.Propeller.RotorSpeed(T_req/4);
        end
    end
end

