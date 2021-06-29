classdef PowerTrainState < ModelState
    %Wraps the output from PowerTrain Simple Model into something that's easier to work with
    %   Detailed explanation goes here    
    properties
        q double % Battery SOC
        BatteryOCV double % Must Be set externally since it's dependent on sym param vals
    end
    
    properties (Dependent)
        BatteryPower
        BusPower
        InverterPower
        MotorPower
        
        BatteryEfficiency
        InverterEfficiency
        MotorEfficiency
        SystemEfficiency
    end
    
    properties (Dependent)
        BatterySOC
        MotorCurrent
        RotorSpeed
        BusVoltage
        BusCurrent
        InverterCurrent_DC
        InverterVoltage_q
        MotorTorque
        TotalThrust
    end
    
    %% Power and Efficiency Get Methods
    methods
        function x = get.BatteryPower(obj)
            x = obj.BatteryOCV.*obj.BusCurrent;
        end
        
        function x = get.BusPower(obj)
            x = obj.BusVoltage.*obj.BusCurrent;
        end
        
        function x = get.InverterPower(obj)
            x = obj.InverterVoltage_q.*obj.MotorCurrent;
        end
        
        function x = get.MotorPower(obj)
            x = obj.MotorTorque.*obj.RotorSpeed;
        end
        
        function x = get.BatteryEfficiency(obj)
            x = obj.BusPower./obj.BatteryPower;
        end
        
        function x = get.InverterEfficiency(obj)
            x = (obj.InverterPower*4)./obj.BusPower;
        end
        
        function x = get.MotorEfficiency(obj)
            x = obj.MotorPower./obj.InverterPower;
        end
        
        function x = get.SystemEfficiency(obj)
            x = (obj.MotorPower*4)./obj.BatteryPower;
        end
    end
    %% Individual State Get Methods
    methods
        function x = get.BatterySOC(obj)
            x = obj.y(1,:);
        end
        
        function x = get.MotorCurrent(obj)
            x = obj.y(2,:);
        end
        
        function x = get.RotorSpeed(obj)
            x = obj.y(3,:);
        end
        
        function x = get.BusVoltage(obj)
            x = obj.y(4,:);
        end
        
        function x = get.BusCurrent(obj)
            x = obj.y(5,:);
        end
        
        function x = get.InverterCurrent_DC(obj)
            x = obj.y(6,:);
        end
        
        function x = get.InverterVoltage_q(obj)
            x = obj.y(7,:);
        end
        
        function x = get.MotorTorque(obj)
            x = obj.y(8,:);
        end
        
        function x = get.TotalThrust(obj)
            x = obj.y(9,:);
        end
    end
        
        
end

