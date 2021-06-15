classdef 3DBodyModel < Model
    properties (SetAccess = private)
        StateSyms string
        InputSyms string
        XI struct % State Index
    end
    
    methods
        function obj = 3DBodyModel(params)
            obj.Nx = 12;
            obj.Nu = 4;
            obj.Nd = 0;
            obj.Ny = 12; % Full State Feedback
            
            obj.XI.p = 1:3;
            obj.XI.v = 4:6;
            obj.XI.Theta = 7:9;
            obj.XI.omega = 10:12;
            
            obj.StateSyms = ["x","y","z","v_x","v_y","v_z","phi","theta","psi","omega_x","omega_y","omega_z"];
            obj.StateDescriptions = ["X Position", "Y Position", "Z Position", "X Velocity", "Y Velocity", "Z Velocity", "Roll", "Pitch", "Yaw", "X Angular Velocity", "Y Angular Velocity", "Z Angular Velocity"];
            obj.InputSyms = ["f", "tau_x", "tau_y", "tau_z"];
            obj.InputDescriptions = ["Total Thrust", "X Torque", "Y Torque", "Z Torque"];
            obj.DisturbanceDescriptions = [];
            obj.OutputDescriptions = obj.StateDescriptions;
            
            if nargin
                obj.Params = params; % Must be set before calling genMatlabFunctions
            end
        end
        
        function init(obj)
            setF(obj)
            setG(obj)
            init@Model(obj);
        end
        
        function update(obj)
        end
    end
end

