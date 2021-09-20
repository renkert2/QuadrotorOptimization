%% Init
inertia_matrix = [0.00471826, 0.00000006, 0.00003197;...
    0.00000006, 0.00446811, -0.00000053;...
    0.00003197, -0.00000053, 0.00474617];

d = 0.23; % m, distance from center to rotor

frame = Frame('Name', 'Frame');
frame.Mass.Value = NaN;
frame.Mass.Unit =  "kg";
frame.J_f.Value = inertia_matrix;
frame.J_f.Unit = 'kg*m^2';
frame.d.Value = d;
frame.d.Unit = 'm';
frame.Price.Value = 180.5;
frame.init();

obj_array = ComponentData.empty(2,0);

%% Holy Bro S500 Reference Configuration
% Calculation based on measured values
frame_mass = 0.41455;
autopilot_mass = 0.07;
gps_mass = 0.05945;
optical_flow_mass = 0.01905; % Need to update with mount mass
rc_receiver_mass = 0.0157;

total_frame_mass = frame_mass + autopilot_mass + gps_mass + optical_flow_mass + rc_receiver_mass;
frame.Mass.Value = total_frame_mass; % This value will float a little for different accessories/wires that aren't in the Component list
data = getValues(frame.Params);
obj_array(1) = ComponentData("Frame", "HolyBro", "S500 V2", data);



FrameComponentData = obj_array;
save FrameComponentData.mat FrameComponentData;