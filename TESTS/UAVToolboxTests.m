model = multirotor;

s = state(model);
s(1:3) = [3;2;1];

u = control(model);
u.Roll = -pi/12;
u.Thrust = 2;

e = environment(model);

sdot = derivative(model,s,u,e);

%%
simOut = ode45(@(~,x)derivative(model,x,u,e), [0 3], s);
plot(simOut.y(9,:))
figure
plot(simOut.y(2,:));
hold on
plot(simOut.y(3,:));
legend('Y-position','Z-position')
hold off

%%
translations = simOut.y(1:3,1:300:end)'; % xyz position
rotations = eul2quat(simOut.y(7:9,1:300:end)'); % ZYX Euler
plotTransforms(translations,rotations,...
    'MeshFilePath','multirotor.stl','InertialZDirection',"down")
view([90.00 -0.60])