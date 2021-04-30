load battSweep
load motorSweep
load propSweep

%%


% hold on
% plot3(X_opt(1), X_opt(2), ft_opt, '.r', 'MarkerSize', 20);
% hold off
%% Thrust Ratio

%% Battery
tr = getGrid(battSweep.I,  [battSweep.PD.ThrustRatio], size(battSweep.ft));
figure
surf(battSweep.X(:,:,1), battSweep.X(:,:,2),tr);
zlim([1 16])
xlabel("$$N_p$$", 'Interpreter', 'latex')
ylabel("$$N_s$$", 'Interpreter', 'latex')
zlabel('Thrust Ratio', 'Interpreter', 'latex')
title('Thrust Ratio vs. Battery Design')

%% Motor
tr = getGrid(motorSweep.I,  [motorSweep.PD.ThrustRatio], size(motorSweep.ft));
figure
surf(motorSweep.X(:,:,1), motorSweep.X(:,:,2),tr);
xlabel("$$kV$$", 'Interpreter', 'latex')
ylabel("$$R_m$$", 'Interpreter', 'latex')
zlabel('Thrust Ratio', 'Interpreter', 'latex')
title('Thrust Ratio vs. Motor Design')

%% Propeller
tr = getGrid(propSweep.I,  [propSweep.PD.ThrustRatio], size(propSweep.ft));
figure
surf(propSweep.X(:,:,1), propSweep.X(:,:,2),tr);
zlim([1 16])
xlabel("$$D$$", 'Interpreter', 'latex')
ylabel("$$P$$", 'Interpreter', 'latex')
zlabel('Thrust Ratio', 'Interpreter', 'latex')
title('Thrust Ratio vs. Propeller Design')
%%

function grid = getGrid(I,data,size)
    grid = NaN(size);
    
    for i = 1:numel(data)
        index = I(:,i);
        grid(index(1), index(2)) = data(i);
    end
end