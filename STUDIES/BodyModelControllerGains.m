bms = BodyModelSystem(QR_S500);
bms.init();
%%
rho_range = [1e-6, 1, 10];

out_struct = struct('rho', [], 't', [], 'y', []);
for i = 1:numel(rho_range)
    rho = rho_range(i);
    out_struct(i).rho = rho;
    bms.setLQR(rho);
    [out_struct(i).t, out_struct(i).y] = bms.Simulate();
end

%%
for i = 1:numel(rho_range)
    bms.BM.plot(out_struct(i).t, out_struct(i).y, 10)
    t = sprintf("Trajectory: $$\\rho = %f$$", rho_range(i));
    title(t, 'Interpreter', 'latex')
end

%% Speed and Controller with Reference Trajectory
rho_range = [1e-4, .1, 1];
speed_range = [1, 5, 10];
out_struct = struct('rho', [], 'speed', [], 't', [], 'y', []);
for i = 1:numel(speed_range)
    speed = speed_range(i);
    bms.setRefTraj('Speed', speed);
    for j = 1:numel(rho_range)
        out_struct(i,j).speed = speed;
        rho = rho_range(j);
        out_struct(i,j).rho = rho;
        
        bms.setLQR(rho);
        [out_struct(i,j).t, out_struct(i,j).y] = bms.Simulate();
    end
end

%%
tiledlayout(3,3, 'TileSpacing', 'compact', 'Padding', 'compact')

for i = 1:numel(speed_range)
    for j = 1:numel(rho_range)
        ax = nexttile;
        s = out_struct(i,j);
        bms.plot(s.y, 'ParentAxes', ax, 'Annotate', false)
        title(sprintf('Speed: %.4f, Rho: %.4f', s.speed, s.rho))
    end
end