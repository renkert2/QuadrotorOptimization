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