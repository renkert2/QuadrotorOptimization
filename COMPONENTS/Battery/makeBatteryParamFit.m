load BatteryComponentData.mat

%% 
t =  table(BatteryComponentData);

%%
x = t.Q.*t.N_s.*t.N_p;
y = t.Mass;
plot(x,y,'.b')
%%
pf = paramFit(2, 2);

inputs = [t.N_s, t.Q];
outputs = [t.R_s,t.Mass];
pf.setData(inputs,outputs)
pf.setBoundary();

%%
f = @(a, b, x, y) a*x.*y + b;
ftM = fittype( f , 'independent', {'x','y'}, 'dependent', {'z'});
foM = fitoptions('Method', 'NonlinearLeastSquares');

ftRs =  fittype( 'a/(y+b)+c*x^(d/y) +k', 'independent', {'x', 'y'}, 'dependent', 'z' );
foRs = fitoptions( 'Method', 'NonlinearLeastSquares' );
foRs.Algorithm = 'Levenberg-Marquardt';
foRs.Display = 'Off';
foRs.Robust = 'Bisquare';
foRs.StartPoint = [0.932734068002066 0.597116865382136 0.78528762506439 0.546881519204984 0.278498218867048];

pf.setModels(ftRs, foRs, ftM, foM);