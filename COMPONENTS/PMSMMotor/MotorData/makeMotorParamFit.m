% Construct paramFit object
pf = paramFit(2,3);

% Set Data
load MotorComponentData.mat MotorComponentData;
[t,~] = table(MotorComponentData);

input_data = [t.kV, t.Rm];
output_data = [t.Mass, t.D, t.Price];
pf.setData(input_data, output_data);
pf.setBoundary();

%% Set Models
% Mass Fit
ft_mass = fittype( '(a/(x+f))^(d) + (b/(y+g))^(e) + c', 'independent', {'x', 'y'}, 'dependent', 'z' );
opts_mass = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts_mass.Display = 'Off';
opts_mass.Lower = [0 0 0 0 0 0 0];
opts_mass.Robust = 'Bisquare';
opts_mass.StartPoint = [0.781043314488573 0.978966203552797 0.845317915312609 0.547008892286345 0.296320805607773 0.7447 0.188955015032545];
opts_mass.Upper = [Inf Inf Inf 3 3 0.1 0.1];

% Diam Fit
ft_diam = fittype( '(a/(x+f))^(d) + (b/(y+g))^(e) + c', 'independent', {'x', 'y'}, 'dependent', 'z' );
opts_diam = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts_diam.Display = 'Off';
opts_diam.Lower = [0 0 0 0 0 0 0];
opts_diam.Robust = 'Bisquare';
opts_diam.StartPoint = [0.817627708322262 0.794831416883453 0.644318130193692 0.378609382660268 0.811580458282477 0.532825588799455 0.350727103576883];
opts_diam.Upper = [Inf Inf Inf 3 3 1000 1];

% Price Fit
ft_price = fittype( '(a/(x+f))^(d) + (b/(y+g))^(e) + c', 'independent', {'x', 'y'}, 'dependent', 'z' );
opts_price = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts_price.Display = 'Off';
opts_diam.Robust = 'Bisquare';
opts_price.Lower = [0 0 0 0 0 0 0];
opts_price.StartPoint = [0.502015967364817 0.497167695799575 0.381499861098169 0.28601827249938 0.699133559301209 0.796257943281577 0.441589056151223];

pf.setModels(ft_mass, opts_mass, ft_diam, opts_diam, ft_price, opts_price);


%%
MotorFit = pf;
save MotorFit.mat MotorFit;