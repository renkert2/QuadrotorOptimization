global individualSweeps
load individualSweeps.mat

%%
[~,x,y] = correlate("D", "Propeller", "P");
x_range = [ov(1).lb ov(1).ub];
y_range = [ov(2).lb ov(2).ub];

plotFilledNaN(x,y,x_range,y_range)

%%

vars = ["D", "P", "N_p", "N_s", "kV", "Rm"];
comps = ["Propeller", "Propeller", "Battery", "Battery", "Motor", "Motor"];
lb = unscale(ov, ov.LB)';
ub = unscale(ov, ov.UB)';

t = tiledlayout(numel(vars), numel(vars));
title(t,"Design Variable Coupling");
t.TileSpacing = 'tight';
t.Padding = 'compact';
xlabel(t,"Independent Variable");
ylabel(t, "Dependent Variable");
lblspecs = {'Interpreter', 'latex', 'FontSize', 14, 'FontWeight', 'bold'};

tile_counter = 1;
N = numel(vars);
CMat = zeros(N,N);
for i = 1:N
    for j = 1:N
        nexttile(tile_counter)
        [CMat(j,i),x,y] = correlate(vars(j), comps(i), vars(i));
        tile_counter = tile_counter+1;
        
        x_range = [lb(j) ub(j)];
        y_range = [lb(i) ub(i)];
        plotFilledNaN(x,y,x_range,y_range);
        
        if j == 1
            ylabel("$$"+vars(i)+"$$",lblspecs{:});
        else
            set(gca,'Yticklabel',[]) 
        end
        if i == numel(vars)
            xlabel("$$"+vars(j)+"$$", lblspecs{:});
        else
            set(gca,'Xticklabel',[]) 
        end
    end
end

function [c,x,y] = correlate(var1, comp2, var2)
    global individualSweeps
    x = individualSweeps.(var1).X;
    dd_array = [individualSweeps.(var1).DD.(comp2)];
    if var2 == "kV"
        data = [dd_array.("K_t")];
        data = PMSMMotor.KtTokV(data);
    else
        data = [dd_array.(var2)];
    end
    
    y = processSweep(individualSweeps.(var1).I, data, length(x));
    
    m = [x',y'];
    m = m(all(~isnan(m),2),:);
    c = corrcoef(m);
    c = c(1,2);
end

function plotSweep(var, data)
    global individualSweeps
    v = individualSweeps.(var);
    x = v.X;
    y = processSweep(individualSweeps.(var).I, data, length(x));
    
    figure
    plot(x,y);
    xlabel("$$"+var+"$$", 'Interpreter', 'latex');
end

function plotFilledNaN(x,y, x_range, y_range)
    plot(x,y);
    xlim(x_range);
    ylim(y_range);
    
    nan_i = find([isnan(y)]);
    
    if nan_i
        split_i = find(diff(nan_i) > 1) + 1;
        nan_groups = [1 split_i numel(y)];
        hold on
        for i = 1:(numel(split_i)+1)
            group_i = nan_i(nan_i >= nan_groups(i) & nan_i <= nan_groups(i+1));
            start_x = x(max(1,group_i(1)-1));
            finish_x = x(min(group_i(end)+1, numel(x)));
            
            patch([start_x start_x finish_x finish_x], [y_range fliplr(y_range)], 'r','FaceAlpha',0.2,'EdgeColor','none');
        end
        hold off
    end
end