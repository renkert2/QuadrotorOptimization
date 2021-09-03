function AF_Vals = ParameterSensitivityAnalysis(qr,opts)
    arguments
        qr QuadRotor
        opts.RelativeDelta = 0.1 % Perturb Parameter Values by 10%
        opts.AnalysisFunction = @(qr) qr.PerformanceData.SteadyState.BusCurrent
        opts.RelativeChange = true
        opts.RemoveNoChange = true
    end
    % Init
    AF = opts.AnalysisFunction;
    
    % Initial Function Value
    qr.update();
    AF0 = AF(qr); 
       
    % Get Independent Tunable Parameters
     params = qr.Params([qr.Params.Dependent] == false & [qr.Params.Tunable] == true);
     N = numel(params);
     AF_Vals = zeros(N,2);
     for i = 1:N
         AF_Vals(i,:) = evalParam(params(i));
     end
     if opts.RemoveNoChange
         I = ~all(abs(AF_Vals - AF0) < 1e-6,2);
         AF_Vals = AF_Vals(I,:);
         params = params(I);
     end
     
     if opts.RelativeChange
         AF_Vals = AF_Vals/AF0 - 1;
     end
     
          
     parents = [params.Parent];
     parent_names = vertcat(parents.Name);
     syms = params.latex;
     
     param_labels = compose("%s: %s", parent_names, syms);
     
     barh(categorical(param_labels), AF_Vals);
     ax = gca;
     ax.TickLabelInterpreter = 'latex';
     
     title("Sensitivity Analysis")
     if opts.RelativeChange
        xlabel("Analysis Function (Relative Change)");
     else
        xlabel("Analysis Function");
     end
     legend(["$$-\Delta$$", "$$+\Delta$$"],'Interpreter', 'latex')
     
     
     
     function delta = evalParam(p)
        delta = zeros(1,2);
        p0 = p.Value;
        p_delta = opts.RelativeDelta*p0;
        
        p.Value = p0 - p_delta;
        qr.update()
        delta(1) = AF(qr);
        
        p.Value = p0 + p_delta;
        qr.update()
        delta(2) = AF(qr);
        
        p.Value = p0;
        qr.update();
     end
end

