function params = NewStepPreProcess(sys, t, x, controller, params) %#ok<INUSL>
    
    y = struct2array(sys.VirtualConstraints);
    nx = length(x)/2;
    for i=1:numel(y)
        if strcmp(y(i).PhaseType,'TimeBased')
            params.(y(i).PhaseParamName) = t + params.(y(i).PhaseParamName);
        else
            % tau = calcPhaseVariable(y(end), t, x(1:nx), x(nx+1:end), params.(y(i).PhaseParamName));
            % delta = tau{1}*(params.(y(i).PhaseParamName)(1) - params.(y(i).PhaseParamName)(2)) ...
            %         +  params.(y(i).PhaseParamName)(2);
            % params.(y(i).PhaseParamName)(2) =  delta;
        end
    end

    if strcmp(y(i).PhaseType,'TimeBased')
        controller.Param.time_start = t;
    end
end