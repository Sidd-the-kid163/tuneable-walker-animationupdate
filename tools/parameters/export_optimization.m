function [logger, edges, params, export_path] = export_optimization( nlp, sol, behavior, do_save, save_path, gaitName)
%
% Save/export the optimization result.
% _________________________________________________________________________

[ logger, edges ] = nlp_to_logger( nlp, sol );

for i = 1:numel(logger)
    params{i,1} = logger(i).static.params;
    
    if ~isempty(params{i})    
        params{i,1}.name = logger(i).plant.Name;
        params{i,1}.x0 = [logger(i).flow.states.x(:,1); ...
                          logger(i).flow.states.dx(:,1)];

        %%% hard code: remove holonomic constraints static parameters
        fieldNames = fields(params{i,1});
        for j = 1:length(fieldNames)
            if any(contains(fieldNames(j), {'Foot'}))
                params{i,1} = rmfield(params{i,1}, fieldNames(j));
            end
        end
        
    end
end


%% Fit state trajectories for for open-loop domains
% params = export_OL_params(nlp, params, behavior, logger);


%% Generate assymetric parameters for symmetric_optimization
if behavior.isSymmetric
    params = remap_symmetric_param(params, behavior, logger);
end


%% Save info and parameters
if do_save
    baseDir = './';
    
    if nargin < 6
        gaitName = string(datetime('now', 'Format', 'yyyy-MM-dd''T''HH-mm'));
    end
    
    export_path = fullfile(baseDir, save_path, gaitName, '/');
    if ~exist(export_path,'dir')
        mkdir(char(export_path));
    end
    
    k=1;
    for i = 1:numel(params)
        if ~isempty(fields(params{i}))
            export_params{k} = params{i};
            k = k+1;
        end
    end
    
    tmp = strcat(export_path, ['params_',char(gaitName)], '.yaml');
    yaml_write_file(tmp, export_params);
    
    % Exp.exportExpYAML(behavior, '', logger, export_path);
    
    % Save peripheral data
    if ~exist(strcat(export_path, 'info'), 'dir')
        mkdir(char(strcat(export_path, 'info')));
    end
    
    constr_out = strcat(export_path, 'info/constr.txt');
    varibs_out = strcat(export_path, 'info/vars.txt');
    costs_out  = strcat(export_path, 'info/cost.txt');

    %%% Save the folder 'info'
    nlp.checkConstraints(sol, 1e-3, constr_out);
    nlp.checkVariables(sol, 1e-3, varibs_out);
    nlp.checkCosts (sol, costs_out);
else
    export_path = '';
end

end