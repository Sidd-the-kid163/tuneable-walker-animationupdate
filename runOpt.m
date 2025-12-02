function [params, logger, sol, info] = runOpt(variable_params, behavior,logger, save_path, gaitName)

%export path
export_path = 'gen/opt';

% variable params: [step length, thigh length]
steplength = variable_params(1);
thighlength = variable_params(2);
% keep total length constant (each leg is nominally 0.5m)
shanklength = 0.8-thighlength;

inputparams.thighlength = thighlength;
inputparams.shanklength = shanklength;

% OTHER PARAMETERS
inputparams.average_vel = 0.6; %not being used
inputparams.min_foot_clearance = 0.05;
inputparams.impact_vel = -0.2;
inputparams.step_length = steplength;
inputparams.max_height_tau = 0.5;

%% run optimization
max_iter = 400;
do_save = true;

% load nlp with input params
nlp = feval(strcat(behavior.name, '.Constraints.setupOpt'), behavior,...
   inputparams);

%generate costs
Opt.Compile(nlp,[],[],export_path);

% Use initial guess if provided
if ~isempty(logger)
    if exist('logger','var') == 1
        loadNodeInitialGuess(nlp.Phase(1), logger(1));
        if length(logger) > 1 && length(logger) < 3
            loadNodeInitialGuess(nlp.Phase(3), logger(2));
            loadEdgeInitialGuess(nlp.Phase(2), logger(1), logger(2));
            loadEdgeInitialGuess(nlp.Phase(4), logger(2), logger(1));
        elseif length(logger) > 2
            loadNodeInitialGuess(nlp.Phase(3), logger(2));
            loadNodeInitialGuess(nlp.Phase(5), logger(3));
            loadNodeInitialGuess(nlp.Phase(7), logger(4));
            loadEdgeInitialGuess(nlp.Phase(2), logger(1), logger(2));
            loadEdgeInitialGuess(nlp.Phase(4), logger(2), logger(3));
            loadEdgeInitialGuess(nlp.Phase(6), logger(3), logger(4));
            loadEdgeInitialGuess(nlp.Phase(8), logger(4), logger(1));
        else
            loadEdgeInitialGuess(nlp.Phase(2), logger(1), logger(1));
        end
    end
end

%%% Link the NLP problem to a NLP solver
options = struct();
options.max_iter = max_iter;
options.tol = 1e-2;
solver = IpoptApplication(nlp,options);

% Log all activities in cmd
diary log.txt;

%%% Run optimization -- Use ipopt
t2 = tic;
[sol, info] = optimize(solver); 
fprintf('Optimization took %f sec.\n', toc(t2));

if info.status < 0
    fprintf('Optimization failed.\n');
end
diary off;

[logger, edges, params, ~] = export_optimization(nlp, sol, behavior, do_save, save_path, gaitName );

if do_save
    data = struct();
    data.logger = logger;
    data.edges = edges;
    data.params = params;
    data.sol = sol;
    data.nlp = nlp;
    
    %%% log optimization results
    save(strcat(save_path, gaitName, '/optData.mat'), 'data');
    movefile('log.txt', strcat(save_path,gaitName));
end

end
%{
% Iterate over list`
for i = 1:length(all_steplength)
    for j = 1:length(all_thighlength)
    end
end
%}