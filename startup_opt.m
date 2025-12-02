function [nlp, behavior] = startup_opt(compileMex)
% This script initialize AMBER-PF simulator and optimization.
% _________________________________________________________________________

%%% Control the symbolic expression export
if compileMex
    do_export_model = true;
    do_export_behavior = true;
else
    do_export_model = false;
    do_export_behavior = false;
end

%%% whether this behavior is "symmetrically" periodic
is_symmetric = true;

%%% name of the behavior
% behaviorName = 'walk_PF';
% behaviorName = 'human_PF';
behaviorName = 'human_PF_adjustable';


%% Load and compile the behavior
addpath(genpath('tools'));
behavior = loadBehavior(behaviorName, is_symmetric, ....
    false, do_export_behavior, do_export_model);

%% Load the behavior specific NLP

% default input params
inputparams.average_vel = 0.5; 
inputparams.lift_off_vel = 0.1; 
inputparams.min_foot_clearance = 0.04;
inputparams.impact_vel = -0.4;
inputparams.step_length = 0.4;
inputparams.max_height_tau = 0.6;
inputparams.thighlength = 0.4;
inputparams.shanklength = 0.4;

nlp = feval(strcat(behavior.name, '.Constraints.setupOpt'), behavior, inputparams);

%%% Compile and export optimization functions
if compileMex
    do_export_optModel    = true;
    do_export_optBehavior = true;
    do_export_optCost     = true;
else
    do_export_optModel    = false;
    do_export_optBehavior = false;
    do_export_optCost     = false;
end

t1 = tic;
warning('off');
customExportOptimization(behavior, nlp, do_export_optModel, ...
    do_export_optBehavior, do_export_optCost);
warning('on');
fprintf('Matlab compilation took %f minutes.\n', toc(t1)/60);


end