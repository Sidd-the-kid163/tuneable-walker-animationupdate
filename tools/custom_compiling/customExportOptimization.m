function [ ] = customExportOptimization( behavior, nlp, ...
                                         do_export_optModel,...
                                         do_export_optBehavior,...
                                         do_export_optCost )
%
% Compile expressions for optimization with customizible options
% - do_export_optModel    : whether to export EOM
% - do_export_optBehavior : whether to export constraints(includes constraint forces and spring forces)
% - do_export_optCost     : whether to export objective functions
% _________________________________________________________________________

robot = behavior.robotModel;
if strcmp(robot.States.x.label{3}, 'BaseRotY')
    baseType = '2D';
else
    baseType = '3D';
end

export_path = strcat('export/', 'dynamics/', robot.Name, '/', baseType);

if ~exist(export_path,'dir')
    mkdir(export_path);
end
addpath(export_path);

%% EOM
if do_export_optModel
    %%% dynamics
    compileDyn(nlp, export_path); 
    
    %%% compile dynamic_constraints for opt
	compileConstraint(nlp, [], {'dynamics_equation'}, export_path, []);
end

if ~do_export_optModel && do_export_optBehavior
    %%% compile dynamic_constraints for opt without D, C\dot{q}, G
    compileDynConstraints_woDCeG(nlp, export_path);
end

%% Behavior
export_path = strcat('export/', behavior.name, '/opt');
if ~exist(export_path,'dir')
    mkdir(export_path);
end
addpath(export_path);

if do_export_optBehavior
    compileConstraint(nlp,[],[], export_path,{'dynamics_equation'});
end

%%% to compile a particular constraint
% compileConstraint(nlp,[],{'nsfVel_baseVel_x'},export_path);

%% Cost Function
if do_export_optCost
    compileObjective(nlp, [],[], export_path);
end

end