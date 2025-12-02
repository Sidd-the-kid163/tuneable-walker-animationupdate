function behavior = loadBehavior(behaviorName, is_symmetric, ...
                                 delay_coriolis, do_export_behavior, do_export_model)
if ispc
    tmp = strsplit(pwd, '\');
else
    tmp = strsplit(pwd, '/');
end
this_pwd = tmp{end-1};

%%% Remove all other on the export path
p = strread(path,'%s','delimiter',':');
for i = 1:numel(p)
    if ~isempty(strfind(p{i}, [this_pwd,'\matlab\export']))
        keyboard
        rmpath(p{i});
    end
end

%%% Ensure the config is on path
config_dir = 'configuration/';
addpath(config_dir);
behavior_path = strcat(config_dir, 'behaviors/');
addpath(behavior_path);

behavior = feval(strcat(behaviorName, '.behavior'));
behavior.init(is_symmetric, delay_coriolis);

%% Export dynamic model
t1 = tic;

%%% Dynamics
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

warning('off');

if do_export_model
    for i = 1:numel(robot.Mmat)
        robot.Mmat{i}.export(export_path); % Inertia matrix
    end
    %%% export drift vectors: C\dot{q}, G
    for i = 1:numel(robot.Fvec)
        robot.Fvec{i}.export(export_path);
    end
end

%%% Behavior
export_path = strcat('export/', behaviorName, '/sim');
if ~exist(export_path,'dir')
    mkdir(export_path);
end
addpath(export_path);

%%% this can handle odd number of edges
if do_export_behavior
    v = fields(behavior.vertices);
    for i = 1:numel(v)
        customCompileVertex(behavior.vertices.(v{i}), export_path);
    end
    
    if ~isempty(behavior.edges)
        e = fields(behavior.edges);
        for i =  1:numel(e)
            customCompileEdge(behavior.edges.(e{i}), export_path);
        end
    end
end

warning('on');
fprintf('Compilation took %f minutes.\n', toc(t1)/60);
disp('************************************************')

end