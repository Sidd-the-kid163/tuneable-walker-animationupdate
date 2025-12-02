%% Add FROST 
addpath('frost-dev');
frost_addpath;

%% Set up gait environment
%behavior = loadBehavior2('human_PF_adjustable',true,false,false,false);
behavior = loadBehavior('human_PF_adjustable',true,false,false,false);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Generate a library of gaits
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%[params, logger, sol, status] = runOpt(gait_params,nlp,behavior, logger, library_folder, label_append); %what is logger

% SAVE GIF
%gif_folder = strcat(library_folder, 'gifs/',label_append);
% logger_sim = simRobot(behavior, 10, params);
exportVirtualConstraint(behavior, "/home/sidd163/Desktop/tunable-walker-trajopt-main/vc.json");
conGUI = Plot.LoadAnimatorFROST(behavior);
%Plot.MakeGIF(conGUI);

%close all;
%app2Handle = findall(findall(0,'Type', 'figure'), 'Name', 'UI Figure');
%returns handle of UI Figure
%close(app2Handle);