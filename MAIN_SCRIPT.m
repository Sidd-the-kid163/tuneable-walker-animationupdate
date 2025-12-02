%% Add FROST 
addpath('frost-dev');
frost_addpath;

%%add snakeyaml
javaaddpath('/home/sidd163/snakeyaml.jar');

%% Set up gait environment
compileMex = false;
[nlp, behavior] = startup_opt(compileMex);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Generate a library of gaits
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

logger = [];
all_steplength = 0.2:0.05:0.4;
all_thighlength = 0.2:0.05:0.4;

library_folder = 'All_Gaits/'; %trying for smooth stable gaits

%Iterate over list`
for i = 1:length(all_steplength)
    for j = 1:length(all_thighlength)

        individual_timer = tic;
        gait_params = [0.2,0.25];
    
        label_append = strcat('gait_', num2str(i), '_', num2str(j));
        show_this = strcat("params: ", num2str(gait_params(1)), ", ", num2str(gait_params(2)));
        display(show_this)
        restoredefaultpath;
        addpath(genpath("/home/sidd163/Desktop/tunable-walker-trajopt-main/"));
        addpath('frost-dev');
        frost_addpath;
        javaaddpath('/home/sidd163/snakeyaml.jar');
        [params, logger, sol, status] = runOpt(gait_params,behavior, logger, library_folder, label_append);
        %save('loggertempsave.mat', logger);
        % SAVE GIF
        gif_folder = strcat(library_folder, 'gifs/',label_append);
        % logger_sim = simRobot(behavior, 10, params);
        conGUI = Plot.LoadAnimatorFROSTOG(behavior, logger);
        Plot.MakeGIFOG(conGUI,logger,gif_folder);


        close all;
        app2Handle = findall(findall(0,'Type', 'figure'), 'Name', 'UI Figure');
        close(app2Handle);
      
    end
end

%library_folder = 'All_Gaits/'; %trying for smooth stable gaits
%load("/home/sidd163/Desktop/tunable-walker-trajopt-main/loggertempsave.mat")
%label_append = strcat('gait_', num2str(1), '_', num2str(2));
%gif_folder = strcat(library_folder, 'gifs/',label_append);
%conGUI = Plot.LoadAnimatorFROSTOG(behavior, logger);
%Plot.MakeGIFOG(conGUI,logger,gif_folder);