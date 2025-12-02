function [] = plotData(behavior, logger, nlp, sol, plotType, params) 
% Plot data for simulation & optimization & experiments.
%
% Color syntax: 
%       Red color is for         "optimization data"
%       Blue color is for        "simulation data"
%       magenta-dash line is for "desired data"
%       Black-dash line is for   "boundary data"
%
%       green-solid:  "left foot"
%       green-dash:   "right foot"
% 
% Arguments:
%   - behavior, logger, nlp, sol, params: obvious
%   - plotType: 'sim' 'opt' 'optsim' (plot sim or opt, or together) 
%                Note optsim does not work for more than one step. 
% _________________________________________________________________________


if nargin < 3
    nlp = [];
end
if nargin < 4
    sol = [];
end
if nargin < 5
    plotType = 'sim'; 
end
if nargin < 6
    params = [];
end

export_path = './export/geometry';
if ~exist(export_path,'dir')
    mkdir(export_path);
    genPlotCode(behavior, export_path);
end
addpath(export_path);

JointNames_all = {behavior.robotModel.Joints(:).Name};
motorNames = {behavior.robotModel.Joints(4:end).Name}';


%% To plot sim and opt togerther, load the optimized gait first
if strcmp(plotType, 'optsim')
    warning('Compare simulated data with the latest opt gait.')
    listing = dir(['params/', behavior.name]); 
    gaitName = listing(end).name; 
    load(['params/', behavior.name, '/', gaitName, '/optData.mat']);
    logOpt = data.logger;
    params = loadGaitYAML(behavior, gaitName);
end


%% Collect plotting comparisons
model_bounds = behavior.robotModel.getLimits();
nDomain = behavior.hybridSystem.Gamma.numnodes;
if strcmp(plotType, 'opt')
    nStep = 1;
else
    nStep = ceil(length(logger)/nDomain);
end
nVertex = nDomain * nStep;

%%% Color Map
N = 20; 
X = linspace(0,pi*3,1000); 
Y = bsxfun(@(x,n)sin(x+2*n*pi/N), X.', 1:N); 
C = linspecer(N);

if strcmp(plotType, 'opt')
    mColor = 'r';
else
    mColor = 'b';
end

com_data = cell(nVertex, 1);



%% Main plotting function
for k = 1:nVertex
    
    iDomain = mod(k + (nDomain-1), nDomain) + 1;    
    domain = behavior.hybridSystem.Gamma.Nodes.Domain{iDomain};
    
    %% Data Processing
    t   = logger(k).flow.t;
    x   = logger(k).flow.states.x;
    dx  = logger(k).flow.states.dx;
    ddx = logger(k).flow.states.ddx;
    u   = logger(k).flow.inputs.Control.u;
    if isfield(logger(k).flow.inputs, 'ConstraintWrench')
        wrenches = logger(k).flow.inputs.ConstraintWrench;
    else
        wrenches = [];
    end
    
    if isfield(logger(k).flow.inputs, 'External')
        external = logger(k).flow.inputs.External;
    else
        external = [];
    end
    
    % Compute $COM$ data
    com = zeros(6, length(t));
    for i = 1:length(t)
        %com_pos = arrayfun(@(var)com_position(var(:,1)), x);
        com_pos = com_posFun(x(:,i));
        com_vel = com_velFun(x(:,i), dx(:,i));
        com(:,i) = [com_pos'; com_vel];
    end
    com_data{k} = com;
    
    % Compute $feet$ data
    clear RF_pos LF_pos RF_vel LF_vel
    for i = 1:length(t)
        RF_pos(:,i) = RightMidFoot_posFun(x(:,i) );
        LF_pos(:,i) = LeftMidFoot_posFun( x(:,i) );
        RF_vel(:,i) = RightMidFoot_velFun(x(:,i), dx(:,i) );
        LF_vel(:,i) = LeftMidFoot_velFun( x(:,i), dx(:,i) );
    end

    %%%%%%% Only plotting for sim %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if strcmp(plotType, 'sim')
        tau = logger(k).flow.tau_position;
        ya  = logger(k).flow.ya_position;
        yd  = logger(k).flow.yd_position; 
        
        if isfield(logger(k).flow, 'ya_velocity')
            y1a = logger(k).flow.ya_velocity;
            y1d = logger(k).flow.yd_velocity;
        end
    end
    
    %%%%%%% Only plotting for opt %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if strcmp(plotType, 'opt')
        % Recover $tau$
        vc = domain.VirtualConstraints.position;
        tau = zeros(1,length(t));
        for i = 1:length(t)
            temp = vc.calcPhaseVariable(t(i), x(:,i), dx(:,i), params{iDomain}.pposition);
            tau(i) = temp{1};
        end
        
        tau_dot = zeros(1,length(t));
        % for i = 1:length(t)
        %     tau_dot(i) = d1tau_position_slip0(x(:,i), dx(:,i), params{iDomain}.pposition);
        % end
        
        hip_vel = zeros(1,length(t));
        for i = 1:length(t)
            tmp = hipVelocity(x(:,i), dx(:,i));
            hip_vel(i) = tmp(1);
        end
        
        % Recover $ya$ $yd$
        ya = []; yd = [];
        for i = 1:length(t)
            yaTemp = vc.calcActual(x(:,i), dx(:,i));
            ydTemp = vc.calcDesired(t(i), x(:,i), dx(:,i), params{iDomain}.aposition, params{iDomain}.pposition);
            ya(:,i) = yaTemp{1};
            yd(:,i) = yaTemp{1};
            
            if isfield(domain.VirtualConstraints, 'velocity')
                y1aTemp = domain.VirtualConstraints.velocity.calcActual(x(:,i), dx(:,i));
                y1dTemp = domain.VirtualConstraints.velocity.calcDesired(t(i), x(:,i), dx(:,i), params{iDomain}.avelocity);
                y1a(:,i) = y1aTemp{1};
                y1d(:,i) = y1dTemp{1};
            end
        end
    end
    
    %%%%%%% Plotting for opt vs. sim %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if strcmp(plotType, 'optsim')
        tOpt   = logOpt(k).flow.t;
        xOpt   = logOpt(k).flow.states.x;
        dxOpt  = logOpt(k).flow.states.dx;
        ddxOpt = logOpt(k).flow.states.ddx;
        uOpt   = logOpt(k).flow.inputs.Control.u;
        if isfield(logOpt(k).flow.inputs, 'ConstraintWrench')
            wrenchesOpt = logOpt(k).flow.inputs.ConstraintWrench;
        else
            wrenchesOpt = [];
        end
        
        % Compute COM vel data
        com_vel_opt =  zeros(3, length(tOpt));
        for i = 1:length(tOpt)
            com_vel_opt(:,i) = com_velFun(xOpt(:,i), dxOpt(:,i));
        end

        % Recover $tau$
        vc = domain.VirtualConstraints.position;
        tauOpt = zeros(1,length(tOpt));
        for i = 1:length(tOpt)
            temp = vc.calcPhaseVariable(tOpt(i), xOpt(:,i),dxOpt(:,i), params{iDomain}.pposition);
            tauOpt(i) = temp{1};
        end
        
        % Recover $ya$ $yd$
        yaOpt = [];
        ydOpt = [];
        for i = 1:length(tOpt)
            yaTemp = vc.calcActual(xOpt(:,i), dxOpt(:,i));
            ydTemp = vc.calcDesired(tOpt(i), xOpt(:,i), dxOpt(:,i), params{iDomain}.aposition, params{iDomain}.pposition);
            yaOpt(:,i) = yaTemp{1};
            ydOpt(:,i) = yaTemp{1};
        end
        
        % Recover feet data from opt
        clear RF_pos_opt LF_pos_opt RF_vel_opt LF_vel_opt
        for i = 1:length(tOpt)
            RF_pos_opt(:,i) = RightMidFoot_posFun(xOpt(:,i) );
            LF_pos_opt(:,i) = LeftMidFoot_posFun( xOpt(:,i) );
            RF_vel_opt(:,i) = RightMidFoot_velFun(xOpt(:,i), dxOpt(:,i) );
            LF_vel_opt(:,i) = LeftMidFoot_velFun( xOpt(:,i), dxOpt(:,i) );
        end
        
        tau = logger(k).flow.tau_position;
        ya  = logger(k).flow.ya_position;
        yd  = logger(k).flow.yd_position; 
        
    end
    
    
    %% Tau
    h = figure(100); 
    if(k==1) clf; end
    h.Name = 'tau'; h.WindowStyle = 'docked'; 
    subplot(2,2,1)
        plot(t, tau); hold on; grid on; title 'Tau';
        xlabel 'Time (s)'; ylabel 'tau'; ylim([-0.1, 1.1]);
        if strcmp(plotType, 'optsim')
            plot(tOpt, tauOpt, '*');
        end
    if strcmp(plotType, 'opt')
        subplot(2,2,3)
            plot(t, tau_dot); hold on; grid on; title 'Tau dot';
        subplot(2,2,[2,4])
            plot(t, hip_vel); hold on; grid on; 
            title(['hip velX, ',num2str(mean(hip_vel),3)]);
    end
    
    
    %% States
    jointName = {behavior.robotModel.Joints.Name};
    iBase     = find(strncmpi(jointName,'Base', length('Base')));
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% base position %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %----------------------------------------------------------------------
    % Floating base
    h = figure(1011); 
    if (k==1) clf; end
    h.Name = 'base'; h.WindowStyle = 'docked'; hold on; grid on;
    
    for i = 1:length(iBase)
        j = iBase(i);
        subplot(3,3,i); hold on; grid on;
        
        plot(t, x(j,:), 'Color', mColor);
        if strcmp(plotType, 'optsim')
            plot(tOpt, xOpt(j,:), '*');
        end
        
        % Bounds
        lb = model_bounds.states.x.lb(j);
        ub = model_bounds.states.x.ub(j);
        plot([t(1), t(end)], [lb, lb], 'k--');
        plot([t(1), t(end)], [ub, ub], 'k--');
        
        % Label
        title(behavior.robotModel.States.x.label{j});
        xlabel('Time(s)'); ylabel('q (rad)');
    end
    
    %%% base velocity     
    for i = 1:length(iBase)
        j = iBase(i);
        subplot(3,3,i+3); hold on; grid on;
        
        plot(t, dx(j,:), 'Color', mColor);
        if strcmp(plotType, 'optsim')
            plot(tOpt, dxOpt(j,:), '*');
        end
        
        % Bounds
        lb = model_bounds.states.dx.lb(j);
        ub = model_bounds.states.dx.ub(j);
        plot([t(1), t(end)], [lb, lb], 'k--');
        plot([t(1), t(end)], [ub, ub], 'k--');
        
        % Label
        title(behavior.robotModel.States.dx.label{j});
        xlabel('Time(s)'); ylabel('dq (rad/s)');
    end
    
    %%% base accleration 
    for i = 1:length(iBase)
        j = iBase(i);
        subplot(3,3,i+6); hold on; grid on;
        
        plot(t, ddx(j,:), 'Color', mColor);
        if strcmp(plotType, 'optsim')
            plot(tOpt, ddxOpt(j,:), '*');
        end
        
        % Label
        title(behavior.robotModel.States.ddx.label{j});
        xlabel('Time(s)'); ylabel('ddq (rad/s^2)');
    end
    
    h = figure(1014); if (k==1) clf; end
    h.Name = 'b_pp'; h.WindowStyle = 'docked'; hold on; grid on;   
    for i = 1:3
        j = iBase(i);
        subplot(1,3,i); hold on; grid on;
        plot(x(j,:), dx(j,:), 'Color', mColor);
        if strcmp(plotType, 'optsim')
            plot(xOpt(j,:), dxOpt(j,:), '*');
        end
        title([behavior.robotModel.States.ddx.label{j},' phase']);
        xlabel('pos'); ylabel('vel (rad/s)');
    end    
    %----------------------------------------------------------------------
    %%% base position %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Leg states 
    % ---------------------------------------------------------------------
    h = figure(1021); 
    if(k==1) clf; end
    h.Name = 'leg_pos'; h.WindowStyle = 'docked'; hold on; grid on;
    %%% leg position
    for i = 1:4
        subplot(2, 2, i); hold on; grid on;        
        plot(t, x(i+3,:), 'Color', mColor);
        title(motorNames(i)); xlabel('Time(s)'); ylabel('q (rad)');
    end
    
    h = figure(1022); 
    if(k==1) clf; end
    h.Name = 'leg_vel'; h.WindowStyle = 'docked'; hold on; grid on;
    %%% leg position
    motorNames = domain.States.x.label(4:end);
    for i = 1:4
        subplot(2, 2, i); hold on; grid on;        
        plot(t, dx(i+3,:), 'Color', mColor);
        title(motorNames(i)); xlabel('Time(s)'); ylabel('q (rad)');
    end
    
    
    h = figure(1023); 
    if(k==1) clf; end
    h.Name = 'leg_acc'; h.WindowStyle = 'docked'; hold on; grid on;
    %%% leg position
    for i = 1:4
        subplot(2, 2, i); hold on; grid on;        
        plot(t, ddx(i+3,:), 'Color', mColor);
        title(motorNames(i)); xlabel('Time(s)'); ylabel('q (rad)');
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    
    %% COM position & velocity
    h = figure(120); 
    if (k==1) clf; end
    h.Name = 'COM'; h.WindowStyle = 'docked';
    subplot(2,2,1)
        plot(t, com(1,:), 'LineWidth', 2, 'Color', mColor); grid; hold on;
        title('comPos X'); %ylim([.6,1.2]);
    subplot(2,2,2)
        plot(t, com(4,:), 'LineWidth', 2, 'Color', mColor); grid; hold on;
        if strcmp(plotType, 'optsim')
            plot(tOpt, com_vel_opt(1,:), '*', 'LineWidth', 1);
        end
        title('comVel X'); %ylim([-2,2]);

    subplot(2,2,3)
        plot(t, com(3,:), 'LineWidth', 2, 'Color', mColor); grid; hold on;
        title('comPos Z'); %ylim([.8,1.2]);
    subplot(2,2,4)
        plot(t, com(6,:), 'LineWidth', 2, 'Color', mColor); grid; hold on;
        if strcmp(plotType, 'optsim')
            plot(tOpt, com_vel_opt(3,:), '*', 'LineWidth', 1);
        end
        title('comVel Z'); %ylim([-2,2]);
    
    h = figure(121); 
    if(k==1) clf; end
        h.Name = 'COM_pp'; h.WindowStyle = 'docked';
        subplot(2,1,1)
            plot(com(1,:), com(4,:), 'LineWidth', 2, 'Color', mColor); grid; hold on;
            xlabel('comPos X'); ylabel('comVel X');
        subplot(2,1,2)
            plot(com(3,:), com(6,:), 'LineWidth', 2, 'Color', mColor); grid; hold on;
            xlabel('comPos Z'); ylabel('comVel Z');
    
%     h = figure(122); 
%     if(k==1) clf; end
% 	h.Name = 'COM vs. Base'; h.WindowStyle = 'docked';
%     subplot(2,2,1)
%     plot(t, com(1,:)-x(1,:)); hold on; title('com-x - basePosX')
%     subplot(2,2,3)
%     plot(t, com(3,:)-x(2,:)); hold on; title('com-z - basePosZ')
%     
%     subplot(2,2,2)
%     plot(t, com(4,:)-dx(1,:)); hold on; title('comVelx - baseVelX')
%     subplot(2,2,4)
%     plot(t, com(6,:)-dx(2,:)); hold on; title('comVelz - baseVelZ')
    
    
    %% Torques (motor torque)
    h = figure(130); if(k==1) clf; end
    h.Name = 'u'; h.WindowStyle = 'docked'; hold on; grid on;
    for i = 1:size(u,1)
        subplot(2, 2, i); 
        plot(t, u(i,:), 'Color', mColor); hold on; grid on;
        
        if strcmp(plotType, 'optsim')
            plot(tOpt, uOpt(i,:), '*', 'LineStyle','--');
        end
        
        % Label
        xlabel('Time(s)'); ylabel('u (Nm)'); title(motorNames(i));
    end
        
    
    %% Outputs
    % this is hard coded, need to improve
    h = figure(132); if(k==1) clf; end
    h.Name = 'outputs'; h.WindowStyle = 'docked';

    indexY = 1:4;
    for iy = 1:length(indexY)
        subplot(2,2,indexY(iy))
        plot(t, ya(iy,:)); hold on; grid on;
        plot(t, yd(iy,:), 'b--'); hold on;
        title(logger(k).plant.VirtualConstraints.position.OutputLabel(iy));
    end

    % output errors
    h = figure(133); if(k==1) clf; end
    h.Name = 'tracking'; h.WindowStyle = 'docked';
    for iy = 1:length(indexY)
        subplot(2,2,indexY(iy))
        plot(t, ya(iy,:)-yd(iy,:)); hold on; grid on;
        title(logger(k).plant.VirtualConstraints.position.OutputLabel(iy));
    end
    

    %% Holonomic constraint violation and Contact force 
    %%% Constraint Wrenches %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ~isempty(wrenches) && k<3
        w_fields = fields(wrenches);
        
        wName = w_fields{1};
        wrench = wrenches.(wName);

        h = figure(151);
        if(k==1) clf; end
        h.Name = wName; h.WindowStyle = 'docked';
        hold on; grid on;

        % hard coded %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if size(wrench,1) == 1
            subplot(2,1,1)
                if endsWith(domain.Name,'P')
                    plot(t, -external.fx, 'r', 'LineWidth', 2); 
                else
                    plot(t, external.fx, 'r', 'LineWidth', 2); 
                end
                hold on; grid on; ylabel('CWrench (Nm)|(N)'); 
                title('Contact force, Fx');
            subplot(2,1,2)
                plot(t, wrench(1,:), 'b');
                hold on; grid on; xlabel('Time (s)'); ylabel('CWrench (Nm)|(N)');
                title('Contact force, Fz');
        else
            subplot(2,1,1)
                plot(t, wrench(1,:), 'b');
                hold on; grid on; ylabel('CWrench (Nm)|(N)');
                title('Contact force, Fx');
            subplot(2,1,2)
                plot(t, wrench(2,:), 'b');
                hold on; grid on; xlabel('Time (s)'); ylabel('CWrench (Nm)|(N)');
                title('Contact force, Fz');
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        
        
        %%% Holonomic violations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        h_fields = fields( logger(k).plant.HolonomicConstraints );
        for w = 1:numel(h_fields)

            hName = h_fields{w};        
            % Compute Holonomic data & plot it
            holo_pos = [];
            for i = 1:length(t)
                holo_func = logger(k).plant.HolonomicConstraints.(hName);
                holo_pos(:,i) = holo_func.calcConstraint(x(:,i));
            end

            h = figure(151+w); 
            if(k==1) clf; end
            h.Name = ['H', h_fields{w}]; h.WindowStyle = 'docked';
            hold on; grid on;

            nH = length(holo_pos(:,1));
            nrows = 2;
            ncols = ceil(nH/nrows);
            for j = 1:nH
                subplot(nrows,ncols,j); hold on; grid on;
                plot(t, holo_pos(j,:));
                xlabel('Time (s)'); 
                title('Holonomic constraint violation');
            end
        end
    end
    
    
    %% Feet Status
    h = figure(200); 
    if(k==1) clf; end
    h.Name = 'Feet'; h.WindowStyle = 'docked';
    subplot(2,2,1)
        plot(t, RF_pos(1,:), 'LineWidth', 2, 'Color', mColor); hold on
        plot(t, LF_pos(1,:), 'LineWidth', 2, 'Color', 'g'); hold on
        legend('rightFoot(stance)', 'leftFoot(non-stance)', 'Location','best');
        title('foot pos X');
    subplot(2,2,2)
        plot(t, RF_vel(1,:), 'LineWidth', 2, 'Color', mColor); hold on
        plot(t, LF_vel(1,:), 'LineWidth', 2, 'Color', 'g'); hold on
        legend('rightFoot(stance)', 'leftFoot(non-stance)', 'Location','best');
        title('foot vel X'); 
    subplot(2,2,3)
        plot(t, RF_pos(3,:), 'LineWidth', 2, 'Color', mColor); hold on
        plot(t, LF_pos(3,:), 'LineWidth', 2, 'Color', 'g'); hold on
        legend('rightFoot(stance)', 'leftFoot(non-stance)');
        title('foot pos Z');
    subplot(2,2,4)
        plot(t, RF_vel(3,:), 'LineWidth', 2, 'Color', mColor); hold on
        plot(t, LF_vel(3,:), 'LineWidth', 2, 'Color', 'g'); hold on
        legend('rightFoot(stance)', 'leftFoot(non-stance)');
        title('foot vel Z');
    
    %% Feet touch down placement %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    h = figure(210); 
    if(k==1) clf; end
    h.Name = 'touchDown'; h.WindowStyle = 'docked';
    subplot(1,2,1)
        plot(t, LF_pos(1,:),    'LineWidth', 2,  'Color', 'g'); hold on
        plot(t, x(1,:),         'LineWidth', 2,  'Color', mColor); hold on
        plot(t, com(1,:), '-.', 'LineWidth', 1.5, 'Color', mColor); grid on
        legend('left (nsf) X', 'Base X', 'COM X'); xlabel time;
        title 'nsf-X vs. com-X';
    subplot(1,2,2)
        plot(t, x(2,:),         'LineWidth', 2,  'Color', mColor); hold on
        plot(t, com(3,:), '-.', 'LineWidth', 1.5, 'Color', mColor); grid on
        legend('Base Z', 'COM Z'); xlabel time;
        title 'nsf-Z vs. com-Z';
        
       
end %-end of plotting


end