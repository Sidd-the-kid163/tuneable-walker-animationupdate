function V_constraints(nlp, bounds, varargin)
% NL constraint on a vertex

domain = nlp.Plant;
x = domain.States.x;
dx = domain.States.dx;
u  = domain.Inputs.Control.u;
N = nlp.NumNode;

%% RD 2 output (Better Not Tune)
domain.VirtualConstraints.position.imposeNLPConstraint(nlp, ...
                     [bounds.position.kp, bounds.position.kd], [1, 1]);

%%% RD2 tau boundary [0,1]
tau = domain.VirtualConstraints.position.PhaseFuncs{1};
addNodeConstraint(nlp, tau, {'x','pposition'}, 'first', 0, 0, 'Nonlinear');
addNodeConstraint(nlp, tau, {'x','pposition'}, 'last',  1, 1, 'Nonlinear');


%% functions for feet (do not change)
sf_pos = domain.getCartesianPosition(domain.feet.RightMidFoot);
sf_vel = jacobian(sf_pos, x) * dx;
nsf_pos = domain.getCartesianPosition(domain.feet.LeftMidFoot);
nsf_vel = jacobian(nsf_pos, x) * dx;

% sf_posX_fun = SymFunction('sf_posX', sf_pos(1), {x});
% sf_posZ_fun = SymFunction('sf_posZ', sf_pos(3), {x});
% sf_velX_fun = SymFunction('sf_velX', sf_vel(1), {x,dx});
% sf_velZ_fun = SymFunction('sf_velZ', sf_vel(3), {x,dx});
nsf_posX_fun = SymFunction('nsf_posX', nsf_pos(1), {x});
nsf_posZ_fun = SymFunction('nsf_posZ', nsf_pos(3), {x});
nsf_velX_fun = SymFunction('nsf_velX', nsf_vel(1), {x, dx});
nsf_velZ_fun = SymFunction('nsf_velZ', nsf_vel(3), {x, dx});

% Delta for inequality -> equality constraints
delta_percent = 0.1;

%% Fixed Constraints
%%% NSF always move forward (stay fixed)
addNodeConstraint(nlp, nsf_velX_fun, {'x','dx'}, 'all', -0.2, 1.5, 'Nonlinear');
addNodeConstraint(nlp, nsf_velX_fun, {'x','dx'}, 1:floor(N/2), -0.01, 1.5, 'Nonlinear');

%%% redundant (stay fixed)
addNodeConstraint(nlp, nsf_posZ_fun, {'x'}, 'first', 0, 0,'Nonlinear');
addNodeConstraint(nlp, nsf_posZ_fun, {'x'}, 'last', 0, 0,'Nonlinear');

%%% NSF always above ground (stay fixed)
addNodeConstraint(nlp, nsf_posZ_fun, {'x'}, 'last', 0, 0.5,'Nonlinear'); 
addNodeConstraint(nlp, nsf_posZ_fun, {'x'}, floor(N*0.2):floor(N*0.8), ...
                                                 0.02, 0.5,'Nonlinear'); 


%% nonstance foot (essential )

%%% lift off velocity
addNodeConstraint(nlp, nsf_velZ_fun, {'x','dx'}, 'first', 0.01, 0.5, 'Nonlinear');

%%% foot clearance (tune)
addNodeConstraint(nlp, nsf_posZ_fun, {'x'}, floor(N*bounds.max_height_tau), bounds.min_foot_clearance, 0.3,'Nonlinear');
 
%%% impact velocity (tune)
addNodeConstraint(nlp, nsf_velZ_fun, {'x','dx'}, 'last', -0.8, -0.2, 'Nonlinear');

%%% impact velocity (tune)
addNodeConstraint(nlp, nsf_velZ_fun, {'x','dx'}, 'last', ...
                    bounds.impact_vel-(delta_percent*0.05), ...
                    bounds.impact_vel+(delta_percent*0.05), ...
                    'Nonlinear');

%%% Step length (final posture) (tune)
% stepLength_fun = SymFunction('stepLength', nsf_pos(1), {x});
addNodeConstraint(nlp, nsf_posX_fun, {'x'}, 'last',...
                    bounds.step_length-(delta_percent*0.05), ...
                    bounds.step_length+(delta_percent*0.05), ...
                    'Nonlinear');

%% Average walking speed (essential )
% v_target = [0.3, 0.6];

v_target = [bounds.averageVel-(delta_percent*0.05),...
            bounds.averageVel+(delta_percent*0.05)];

T0  = SymVariable('t0',  [2, 1]);
TF  = SymVariable('tf',  [2, 1]);
X0  = SymVariable('x0', [domain.numState,1]);
XF  = SymVariable('xF', [domain.numState,1]);
avg_vel = (XF(1) - X0(1)) / (TF(2) - T0(1)); 
avg_vel_fun = SymFunction('average_velocity', avg_vel, {T0, TF, X0, XF});
avg_vel_cstr = NlpFunction('Name','average_velocity',...
    'Dimension',1, ...
    'lb', v_target(1), ...
    'ub', v_target(2), ...
    'Type','Nonlinear', ...
    'SymFun', avg_vel_fun,...
    'DepVariables', [nlp.OptVarTable.T(1); nlp.OptVarTable.T(end); ...
                     nlp.OptVarTable.x(1); nlp.OptVarTable.x(end)]);
addConstraint(nlp, 'average_velocity', 'last', avg_vel_cstr);

% %% other tunable constraints
% torsoFun = SymFunction('torsoAngle', x(3), {x});
% addNodeConstraint(nlp, torsoFun, {'x'}, 'all', -0.05, 0.15,'Nonlinear');

end