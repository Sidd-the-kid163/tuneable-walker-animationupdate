function [ nlp ] = setupOpt( behavior , userInputs)
% This function defines cost function and NLP constraints.
import([behavior.name, '.Constraints.*']);

%% Customize model boundaries (tunable)
model_bounds = behavior.robotModel.getLimits();
model_bounds.states.ddx.lb = -150 * ones(behavior.robotModel.numState,1);
model_bounds.states.ddx.ub =  150 * ones(behavior.robotModel.numState,1);

model_bounds.states.x.x0 = zeros(behavior.robotModel.numState,1);

name = 'BasePosX';
idx = behavior.robotModel.getJointIndices(name);
model_bounds.states.x.lb(idx) = -0.4;
model_bounds.states.x.ub(idx) = 0.6;
model_bounds.states.x.x0(idx) = -0.2;
model_bounds.states.dx.lb(idx) = 0.1;
model_bounds.states.dx.ub(idx) = 2;

name = 'BasePosZ';
idx = behavior.robotModel.getJointIndices(name);
model_bounds.states.x.lb(idx) =0.7;
model_bounds.states.x.ub(idx) = 2;
model_bounds.states.x.x0(idx) = 1;
model_bounds.states.dx.lb(idx) = -1;
model_bounds.states.dx.ub(idx) = 1;

name = 'BaseRotY';
idx = behavior.robotModel.getJointIndices(name);
model_bounds.states.x.lb(idx) = 0;
model_bounds.states.x.ub(idx) = 0.2;
model_bounds.states.x.x0(idx) = 0.1;
model_bounds.states.dx.lb(idx) = -1;
model_bounds.states.dx.ub(idx) = 1;

idx = behavior.robotModel.getJointIndices({'RightHip','LeftHip'});
model_bounds.states.x.lb(idx) = deg2rad(-90);
model_bounds.states.x.ub(idx) = deg2rad(90);
model_bounds.states.dx.lb(idx) = -2;
model_bounds.states.dx.ub(idx) =  2;

idx = behavior.robotModel.getJointIndices({'RightKnee','LeftKnee'});
model_bounds.states.x.lb(idx) = deg2rad(10);
model_bounds.states.x.ub(idx) = deg2rad(80);
model_bounds.states.dx.lb(idx) = -2;
model_bounds.states.dx.ub(idx) =  2;

idx = behavior.robotModel.getJointIndices({'LeftThighExtension','RightThighExtension'});
delta_thighlength = userInputs.thighlength-0.4;
model_bounds.states.x.lb(idx) = delta_thighlength-0.01;
model_bounds.states.x.ub(idx) = delta_thighlength+0.01;
model_bounds.states.dx.lb(idx) = -0.1;
model_bounds.states.dx.ub(idx) = 0.1;

idx = behavior.robotModel.getJointIndices({'LeftShinExtension','RightShinExtension'});
delta_shanklength = userInputs.shanklength-0.4;
model_bounds.states.x.lb(idx) = delta_shanklength-0.01;
model_bounds.states.x.ub(idx) = delta_shanklength+0.01;
model_bounds.states.dx.lb(idx) = -0.1;
model_bounds.states.dx.ub(idx) = 0.1;

des_sd = userInputs.step_length/userInputs.average_vel;
step_duration = [des_sd-0.2, des_sd+0.2]; %essential: how long should a step last

%% Use Inputs
model_bounds.averageVel = userInputs.average_vel;
model_bounds.min_foot_clearance = userInputs.min_foot_clearance;
model_bounds.impact_vel = userInputs.impact_vel;
model_bounds.step_length = userInputs.step_length;
model_bounds.max_height_tau = userInputs.max_height_tau;

%% % RD2 outputs
model_bounds.position.kp = 600;
model_bounds.position.kd = 40;

% trajectories coefficient
model_bounds.params.aposition.lb = -5; 
model_bounds.params.aposition.ub =  5;

% phase variable: linearized hip position
model_bounds.params.pposition.lb = [0.06, -0.4]; 
model_bounds.params.pposition.ub = [0.4,  -0.06];

%% Domain-specified bounds (Better Not Tune)
bounds = struct();
vNames = fields(behavior.vertices);
eNames = fields(behavior.edges);
bounds.(vNames{1}) = model_bounds;
bounds.(eNames{1}) = model_bounds;
bounds.(eNames{1}).states.xn = model_bounds.states.x;
bounds.(eNames{1}).states.dxn = model_bounds.states.dx;

% Torque limits
% trans = 91.4286;
% torque_limit = 60/trans;
% bounds.(vNames{1}).inputs.Control.u.lb = -torque_limit * ones(8,1);
% bounds.(vNames{1}).inputs.Control.u.ub =  torque_limit * ones(8,1);

%%% Time
bounds.(vNames{1}).time.t0.lb = 0;
bounds.(vNames{1}).time.t0.ub = 0;
bounds.(vNames{1}).time.t0.x0 = 0;
bounds.(vNames{1}).time.tf.lb = step_duration(1); 
bounds.(vNames{1}).time.tf.ub = step_duration(2);
bounds.(vNames{1}).time.tf.x0 = mean(step_duration);

%%% Constraint Wrench Forces 
bounds.(vNames{1}).inputs.ConstraintWrench.fRightMidFoot.lb = [-300,-10];
bounds.(vNames{1}).inputs.ConstraintWrench.fRightMidFoot.ub = [300, 500];
bounds.(vNames{1}).inputs.ConstraintWrench.fRightMidFoot.x0 = [0, 300];  
bounds.(vNames{1}).params.pRightMidFoot.lb = zeros(2,1);
bounds.(vNames{1}).params.pRightMidFoot.ub = zeros(2,1);
bounds.(vNames{1}).params.pRightMidFoot.x0 = zeros(2,1);

%% Setup the NLP
behavior.vertices.(vNames{1}).UserNlpConstraint = str2func('V_constraints');
behavior.edges.(eNames{1}).UserNlpConstraint = str2func('E_constraints');

num_grid.(vNames{1}) = 6;
nlp = HybridTrajectoryOptimization(behavior.name, behavior.hybridSystem, num_grid, ...
                                   [], 'EqualityConstraintBoundary', 1e-4);
nlp.configure(bounds);


%% Add a cost function
weight= 1e-1;
CostType = {'JointOnlyTorqueSquare'};   %-{'TorqueSquare', 'Jerk', 'accMovement'}
nlp = Opt.applyCost(behavior, nlp, CostType, weight);

% weight= 1e1;
% CostType = {'accMovement'};   %-{'TorqueSquare', 'Jerk', 'accMovement'}
% nlp = Opt.applyCost(behavior, nlp, CostType, weight);
% 
% weight= 1e1;
% CostType = {'BaseMovement'};   %-{'TorqueSquare', 'Jerk', 'accMovement'}
% nlp = Opt.applyCost(behavior, nlp, CostType, weight);


nlp.update;


end