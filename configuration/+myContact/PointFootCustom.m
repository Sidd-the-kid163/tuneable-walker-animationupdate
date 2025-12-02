function [domain] = PointFootCustom(domain, foot)
%
% Contact dynamics for point foot 
%
% _________________________________________________________________________

if strcmp(foot, 'Left')
    contactPoint = domain.feet.LeftMidFoot;
elseif strcmp(foot, 'Right')
    contactPoint = domain.feet.RightMidFoot;
else
    error('Not a valid leg, ONLY accept Left or Right.');
end
contact = ToContactFrame(contactPoint, 'PointContactWithFriction');

%%% Friction Cone parameters (defined in RobotLinks model instead)
% fric_coef.mu = 0.6;  %-translational friction
% fric_coef.gamma = 25; %-torsional friction
% fric_coef.mu * norm(domain.p_toe - domain.p_midfoot);

%% Wrench base
ref = eye(3);
  I = eye(6);
if strcmp(domain.Joints(3).Name, 'BaseRotY')
    planar = true;
    wrenchBase = I(:,[1,3]); % x, z
else
    planar = false;
    wrenchBase = I(:,[1,2,3,6]); % (x, y, z, yaw)
    % wrenchBase = I(:,[1,2,3]); % (x, y, z), yaw constraint is enforced later.
end
G = [eye(3),     zeros(3,3);
     zeros(3,3), ref        ] * wrenchBase;
    
%%% compute the spatial position of contact points (cartesian position + Euler angles)
pos = getCartesianPosition(domain, contact);
rpy = getRelativeEulerAngles(domain, contact, ref);

%%% extract the contrained elements
constr =  G' * transpose([pos, rpy]);

%% this might be a BUG
% Read Chap3.4 (Chap2.4 helps too) to understand

%%% to debug:
% keyboard;
% run('debug/testJac.m');

%%% the defacult, which is copied from $ addContact.m, line 59-61
%   It might be wrong for FEET walking
jac_pos = jacobian(pos, domain.States.x);
jac_rot = getBodyJacobian(domain, contact);
jac = [jac_pos; jac_rot(4:6,:)];


constr_jac = wrenchBase' * jac;


%% labels for holonomic constraint
label_full = cellfun(@(x)[contact.Name,x],...
             {'PosX','PosY','PosZ','Roll','Pitch','Yaw'},'UniformOutput',false);
for i = size(wrenchBase,2):-1:1
    label{i} = label_full{wrenchBase(:,i)==1};
end
    
%%% create a holonomic constraint object
contact_constr = HolonomicConstraint(domain, constr, contact.Name,...
                                     'Jacobian', constr_jac,...
                                     'ConstrLabel', {label},...
                                     'DerivativeOrder',2);
domain = addHolonomicConstraint(domain, contact_constr);

%% contact wrench input vector (Constraint force)
f_name = contact_constr.InputName;
f      = domain.Inputs.ConstraintWrench.(f_name);
    
%% Friction Cone
% get the friction cone constraint
mu       = SymVariable('mu');
gamma    = SymVariable('gamma');
fun_name = ['u_friction_cone_', domain.Name];
    
if planar
    % x, y, z, pitch
    constr = [f(2); % fz >= 0
              f(1) + (mu/sqrt(2))*f(2);  % -mu/sqrt(2) * fz < fx
             -f(1) + (mu/sqrt(2))*f(2)]; % fx < mu/sqrt(2) * fz

    % create a symbolic function object
    f_constr = SymFunction(fun_name, constr,{f},{mu});

    % create the label text
    label = {'normal_force'; 'friction_x_pos'; 'friction_x_neg'};

    % validate the provided static friction coefficient
    % validateattributes(fric_coef.mu,{'double'},...
    %                    {'scalar','real','>=',0},...
    %                    'ContactFrame.getFrictionCone','mu');
    % auxdata = fric_coef.mu;
    auxdata = domain.fric_coef.mu;
    
else
    % x, y, z, yaw
    constr = [f(3); % fz >= 0
              f(1) + (mu/sqrt(2))*f(3);  % -mu/sqrt(2) * fz < fx
             -f(1) + (mu/sqrt(2))*f(3);  % fx < mu/sqrt(2) * fz
              f(2) + (mu/sqrt(2))*f(3);  % -mu/sqrt(2) * fz < fu
             -f(2) + (mu/sqrt(2))*f(3);  % fy < mu/sqrt(2) * fz
              f(4) + gamma * f(3);       % -gamma * fz < wz
             -f(4) + gamma * f(3)        % wz < gamma * fz
            ];
    % create a symbolic function object
    f_constr = SymFunction(fun_name, constr,{f},{[mu;gamma]});

    % create the label text
    label = {'normal_force';
             'friction_x_pos'; 'friction_x_neg'; 'friction_y_pos'; 'friction_y_neg';
             'tor_friction_neg'; 'tor_friction_pos'
            };

    % validate the provided static friction coefficient
    validateattributes(fric_coef.mu,{'double'},...
        {'scalar','real','>=',0},...
        'ContactFrame.getFrictionCone','mu');

    % validate the provided torsional friction coefficient
    validateattributes(fric_coef.gamma,{'double'},...
        {'scalar','real','>=',0},...
        'ContactFrame.getFrictionCone','gamma');
    auxdata = [fric_coef.mu; fric_coef.gamma];
end    
    
% create an unilateral constraint object
fc_cstr = UnilateralConstraint(domain, f_constr,...
                                ['fc' contact.Name], f_name, ...
                                'ConstrLabel',{label(:)'},...
                                'AuxData',auxdata);
domain = addUnilateralConstraint(domain, fc_cstr);
        
end

