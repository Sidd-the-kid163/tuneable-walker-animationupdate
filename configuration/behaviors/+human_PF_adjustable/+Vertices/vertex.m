function domain = vertex(model)
% Define dynamics & control for all domains.
% model: a robotLinks model
% _________________________________________________________________________


%% Initialize this domain
domain = copy(model);
domain.setName('v1');

%%% states
x = domain.States.x;
dx = domain.States.dx;

%%% contacts
nsf_pos = domain.getCartesianPosition(domain.feet.LeftMidFoot); 
sf_pos  = domain.getCartesianPosition(domain.feet.RightMidFoot);
domain = myContact.PointFootCustom(domain, 'Right');

%%% discrete event: nsf reaches ground
tmp = UnilateralConstraint(domain, nsf_pos(3), 'nsf', 'x');  
domain = domain.addEvent(tmp);


%% Phase variable

%%% linearized hip position
hip_pos = domain.getCartesianPosition(domain.Joints(getJointIndices(domain,'RightHip')));
phip = hip_pos(1) - sf_pos(1);
deltaphip = linearize(phip, x);

%%% define tau(q)
p = SymVariable('p',[2,1]);
tau = (deltaphip - p(2))/(p(1)-p(2));


%% RD2 Outputs
sl='Right'; nsl = 'Left'; 
%%% define the (RD2) actual outputs and its labels 
ya_2    = [x([sl, 'ShinExtension']); x([sl, 'Knee']); x([sl, 'ThighExtension']); x([sl, 'Hip']); x([nsl, 'Hip']); x([nsl, 'ThighExtension']); x([nsl, 'Knee']); x([nsl, 'ShinExtension'])];
y2_label = { [sl, 'ShinExtension'],    [sl, 'Knee'],    [sl, 'ThighExtension'], [sl, 'Hip'],    [nsl, 'Hip'],  [nsl, 'ThighExtension'],  [nsl, 'Knee'], [nsl, 'ShinExtension']};

y2 = VirtualConstraint(domain, ya_2, 'position', ...
                       'DesiredType', 'Bezier', ...
                       'PolyDegree',     5,...
                       'RelativeDegree', 2, ...
                       'OutputLabel',    {y2_label}, ...
                       'PhaseType',      'StateBased', ...
                       'PhaseVariable',  tau, ...
                       'PhaseParams',    p, ...
                       'Holonomic',      true);

domain = domain.addVirtualConstraint(y2);

domain.PreProcess = @Edges.NewStepPreProcess;

end