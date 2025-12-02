function [ nlp ] = applyCost( behavior, nlp, CostType, weight )
% To apply a cost function to the continous domains (vertex) of nlp. 
%
% _________________________________________________________________________

% Access nlp variables
x  = behavior.robotModel.States.x;
dx = behavior.robotModel.States.dx;
ddx = behavior.robotModel.States.ddx;
mg   = 9.81 * sum([behavior.robotModel.Links(:).Mass]);

% Assign cost to each vertex
vertices = fields(behavior.vertices);
for i = 1:numel(vertices)
    
    phaseIndex = nlp.getPhaseIndex(behavior.vertices.(vertices{i}).Name);
    phase = nlp.Phase(phaseIndex);
    domain = phase.Plant;
    
    u  = domain.Inputs.Control.u;
    Be = domain.Gmap.Control.u;
    p  = domain.Params.pposition;
    % a  = domain.Params.aposition;
    a = SymVariable(tomatrix(domain.Params.aposition(:)));

    %%% Find actuated joints (hard-code for AMBER)
    actuatedJoints = x.label(4:end);
    % actuatedJoints = {'RightKnee', 'RightHip', 'LeftHip', 'LeftKnee'};
    
    switch CostType{i}
        case 'mCOT'
            cot     = weight.*sqrt(sum((tovector(u)*(Be'*dx)).^2)).^2 / (mg * dx('BasePosX'));
            cot_fun = SymFunction(['cot_' phase.Name], cot, {u, dx});
            addRunningCost(nlp.Phase(phaseIndex), cot_fun, {'u', 'dx'});
            
            
        case 'TorqueSquare'
            u2 = weight .* tovector(norm(u).^2);
            u2_fun = SymFunction(['torque_', phase.Name], u2, {u});
            addRunningCost(nlp.Phase(phaseIndex), u2_fun, {'u'});
            
        case 'JointOnlyTorqueSquare'
            inds = [1,3,5,7];
            u2 = weight .* tovector(norm(u(inds)).^2);
            u2_fun = SymFunction(['jointtorque_', phase.Name], u2, {u});
            addRunningCost(nlp.Phase(phaseIndex), u2_fun, {'u'});
        
            
        case 'Zero'
            Czero_fun = SymFunction('Czero', 0, {x});
            addRunningCost(nlp.Phase(phaseIndex), Czero_fun, {'x'});
            
            
        case 'SLIP'
            tau = domain.VirtualConstraints.position.PhaseFuncs{1};
            slip_params = load('cassie_slip');
            a_slip_bezier  = slip_params.slip_a_mat;
            
            pCOMslip  = bezier(a_slip_bezier, tau);
            pCOMrobot = domain.getComPosition';
            
            SLIP_norm = (pCOMrobot(1) - pCOMslip(1)).^2 + ...
                        (pCOMrobot(3) - pCOMslip(2)).^2;
            SLIP_fun = SymFunction(['slip_' phase.Name], SLIP_norm, {x, p});
            addRunningCost(nlp.Phase(phaseIndex), SLIP_fun, {'x', 'pposition'});
            
            
        case 'Jerk'
            % Opt.jerkCost;
            actuatedIdx = phase.Plant.getJointIndices(actuatedJoints);
            
            % add variable as "acc of preivous node"
            ddxPrev = SymVariable('ddxM', [phase.Plant.numState,1]);

            % cost function at node i, (i <= N-1)
            ddxActuated = tovector(ddx(actuatedIdx));
            ddxMActuated = tovector(ddxPrev(actuatedIdx));
            costNode = weight .* tovector( sum( (ddxActuated-ddxMActuated).^2 ));
            costNode_func = SymFunction('JerkNodeCost', costNode, {ddx, ddxPrev});

            cost(phase.NumNode-1) = struct();
            [cost.Name] = deal(costNode_func.Name);
            [cost.Dimension] = deal(1);
            [cost.SymFun] = deal(costNode_func);

            vars = phase.OptVarTable;
            for j = 2 : phase.NumNode
                cost(j-1).DepVariables = [ vars.ddx(j), vars.ddx(j-1) ]; 
            end

            %%% add to NLP
            addCost(phase, 'jerkSum','except-first', cost);
            
            
        case 'accMovement'
            % minize the accleration of actuated joints.
            tmp = ddx(actuatedJoints);
            accMov = weight .* tovector(sum(tmp.^2) );
            accMovFun = SymFunction(['accMovement_', phase.Name], accMov, {ddx});
            addRunningCost(nlp.Phase(phaseIndex), accMovFun, {'ddx'});
            
            
        case 'BaseMovement'
            if strcmp(behavior.robotModel.Joints(6).Name, 'BaseRotZ')
                qbIndices = 1:6;
            else
                qbIndices = 1:3;
            end
            
            baseMov = weight .* tovector(sum(dx(qbIndices).^2));
            baseMovFun = SymFunction(['BaseMovement_', phase.Name], baseMov, {dx});
            addRunningCost(nlp.Phase(phaseIndex), baseMovFun, {'dx'});
            
            
        case 'NSFMovement'
            p_nsf = getCartesianPosition(domain, domain.ContactPoints.LeftMidFoot);
            v_nsf = jacobian(p_nsf, x) * dx;
            %v_nsf(3)=[]; %-don't care nsf_velZ
            
            nsf_vel_norm = weight .* tovector(sum(v_nsf.^2));
            nsf_vel_norm_Fun = SymFunction(['NSFMovement_', phase.Name], nsf_vel_norm, {x, dx});
            addRunningCost(nlp.Phase(phaseIndex), nsf_vel_norm_Fun, {'x','dx'});
            
            
        case 'Movement'
            baseWeight = [0; 0; 1; 0; 0; 0];
            FeetWeight = [1; 1; 1];
            
            baseMov = dx(1:6).*baseWeight;
            
            p_nsf = getCartesianPosition(domain, domain.ContactPoints.LeftMidFoot);
            v_nsf = jacobian(p_nsf, x) * dx;
            nsfMove = v_nsf.*FeetWeight;
            
            Mov = weight .* tovector( sum(baseMov.^2) + sum(nsfMove.^2) );
            Mov_Fun = SymFunction(['Movement_', phase.Name], Mov, {x, dx});
            addRunningCost(nlp.Phase(phaseIndex), Mov_Fun, {'x','dx'});
            
    end
end

end