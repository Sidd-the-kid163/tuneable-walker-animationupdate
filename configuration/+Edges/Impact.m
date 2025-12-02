function guard = Impact(tar, impactFootName, isRelabel)
% Applies a rigid impact, through nonstance leg.
% _________________________________________________________________________

if isRelabel
    guard = RigidImpact([impactFootName,'ImpactRelabel'],tar,'nsf');
else
    guard = RigidImpact([impactFootName,'Impact'],       tar,'nsf');
end

%% Re-define the relabel matrix for symmetric optimization
if isRelabel
    jointName = {tar.Joints.Name};

    % Indices of right leg joints
    qRLegIndices = find(strncmpi(jointName,'Right',5));

    % Indices of left leg joints
    qLLegIndices = find(strncmpi(jointName,'Left',4));

    % mirror left to right dof, and right to left dof.
    swappingIndices = cumsum(ones(tar.numState,1));
    swappingIndices(qRLegIndices) = qLLegIndices;
    swappingIndices(qLLegIndices) = qRLegIndices;

    % find all roll joints
    rollJoints = strfind(jointName,'Roll');
    rollJointIndices = find(~cellfun(@isempty,rollJoints));

    % find all yaw joints
    yawJoints = strfind(jointName, 'Yaw');
    yawJointIndices = find(~cellfun(@isempty,yawJoints));

    % flip signs of roll and yaw angles (not needed for 2D)
    swappingSign = ones(tar.numState,1);
    swappingSign(rollJointIndices) = -1*ones(numel(rollJointIndices),1);
    swappingSign(yawJointIndices)  = -1*ones(numel(yawJointIndices),1);

    % No base swapping needed if 2D
    if strcmp(tar.Joints(6).Name, 'BaseRotZ')
        qbIndices = 1:6;
        swappingIndices(qbIndices) = qbIndices;
        swappingSign(qbIndices(2)) = -1; % BasePosY
        swappingSign(qbIndices(4)) = -1; % base-roll: BaseRotX
        swappingSign(qbIndices(6)) = -1; % base-yaw : BaseRotZ
    end

    relabel = diag(swappingSign);
    R = relabel(swappingIndices,:);
    guard.R = R;
end

%% Set the impact constraint
guard.addImpactConstraint(struct2cell(tar.HolonomicConstraints));

end