function exportVirtualConstraint(behavior, outFile)
% exportVirtualConstraint
% 
% Extracts the ENTIRE Virtual Constraint (VC) definition from a FROST
% behavior and saves it into a readable JSON file.
%
% USAGE:
%   hs = behavior.hybridSystem;
%   exportVirtualConstraint(behavior, 'VC_definition.json');
%
% The JSON file will contain:
%   - controlled joints
%   - polynomial degree
%   - phase variable definition
%   - actual and desired output functions (symbolic names)
%   - parameter names and values
%   - relative degree, dimension, labels, types, etc.
%
% This is the closest equivalent to exporting a URDF for VC structure.

    % ----------------------------------------------------------
    % 1. Get the domain (assume single domain gait)
    % ----------------------------------------------------------
    hs = behavior.hybridSystem;
    dom = hs.Gamma.Nodes.Domain{1};

    % ----------------------------------------------------------
    % 2. Extract the actual VC
    % ----------------------------------------------------------
    if isfield(dom.VirtualConstraints, 'position')
        vc = dom.VirtualConstraints.position;
    else
        error('No "position" virtual constraint found in domain.');
    end

    % ----------------------------------------------------------
    % 3. Create export struct with all properties
    % ----------------------------------------------------------
    vc_data = struct();
    props = properties(vc);

    for i = 1:length(props)
        try
            val = vc.(props{i});

            % Symbolics and function handles cannot be JSON encoded directly
            if isa(val, 'SymFunction') || isa(val, 'function_handle')
                vc_data.(props{i}) = class(val);  % store type only
            else
                vc_data.(props{i}) = val;
            end

        catch
            vc_data.(props{i}) = 'UNREADABLE_FIELD';
        end
    end

    % ----------------------------------------------------------
    % 4. Add joint names corresponding to VC outputs
    % ----------------------------------------------------------
    try
        vc_data.ControlledJoints = vc.OutputLabel;
    catch
        vc_data.ControlledJoints = {};
    end

    % ----------------------------------------------------------
    % 5. Add mapping to robot joints (URDF order)
    % ----------------------------------------------------------
    try
        robotJoints = {dom.Joints.Name};
        vc_data.URDF_JointOrder = robotJoints;
    catch
        vc_data.URDF_JointOrder = {};
    end

    % ----------------------------------------------------------
    % 6. Convert to JSON-safe form
    % ----------------------------------------------------------
    json_string = jsonencode(vc_data, PrettyPrint=true);

    % ----------------------------------------------------------
    % 7. Write to file
    % ----------------------------------------------------------
    fid = fopen(outFile, 'w');
    fwrite(fid, json_string);
    fclose(fid);

    fprintf('\n✔ Virtual Constraint exported to %s\n', outFile);

end
