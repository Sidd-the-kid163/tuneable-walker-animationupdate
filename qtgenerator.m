function [q, t] = qtgenerator()

    %  Load YAML
    data = yaml.ReadYaml('All_Gaits/gait_1_1/params_gait_1_1.yaml');

    gait = data.domain{1};     % Python: data['domain'][0]

    
    %  Extract aposition 
    if iscell(gait.aposition)
        flat = cell2mat(gait.aposition);     % 1 x 48 double
    else
        flat = gait.aposition;               % Already numeric
    end

    
    % NumPy is row-major, MATLAB is column-major
    control_points = reshape(flat, 6, 8).';

    %  Build mod_control_points
    mod_control_points = cell(6,1);
    mod_control_points{1} = control_points(5,:);   % Matlab index does not start from 0
    mod_control_points{2} = control_points(3,:);    
    mod_control_points{3} = control_points(7,:);    
    mod_control_points{4} = control_points(8,:);    
    mod_control_points{5} = control_points(4,:);    
    mod_control_points{6} = control_points(3,:);           
    mod_control_points{7} = control_points(2,:);    
    mod_control_points{8} = control_points(1,:);    
    %% -------------------------------------------------------
    %  Parameter vector t = np.linspace(0,1,200)
    % --------------------------------------------------------
    t = linspace(0.0, 1.0, 200);

    %% -------------------------------------------------------
    %  Evaluate trajectories
    % --------------------------------------------------------
    q = zeros(length(t), 11);   % 3 zeros + 8 Bézier values

    for k = 1:length(t)

        ti = t(k);
        x = zeros(1,11);        % x = [] each loop in Python

        % First three zeros (x.append(0), x.append(0), x.append(0))
        % Already zero, do nothing

        % Fill Bézier outputs (order identical to Python)
        for j = 1:8
            x(j+3) = bezier_point_1d(mod_control_points{j}, ti);
        end

        q(k,:) = x;
    end
    q = q.';

end


function value = bezier_point_1d(control_points, t)
    n = 5;
    value = 0.0;

    for i = 0:n
        bern = nchoosek(n, i) * (t^i) * (1 - t)^(n - i);
        value = value + bern * control_points(i+1);
    end
end