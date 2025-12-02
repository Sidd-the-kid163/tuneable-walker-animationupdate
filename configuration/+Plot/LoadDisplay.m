function robot_disp = LoadDisplay(behavior, varargin)
    robot = behavior.robotModel;
    skipExport = true;
    
    root_path = pwd;
    export_path = fullfile(root_path, 'export', behavior.name, 'animator');
    if ~exist(export_path,'dir')
        mkdir(export_path);
        skipExport = false;
    end
    addpath(export_path);
    
    if nargin > 4
        options = varargin(2:end);
        num_sim = varargin{1};
    elseif nargin > 1
        options = {'UseExported', true, 'ExportPath', export_path, 'SkipExporting', skipExport};
        num_sim = varargin{1};
    else        
        options = {'UseExported', true, 'ExportPath', export_path, 'SkipExporting', skipExport};
        num_sim = 1;
    end
    %--- checks to see if folder path exists and if it doesnt, then it
    %exports
    f = figure(num_sim); clf;
    robot_disp = frost.Animator.Display(f, robot, options{:});
    %---loads the robot's urdf and builds 3d objects
    % Remove Extension
    % keys = robot_disp.items.keys;
    % % Remove Extension Joints
    % robot_disp.removeItem('Joint_RightThighExtension');
    % robot_disp.removeItem('Joint_LeftThighExtension');
    % robot_disp.removeItem('Joint_RightShinExtension');
    % robot_disp.removeItem('Joint_LeftShinExtension');
    % 
    % % Remove Links Assosciated with Extension Joints
    % robot_disp.removeItem('Link_LeftThighUpper_to_LeftThighExtension');
    % robot_disp.removeItem('Link_RightThighUpper_to_RightThighExtension');
    % robot_disp.removeItem('Link_LeftThighLower_to_LeftKnee');
    % robot_disp.removeItem('Link_RightThighLower_to_RightKnee');
    % robot_disp.removeItem('Link_LeftShinUpper_to_LeftShinExtension');
    % robot_disp.removeItem('Link_RightShinUpper_to_RightShinExtension');

    % Draw back links
    % links = {robot.Links.Name};
    % joints = {robot.Joints.Name};
    % name = 'Link_LeftThigh_to_LeftKnee';
    % lh = find(contains(joints,'LeftHip'));
    % 
    % frame = CoordinateFrame('Name',robot.Joints(lh).Name,...
    %     'Reference',robot.Joints(lh),...
    %     'Offset',[0,0,0],...
    %     'R',[0,0,0]);
    % offset = obj.model.Joints(i).Offset;
    % obj.items(name) = frost.Animator.Cylinder(obj.axs, robot, frame, offset, name, varargin{:});
    % 

    if endsWith(behavior.name, 'FF') 
        % Add the toe contact points as black spheres
        leftToe = robot.feet.LeftToe;
        leftHeel = robot.feet.LeftHeel;
        rightToe = robot.feet.RightToe;
        rightHeel = robot.feet.RightHeel;

        leftToeItem = frost.Animator.Sphere(robot_disp.axs, robot, leftToe, leftToe.Name, options{:});
        leftHeelItem = frost.Animator.Sphere(robot_disp.axs, robot, leftHeel, leftHeel.Name, options{:});
        rightToeItem = frost.Animator.Sphere(robot_disp.axs, robot, rightToe, rightToe.Name, options{:});
        rightHeelItem = frost.Animator.Sphere(robot_disp.axs, robot, rightHeel, rightHeel.Name, options{:});

        robot_disp.addItem(leftToeItem);
        robot_disp.addItem(leftHeelItem);
        robot_disp.addItem(rightToeItem);
        robot_disp.addItem(rightHeelItem);
    end
    leftMidFoot = robot.feet.LeftMidFoot;
    rightMidFoot = robot.feet.RightMidFoot;
    leftMFItem = frost.Animator.Sphere(robot_disp.axs, robot,  leftMidFoot,  leftMidFoot.Name, options{:});
    rightMFItem = frost.Animator.Sphere(robot_disp.axs, robot, rightMidFoot, rightMidFoot.Name, options{:});
    robot_disp.addItem(leftMFItem);
    robot_disp.addItem(rightMFItem);
 
    %% shin link
    name = 'Link_RightKnee_to_RightMidFoot';
    frame = CoordinateFrame('Name', rightMidFoot.Reference.Name,...
                            'Reference', rightMidFoot.Reference,...
                            'Offset',[0,0,0],...
                            'R',[0,0,0]);
    offset = rightMidFoot.Offset;
    item = frost.Animator.Cylinder(robot_disp.axs, robot, frame, offset, name, options{:});
    robot_disp.addItem(item);
    
    name = 'Link_leftKnee_to_leftMidFoot';
    frame = CoordinateFrame('Name', leftMidFoot.Reference.Name,...
                            'Reference', leftMidFoot.Reference,...
                            'Offset',[0,0,0],...
                            'R',[0,0,0]);
    offset = leftMidFoot.Offset;
    item2 = frost.Animator.Cylinder(robot_disp.axs, robot, frame, offset, name, options{:});
    robot_disp.addItem(item2);
    
    %%
    set(robot_disp.axs,'XLim',[-2,2]);
    view(robot_disp.axs,[0,0]);
    
    %     item = robot_disp.items('EndEff');
    %     item.radius = 0.01;
    %     item = robot_disp.items('Joint_joint1');
    %     item.radius = 0.015;
    %     item = robot_disp.items('Joint_joint2');
    %     item.radius = 0.015;
    %     item = robot_disp.items('Link_link1_to_joint2');
    %     item.radius = 0.01;
    robot_disp.update(zeros(robot.numState,1));
end