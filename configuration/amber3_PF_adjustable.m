classdef amber3_PF_adjustable < RobotLinks
    % Class: Amber3 with point foot

    properties
        
        ContactPoints
        
        feet
        
        lf = 0.2; % distance from pointFoot to ShinExtension

        fs_fun; % Spring forces
        
        % Center of mass position & velocity
        com_fun;
        comV_fun;
        
        %%% spring_conf (how to actuate springy joints)
        
        fric_coef;
        slope;
        
    end
    
    methods
        function obj = amber3_PF_adjustable(urdf)

            base = get_base_dofs('planar');
            
            limits = [base.Limit];% Limits for the base frame 
            
            [limits.lower]    = deal(-0.4, 0.5, -0.1);
            [limits.upper]    = deal( 1.0, 2.0,  0.1);
            [limits.velocity] = deal( 2,  0.5,   1.0);
            
            for i=1:3
                base(i).Limit = limits(i);
            end

            % Extract info from urdf
            [name, links, joints, transmissions] = ros_load_urdf(urdf); 
            
            % Remove Fixed Joints
            % fixed_joints = arrayfun(@(x)strcmp(x.Type,'fixed'),joints);
            % ref_joint_indices = find(fixed_joints);
            % joints(ref_joint_indices) = [];

            config.name = name;
            config.joints = joints;
            config.links = links;
            config.transmissions = transmissions;
           
            obj = obj@RobotLinks(config,base);
            x = obj.States.x;
            dx = obj.States.dx;
            obj.com_fun = obj.getComPosition;
            obj.comV_fun = jacobian(obj.getComPosition, x) * dx;
            
            %% define contact frames

            lf = obj.lf;
            
            % Right foot
            r_foot_frame = obj.Joints(getJointIndices(obj, 'RightShinExtension'));             
            obj.feet.RightMidFoot = CoordinateFrame(...
                                            'Name','RightMidFoot',...
                                            'Reference',r_foot_frame,...
                                            'Offset',[0, 0, -lf],...
                                            'R',[0,0,0]);
            
            % Left foot
            l_foot_frame = obj.Joints(getJointIndices(obj, 'LeftShinExtension'));   
            obj.feet.LeftMidFoot = CoordinateFrame(...
                                            'Name','LeftMidFoot',...
                                            'Reference',l_foot_frame,...
                                            'Offset',[0, 0, -lf],...
                                            'R',[0,0,0]);

            %% friction cone parameters
            obj.fric_coef.mu = 0.8;     %-translational friction (set to 0.09 for slippery walking)
            % obj.fric_coef.gamma = 25; %-torsional friction
            
            obj.slope = 0; %20*pi/180; % slope of the ground.
            
        end
    end
end