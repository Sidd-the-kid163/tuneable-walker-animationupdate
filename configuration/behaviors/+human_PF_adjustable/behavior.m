classdef behavior < handle
% Description: 
%    - model: AMBER_PF 
%    - state-based walking
% _________________________________________________________________________

    properties
        name;           % Name of this behavior
        isSymmetric;    % Flag true = symmetric behavior
        robotModel;     % The robot model associated
        vertices;       % Continuous domains
        edges;          % Discrete domains
        hybridSystem;   % Hybrid system for this behavior
        constraints;    % Optimization constraints
    end
    
    methods
        function obj = init(obj, isSymmetric, delay_coriolis)
            omit_coriolis = true; %Testing

            % Assign name
            tmp = split(mfilename('class'), '.');
            obj.name = tmp{1};
            obj.isSymmetric = isSymmetric;
            
            % Load the namespace domains/edges
            import([obj.name, '.Vertices.*']); 
                        
            %%% Load in the model
            urdf = strcat('urdf/human_PF_adjustable.urdf');
            % urdf = ros_resolve_local_url(strcat('package://amber_opt/robots/',robotName,'.urdf'));
            obj.robotModel = feval('amber3_PF_adjustable', urdf);
            obj.robotModel.configureDynamics('DelayCoriolisSet', delay_coriolis,'OmitCoriolisSet', omit_coriolis);
            
            %%% when some joint is actuated by springs
            % if strcmp(spring_conf, 'spring')
            %     obj.robotModel.appendDriftVector(obj.robotModel.fs_fun);
            % end
                                              
            %%% Load the controller
            controller  = IOFeedback('IO'); 
            
            %%% initialize the hybrid system
            obj.hybridSystem = HybridSystem(obj.name);
            
            %% define hybrid systems
            
            if isSymmetric
                % --------------------------------------------------
                %  vertex1: RightStance                            |
                %  ->edge1: LeftImpactRelabel                      |
                % --------------------------------------------------
                obj.vertices.v1  = vertex(obj.robotModel);
                obj.edges.LeftImpactRelabel     = Edges.Impact(obj.vertices.v1, 'Left', true);
                
                obj.hybridSystem = addVertex(obj.hybridSystem , 'v1', ...
                                             'Domain', obj.vertices.v1, ...
                                             'Control', controller);
                
                obj.hybridSystem = addEdge(obj.hybridSystem , 'v1', 'v1');
                obj.hybridSystem = setEdgeProperties(obj.hybridSystem , ...
                                                     'v1', 'v1', ...
                                                     'Guard', obj.edges.LeftImpactRelabel);
                
            else
                error('Left and right are the same for 2D robot, symmetry is sufficient.');
                % -------------------------------------------------
                %  vertex1: RightStance                             |
                %  -> edge1: LeftImpact                             |
                %  vertex2: LeftStance                              |
                %  -> edge2: RightImpact                            |
                % --------------------------------------------------
                obj.vertices.r_stance = RightSS(obj.robotModel);
                obj.vertices.l_stance = LeftSS(obj.robotModel);
                obj.edges.e1 = Edges.Impact(obj.vertices.l_stance, 'Left');
                obj.edges.e2 = Edges.Impact(obj.vertices.r_stance, 'Right');
                
                obj.hybridSystem = addVertex(obj.hybridSystem, 'RightSS', ...
                                         'Domain', obj.vertices.r_stance, ...
                                         'Control', controller);
                obj.hybridSystem = addVertex(obj.hybridSystem, 'LeftSS', ...
                                         'Domain', obj.vertices.l_stance, ...
                                         'Control', controller);
                
                srcs = {'RightSS' 'LeftSS'};
                tars = {'LeftSS'  'RightSS'};
                obj.hybridSystem = addEdge(obj.hybridSystem, srcs, tars);
                obj.hybridSystem = setEdgeProperties(obj.hybridSystem, srcs, tars, ...
                                                     'Guard', {obj.edges.e1, ...
                                                               obj.edges.e2});
                
            end
            
        end
    end
    
end
