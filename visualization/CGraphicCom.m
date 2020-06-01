classdef CGraphicCom < handle
    % CGraphic Handles graphics related communication between controller and GUI
    
    properties
        
        %% Server or client
        isServer_           =   0;          % 0 - client, 1 - server
        
        %% Running configuration
        cfg_
        
        %% Problem definition
        nQuad_              =   0;
        nDynObs_            =   0;
        N_                  =   0;
        
        %% Quad data
        % size
        quad_size_          =   [];         % [a, b, c], m
        % state
        quad_state_pub_                     % ros publisher
        quad_state_sub_                     % ros subscriber
        quad_state_         =   [];         % state of all quads
        % goal
        quad_goal_pub_
        quad_goal_sub_
        quad_goal_          =   [];
        % input
        quad_input_pub_
        quad_input_sub_
        quad_input_
        % slack
        quad_slack_sub_
        quad_slack_pub_
        quad_slack_         =   [];
        % mpc path
        quad_path_pub_
        quad_path_sub_
        quad_path_          =   [];
        % mpc with covariance
        quad_pathcov_pub_
        quad_pathcov_sub_
        quad_pathcov_       =   [];
        
        %% MPC para
        % coll, quad and obs
        mpc_coll_pub_
        mpc_coll_sub_
        mpc_coll_           =   [];
        % weights, stage and N
        mpc_weights_pub_
        mpc_weights_sub_
        mpc_weights_        =   [];        
        
        %% Obs data
        % size
        obs_size_           =   [];         % [a, b, c], m
        obs_size_pub_
        obs_size_sub_
        % state
        obs_state_pub_
        obs_state_sub_
        obs_state_          =   [];
        % predicted path
        obs_path_pub_
        obs_path_sub_
        obs_path_           =   [];
        % path with cov
        obs_pathcov_pub_
        obs_pathcov_sub_
        obs_pathcov_        =   [];
        
    end
    
    
    methods
        
        %%  =======================================================================
        function obj = CGraphicCom(isServer, cfg, nQuad, nDynObs, N)
            
            obj.isServer_       =   isServer;
            obj.cfg_            =   cfg;
            
            obj.nQuad_          =   nQuad;
            obj.nDynObs_        =   nDynObs;
            obj.N_              =   N;
            
            obj.quad_size_      =   zeros(3, nQuad);
            obj.quad_state_     =   zeros(9, nQuad);
            obj.quad_goal_      =   zeros(4, nQuad);
            obj.quad_input_     =   zeros(4, nQuad);
            obj.quad_slack_     =   zeros(2, nQuad);
            obj.quad_path_      =   zeros(3, N, nQuad);
            obj.quad_pathcov_   =   zeros(6, N, nQuad);
            simNoisePos = obj.cfg_.quad.noise.pos;
            for iQuad = 1 : obj.nQuad_
                for iStage = 1 : obj.N_
                    obj.quad_pathcov_(:, iStage, iQuad) = ...
                        [simNoisePos(1, 1); simNoisePos(2, 2); simNoisePos(3, 3); ...
                         simNoisePos(1, 2); simNoisePos(2, 3); simNoisePos(1, 3)];
                end
            end
                        
            obj.obs_size_       =   zeros(3, nDynObs);
            obj.obs_state_      =   zeros(6, nDynObs);
            obj.obs_path_       =   zeros(3, N, nDynObs);
            obj.obs_pathcov_    =   zeros(6, N, nDynObs);
            simNoisePos = obj.cfg_.obs.noise.pos;
            for jObs = 1 : obj.nQuad_
                for iStage = 1 : obj.N_
                    obj.obs_pathcov_(:, iStage, jObs) = ...
                        [simNoisePos(1, 1); simNoisePos(2, 2); simNoisePos(3, 3); ...
                         simNoisePos(1, 2); simNoisePos(2, 3); simNoisePos(1, 3)];
                end
            end
            
            obj.mpc_coll_       =   zeros(3, 2);
            obj.mpc_weights_    =   zeros(4, 2);
            
        end
        
        
        %%  =======================================================================
        function initializeROS(obj)
            % Initialize ROS
            
            %% Client side, subsribe
            if ~obj.isServer_
                % quad
                obj.quad_state_sub_ = rossubscriber('/VisualCom/QuadState', ...
                    'std_msgs/Float64MultiArray'); 
                obj.quad_path_sub_  = rossubscriber('/VisualCom/QuadMPCPath', ...
                    'std_msgs/Float64MultiArray');
                obj.quad_pathcov_sub_ = rossubscriber('/VisualCom/QuadMPCPathCov', ...
                    'std_msgs/Float64MultiArray');
                obj.quad_input_sub_ = rossubscriber('/VisualCom/QuadInput', ...
                    'std_msgs/Float64MultiArray'); 
                obj.quad_slack_sub_ = rossubscriber('/VisualCom/QuadSlack', ...
                    'std_msgs/Float64MultiArray'); 
                % para
                if obj.cfg_.setParaGui == 1         % publish, set para from gui
                    % quad goal
                    obj.quad_goal_pub_ = rospublisher('/VisualCom/QuadGoal', ...
                        'std_msgs/Float64MultiArray', 'IsLatching', false);
                    % mpc coll
                    obj.mpc_coll_pub_ = rospublisher('/VisualCom/MPCColl', ...
                        'std_msgs/Float64MultiArray', 'IsLatching', false);
                    % mpc weights
                    obj.mpc_weights_pub_ = rospublisher('/VisualCom/MPCWeights', ...
                        'std_msgs/Float64MultiArray', 'IsLatching', false);
                else                                % subsribe
                    % quad goal
                    obj.quad_goal_sub_ = rossubscriber('/VisualCom/QuadGoal', ...
                        'std_msgs/Float64MultiArray');
                    % mpc coll
                    obj.mpc_coll_sub_ = rossubscriber('/VisualCom/MPCColl', ...
                        'std_msgs/Float64MultiArray');
                    % mpc weights
                    obj.mpc_weights_sub_ = rossubscriber('/VisualCom/MPCWeights', ...
                        'std_msgs/Float64MultiArray');
                end
                % obs
                obj.obs_size_sub_ = rossubscriber('/VisualCom/ObsSize', ...
                    'std_msgs/Float64MultiArray');
                obj.obs_state_sub_ = rossubscriber('/VisualCom/ObsState', ...
                    'std_msgs/Float64MultiArray');
                obj.obs_path_sub_  = rossubscriber('/VisualCom/ObsPath', ...
                    'std_msgs/Float64MultiArray');
                obj.obs_pathcov_sub_ = rossubscriber('/VisualCom/ObsPathCov', ...
                    'std_msgs/Float64MultiArray');
            %% Server side, publish    
            else
                % quad
                obj.quad_state_pub_ = rospublisher('/VisualCom/QuadState', ...
                    'std_msgs/Float64MultiArray', 'IsLatching', false);
                obj.quad_path_pub_  = rospublisher('/VisualCom/QuadMPCPath', ...
                    'std_msgs/Float64MultiArray', 'IsLatching', false);
                obj.quad_pathcov_pub_ = rospublisher('/VisualCom/QuadMPCPathCov', ...
                    'std_msgs/Float64MultiArray', 'IsLatching', false);
                obj.quad_input_pub_ = rospublisher('/VisualCom/QuadInput', ...
                    'std_msgs/Float64MultiArray', 'IsLatching', false);
                obj.quad_slack_pub_ = rospublisher('/VisualCom/QuadSlack', ...
                    'std_msgs/Float64MultiArray', 'IsLatching', false);
                % para
                if obj.cfg_.setParaGui == 1         % subscribe
                    % quad goal
                    obj.quad_goal_sub_ = rossubscriber('/VisualCom/QuadGoal', ...
                        'std_msgs/Float64MultiArray');
                    % mpc coll
                    obj.mpc_coll_sub_ = rossubscriber('/VisualCom/MPCColl', ...
                        'std_msgs/Float64MultiArray');
                    % mpc weights
                    obj.mpc_weights_sub_ = rossubscriber('/VisualCom/MPCWeights', ...
                        'std_msgs/Float64MultiArray');
                else                                % publish
                    % quad goal
                    obj.quad_goal_pub_ = rospublisher('/VisualCom/QuadGoal', ...
                        'std_msgs/Float64MultiArray', 'IsLatching', false);
                    % mpc coll
                    obj.mpc_coll_pub_ = rospublisher('/VisualCom/MPCColl', ...
                        'std_msgs/Float64MultiArray', 'IsLatching', false);
                    % mpc weights
                    obj.mpc_weights_pub_ = rospublisher('/VisualCom/MPCWeights', ...
                        'std_msgs/Float64MultiArray', 'IsLatching', false);
                end
                % obs
                obj.obs_size_pub_ = rospublisher('/VisualCom/ObsSize', ...
                    'std_msgs/Float64MultiArray', 'IsLatching', false);
                obj.obs_state_pub_ = rospublisher('/VisualCom/ObsState', ...
                    'std_msgs/Float64MultiArray', 'IsLatching', false);
                obj.obs_path_pub_  = rospublisher('/VisualCom/ObsPath', ...
                    'std_msgs/Float64MultiArray', 'IsLatching', false);
                obj.obs_pathcov_pub_ = rospublisher('/VisualCom/ObsPathCov', ...
                    'std_msgs/Float64MultiArray', 'IsLatching', false);
            end
            
        end
        
        
        %%  =======================================================================
        function getQuadState(obj)
            
            msg = obj.quad_state_sub_.LatestMessage;
            if ~isempty(msg)        % read only if the msg is not empty
                obj.quad_state_ = reshape(msg.Data, 9, obj.nQuad_);
            end
            
        end
        
        
        %%  =======================================================================
        function sendQuadState(obj)
            
            msg = rosmessage('std_msgs/Float64MultiArray');
            msg.Data = reshape(obj.quad_state_, [], 1);
            obj.quad_state_pub_.send(msg);
            
        end
        
        
        %%  =======================================================================
        function getQuadPath(obj)
            
            msg = obj.quad_path_sub_.LatestMessage;
            if ~isempty(msg)        % read only if the msg is not empty
                obj.quad_path_ = reshape(msg.Data, 3, obj.N_, obj.nQuad_);
            end
            
        end
        
        
        %%  =======================================================================
        function sendQuadPath(obj)
            
            msg = rosmessage('std_msgs/Float64MultiArray');
            msg.Data = reshape(obj.quad_path_, [], 1);
            obj.quad_path_pub_.send(msg);
            
        end
        
        
        %%  =======================================================================
        function getQuadPathCov(obj)
            
            msg = obj.quad_pathcov_sub_.LatestMessage;
            if ~isempty(msg)        % read only if the msg is not empty
                obj.quad_pathcov_ = reshape(msg.Data, 6, obj.N_, obj.nQuad_);
            end
            
        end
        
        
        %%  =======================================================================
        function sendQuadPathCov(obj)
            
            msg = rosmessage('std_msgs/Float64MultiArray');
            msg.Data = reshape(obj.quad_pathcov_, [], 1);
            obj.quad_pathcov_pub_.send(msg);
            
        end
        
        
        %%  =======================================================================
        function getQuadGoal(obj)
            
            msg = obj.quad_goal_sub_.LatestMessage;
            if ~isempty(msg)        % read only if the msg is not empty
                obj.quad_goal_ = reshape(msg.Data, 4, obj.nQuad_);
            end
            
        end
        
        
        %%  =======================================================================
        function sendQuadGoal(obj)
            
            msg = rosmessage('std_msgs/Float64MultiArray');
            msg.Data = reshape(obj.quad_goal_, [], 1);
            obj.quad_goal_pub_.send(msg);
            
        end
        
        
        %%  =======================================================================
        function getQuadInput(obj)
            
            msg = obj.quad_input_sub_.LatestMessage;
            if ~isempty(msg)        % read only if the msg is not empty
                obj.quad_input_ = reshape(msg.Data, 4, obj.nQuad_);
            end
            
        end
        
        
        %%  =======================================================================
        function sendQuadInput(obj)
            
            msg = rosmessage('std_msgs/Float64MultiArray');
            msg.Data = reshape(obj.quad_input_, [], 1);
            obj.quad_input_pub_.send(msg);
            
        end
        
        
        %%  =======================================================================
        function getQuadSlack(obj)
            
            msg = obj.quad_slack_sub_.LatestMessage;
            if ~isempty(msg)        % read only if the msg is not empty
                obj.quad_slack_ = reshape(msg.Data, 2, obj.nQuad_);
            end
            
        end
        
        
        %%  =======================================================================
        function sendQuadSlack(obj)
            
            msg = rosmessage('std_msgs/Float64MultiArray');
            msg.Data = reshape(obj.quad_slack_, [], 1);
            obj.quad_slack_pub_.send(msg);
            
        end
        
        
        %%  =======================================================================
        function getMPCColl(obj)
            
            msg = obj.mpc_coll_sub_.LatestMessage;
            if ~isempty(msg)        % read only if the msg is not empty
                obj.mpc_coll_ = reshape(msg.Data, 3, 2);
            end
            
        end
        
        
        %%  =======================================================================
        function sendMPCColl(obj)
            
            msg = rosmessage('std_msgs/Float64MultiArray');
            msg.Data = reshape(obj.mpc_coll_, [], 1);
            obj.mpc_coll_pub_.send(msg);
            
        end
        
        
        %%  =======================================================================
        function getMPCWeights(obj)
            
            msg = obj.mpc_weights_sub_.LatestMessage;
            if ~isempty(msg)        % read only if the msg is not empty
                obj.mpc_weights_ = reshape(msg.Data, 4, 2);
            end
            
        end
        
        
        %%  =======================================================================
        function sendMPCWeights(obj)
            
            msg = rosmessage('std_msgs/Float64MultiArray');
            msg.Data = reshape(obj.mpc_weights_, [], 1);
            obj.mpc_weights_pub_.send(msg);
            
        end
        
        
        %%  =======================================================================
        function getObsSize(obj)
            
            msg = obj.obs_size_sub_.LatestMessage;
            if ~isempty(msg)        % read only if the msg is not empty
                obj.obs_size_ = reshape(msg.Data, 3, obj.nDynObs_);
            end

        end
        
        
        %%  =======================================================================
        function sendObsSize(obj)
            
            msg = rosmessage('std_msgs/Float64MultiArray');
            msg.Data = reshape(obj.obs_size_, [], 1);
            obj.obs_size_pub_.send(msg);
            
        end
        
        
        %%  =======================================================================
        function getObsState(obj)
            
            msg = obj.obs_state_sub_.LatestMessage;
            if ~isempty(msg)        % read only if the msg is not empty
                obj.obs_state_ = reshape(msg.Data, 6, obj.nDynObs_);
            end

        end
        
        
        %%  =======================================================================
        function sendObsState(obj)
            
            msg = rosmessage('std_msgs/Float64MultiArray');
            msg.Data = reshape(obj.obs_state_, [], 1);
            obj.obs_state_pub_.send(msg);
            
        end
        
        
        %%  =======================================================================
        function getObsPath(obj)
            
            msg = obj.obs_path_sub_.LatestMessage;
            if ~isempty(msg)        % read only if the msg is not empty
                obj.obs_path_ = reshape(msg.Data, 3, obj.N_, obj.nDynObs_);
            end
            
        end
        
        
        %%  =======================================================================
        function sendObsPath(obj)
            
            msg = rosmessage('std_msgs/Float64MultiArray');
            msg.Data = reshape(obj.obs_path_, [], 1);
            obj.obs_path_pub_.send(msg);
            
        end
        
        
        %%  =======================================================================
        function getObsPathCov(obj)
            
            msg = obj.obs_pathcov_sub_.LatestMessage;
            if ~isempty(msg)        % read only if the msg is not empty
                obj.obs_pathcov_ = reshape(msg.Data, 6, obj.N_, obj.nDynObs_);
            end
            
        end
        
        
        %%  =======================================================================
        function sendObsPathCov(obj)
            
            msg = rosmessage('std_msgs/Float64MultiArray');
            msg.Data = reshape(obj.obs_pathcov_, [], 1);
            obj.obs_pathcov_pub_.send(msg);
            
        end
        
    end
    
    
end
