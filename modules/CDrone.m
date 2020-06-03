classdef CDrone < handle
    % Common base class for drones
    
    properties
        
        %% id, indicator
        id_             = 1;                % index of the drone
        exp_id_         = 1;                % experiment drone id
        cfg_
        
        %% mode
        modeCoor_       = 0;                % coordination mode
        
        %% physical parameters
        size_           = [0.3; 0.3; 0.5];  % m, [a0, b0, c0]
        maxRoll_        = deg2rad(12);      % rad
        maxPitch_       = deg2rad(12);      % rad
        maxVz_          = 1.0;              % m/s
        maxYawRate_     = deg2rad(90);      % rad/s
        
        %% discrete dynamics parameter, mpc para
        dt_             = 0;                % time step, s
        N_              = 0;                % horizon length
        nQuad_          = 0;                % local number of quad 
        nDynObs_        = 0;                % local number of obs
        nvar_           = 0;                % number of elements in z variables
        npar_           = 0;                % number of local parameters feed to the MPC
        index_                              % local indexing
        
        %% real state
        pos_real_       = zeros(3, 1);      % position
        vel_real_       = zeros(3, 1);      % velocity
        euler_real_     = zeros(3, 1);      % euler angles
        
        %% measured state
        pos_measure_    = zeros(3, 1);      % measured position
        pos_measure_cov_= eye(3);           % measurement noise covariance
        euler_measure_  = zeros(3, 1);      % measured euler angles
        euler_measure_cov_ = eye(3);        % covariance
        
        %% estimated state
        pos_est_        = zeros(3, 1);
        pos_est_cov_    = eye(3);
        vel_est_        = zeros(3, 1);
        vel_est_cov_    = eye(3);
        euler_est_      = zeros(3, 1);
        euler_est_cov_  = eye(3);
        
        %% control input
        u_mpc_          = zeros(4, 1);      % phi_c, theta_c, vz_c, psi_rate_c
        u_body_         = zeros(4, 1);      % transformed
        
        %% information of moving obstacles and other quadrotors
        obs_size_       = [];               % size of moving obs
        obs_path_       = [];               % predicted path of moving obs
        quad_path_      = [];               % predicted path of quad
        obs_pathcov_    = [];
        quad_pathcov_   = [];
        
        %% predicted path
        pred_path_      = [];
        pred_pathcov_   = [];
        
        %% running para
        quad_goal_      = zeros(4, 1);      % [xg, yg, zg, psig]
        mpc_coll_       = zeros(2, 2);      % [lambda, buffer]
        mpc_weights_    = zeros(4, 2);      % [wp, input, coll, slack]
        
        %% mpc plan
        mpc_pAll_       = [];               % parameters feed to the MPC
        mpc_exitflag_                       % MPC solving exitflag
        mpc_info_                           % MPC solving information
        mpc_Xk_         = [];               % store initial conditions for MPC
        mpc_Zk_         = [];               % store computed current stage from MPC
        mpc_Zk2_        = [];               % store computed next stage from MPC
        mpc_ZPlan_      = [];               % store entire planned MPC plan
        mpc_Path_       = [];               % store MPC planned path
        mpc_PathCov_    = [];               % store MPC planned path with
                                            % position covariance
                                            
        %% ROS subscriber/publisher
        pose_measure_sub_                   % subscribe to mocap raw pose data
        state_est_sub_                      % subscribe to estimated state
        obs_path_sub_   = {};               % subscribe to predicted path of moving obstacles
        obs_size_sub_   = {};               % subscribe to size of moving obstacles
        cmd_vel_pub_                        % publish computed control input
        
    end
    
    
    methods
        
        %%  =======================================================================
        function obj = CDrone(quadID, quadExpID, cfg)
            % constructor
            
            global pr model index
            
            obj.id_     =   quadID;                 % specify the ID of the drone
            obj.exp_id_ =   quadExpID;              % real id in experiment
            obj.cfg_    =   cfg;                    % running configuration
            
            obj.size_   =   cfg.quad.size;
            
            obj.dt_     =   model.dt;
            obj.N_      =   model.N;
            
            obj.nQuad_  =   model.nQuad;
            obj.nDynObs_=   model.nDynObs;
            
            obj.nvar_   =   model.nvar;
            obj.npar_   =   model.npar;
            
            obj.index_  =   index;
            
            obj.maxRoll_    =   pr.input.maxRoll;
            obj.maxPitch_   =   pr.input.maxPitch;
            obj.maxVz_      =   pr.input.maxVz;
            obj.maxYawRate_ =   pr.input.maxYawRate;
            
            
            obj.quad_path_  =   zeros(3, obj.N_, obj.nQuad_);
            obj.quad_pathcov_=  zeros(6, obj.N_, obj.nQuad_);
            
            obj.obs_size_   =   zeros(3, obj.nDynObs_);
            obj.obs_path_   =   zeros(3, obj.N_, obj.nDynObs_);
            obj.obs_pathcov_=   zeros(6, obj.N_, obj.nDynObs_);
            for jObs = 1 : obj.nDynObs_
                for iStage = 1 : obj.N_
                    obj.obs_path_(3, iStage, jObs) = -2;
                end
            end
            
            obj.mpc_Path_   =   zeros(3, obj.N_);
            obj.mpc_PathCov_=   zeros(6, obj.N_);
            
            obj.pred_path_    = zeros(3, obj.N_);
            obj.pred_pathcov_ = zeros(6, obj.N_);
            
        end
        
        
        %%  =======================================================================
        function initializeMPC(obj, x_start, mpc_plan)
            % Initialize the intial conditions for the MPC solver with
            % x_start and mpc_plan, only used when necessary
            
            obj.mpc_Xk_    = x_start;
            obj.mpc_ZPlan_ = mpc_plan;
            
        end
        
        
        %%  =======================================================================
        function initializeROS(obj)
            % Initialize ROS publishers and subscribers for the quadrotor
            
            quadExpID = obj.exp_id_;
                       
            %% ROS subscribers
            % mocap raw data
            obj.pose_measure_sub_ = rossubscriber(...
                ['/Bebop', num2str(quadExpID), '/pose'], 'geometry_msgs/PoseStamped');
            % bebop 2 estimator data
            obj.state_est_sub_ = rossubscriber(...
                ['/Bebop', num2str(quadExpID), ...
                '/position_velocity_orientation_estimation'], ...
                'nav_msgs/Odometry');
            % predicted path of moving obstalces
            for jObs = 1 : obj.nDynObs_
                obj.obs_path_sub_{jObs} = rossubscriber(...
                    ['/Target', num2str(jObs), '/path_prediction'], ...
                    'std_msgs/Float64MultiArray');
            end
            % size of moving obstalces
            for jObs = 1 : obj.nDynObs_
                obj.obs_size_sub_{jObs} = rossubscriber(...
                    ['/Target', num2str(jObs), '/size'], ...
                    'std_msgs/Float64MultiArray');
            end
            
            %% ROS publisher
            % to publish mpc control input
            obj.cmd_vel_pub_ = rospublisher(...
                ['/bebop_auto_', num2str(quadExpID), '/cmd_vel'], ...
                'geometry_msgs/Twist', 'IsLatching', false);

        end
        
        
        %%  =======================================================================
        function getObservedSystemState(obj)
            % Get measured real-time position and attitude of the drone 
            % Not really used at the moment
            
            % geometry_msgs/PoseStamped
            msg = obj.pose_measure_sub_.LatestMessage; 
            
            % measured position [x y z]
            obj.pos_measure_ = [msg.Pose.Position.X;
                                msg.Pose.Position.Y;
                                msg.Pose.Position.Z];
            % measured quaternion [w x y z]
            q_measure = [ msg.Pose.Orientation.W;
                          msg.Pose.Orientation.X;
                          msg.Pose.Orientation.Y;
                          msg.Pose.Orientation.Z];
            % transform to euler angles (RPY, roll, pitch, yaw)
            obj.euler_measure_ = quat2eul(q_measure', 'XYZ')';
            
        end
        
        
        %%  =======================================================================
        function getEstimatedSystemState(obj)
            % Get estimated state of the drone
            
            switch obj.cfg_.modeSim
                case 1                  % in simple simulation mode
                    % set estimated state the same as real one
                    obj.pos_est_    =   obj.pos_real_;
                    obj.vel_est_    =   obj.vel_real_;
                    obj.euler_est_  =   obj.euler_real_;
                    
                    % add noise if necessary
                    if obj.cfg_.addQuadStateNoise == 1
                        dpos = zeros(3, 1);
                        dvel = zeros(3, 1);
                        deuler = zeros(3, 1);
                        for i = 1 : 3
                            dpos(i) = random('Normal', 0, ...
                                sqrt(obj.cfg_.quad.noise.pos(i,i)));
                            dvel(i) = random('Normal', 0, ...
                                sqrt(obj.cfg_.quad.noise.vel(i,i)));
                            deuler(i) = random('Normal', 0, ...
                                sqrt(obj.cfg_.quad.noise.euler(i,i)));
                        end
                        obj.pos_est_    =   obj.pos_est_ + dpos;
                        obj.vel_est_    =   obj.vel_est_ + dvel;
                        obj.euler_est_  =   obj.euler_est_ + deuler;
                        obj.pos_est_cov_    =   obj.cfg_.quad.noise.pos;
                        obj.vel_est_cov_    =   obj.cfg_.quad.noise.vel;
                        obj.euler_est_cov_  =   obj.cfg_.quad.noise.euler;
                    end
                    
                otherwise               % in experiment
                    msg = obj.state_est_sub_.LatestMessage;     % nav_msgs/Odometry

                    if ~isempty(msg)        % read only if the msg is not empty
                        % read position [x y z] and covariance from
                        % estimator
                        obj.pos_est_ = [ msg.Pose.Pose.Position.X;
                                         msg.Pose.Pose.Position.Y;
                                         msg.Pose.Pose.Position.Z];
                        obj.pos_est_cov_(1,1) = msg.Pose.Covariance(1);
                        obj.pos_est_cov_(1,2) = msg.Pose.Covariance(2);
                        obj.pos_est_cov_(1,3) = msg.Pose.Covariance(3);
                        obj.pos_est_cov_(2,1) = obj.pos_est_cov_(1,2);
                        obj.pos_est_cov_(2,2) = msg.Pose.Covariance(8);
                        obj.pos_est_cov_(2,3) = msg.Pose.Covariance(9);
                        obj.pos_est_cov_(3,1) = obj.pos_est_cov_(1,3);
                        obj.pos_est_cov_(3,2) = obj.pos_est_cov_(2,3);
                        obj.pos_est_cov_(3,3) = msg.Pose.Covariance(15);
                        % estimated velocity [vx vy vz] and covariance
                        obj.vel_est_ = [ msg.Twist.Twist.Linear.X;
                                         msg.Twist.Twist.Linear.Y;
                                         msg.Twist.Twist.Linear.Z];
                        obj.vel_est_cov_(1,1) = msg.Twist.Covariance(1);
                        obj.vel_est_cov_(1,2) = msg.Twist.Covariance(2);
                        obj.vel_est_cov_(1,3) = msg.Twist.Covariance(3);
                        obj.vel_est_cov_(2,1) = obj.vel_est_cov_(1,2);
                        obj.vel_est_cov_(2,2) = msg.Twist.Covariance(8);
                        obj.vel_est_cov_(2,3) = msg.Twist.Covariance(9);
                        obj.vel_est_cov_(3,1) = obj.vel_est_cov_(1,3);
                        obj.vel_est_cov_(3,2) = obj.vel_est_cov_(2,3);
                        obj.vel_est_cov_(3,3) = msg.Twist.Covariance(15);
                        % estimated quaternion [w x y z]
                        q_estimated = [ msg.Pose.Pose.Orientation.W;
                                        msg.Pose.Pose.Orientation.X;
                                        msg.Pose.Pose.Orientation.Y;
                                        msg.Pose.Pose.Orientation.Z];
                        % transform to euler angles (RPY, roll, pitch, yaw)
                        obj.euler_est_ = quat2eul(q_estimated', 'XYZ')';

                        % post processing, replece NaN in cov with zero
                        obj.pos_est_cov_(isnan(obj.pos_est_cov_)) = 0;
                        obj.vel_est_cov_(isnan(obj.vel_est_cov_)) = 0;
                        
                        % also regard those as real value
                        obj.pos_real_   =   obj.pos_est_;
                        obj.vel_real_   =   obj.vel_est_;
                        obj.euler_real_ =   obj.euler_est_;
                        
                        % add noise if necessary
                        if obj.cfg_.addQuadStateNoise == 1
                            dpos = zeros(3, 1);
                            dvel = zeros(3, 1);
                            deuler = zeros(3, 1);
                            for i = 1 : 3
                                dpos(i) = random('Normal', 0, ...
                                    sqrt(obj.cfg_.quad.noise.pos(i,i)));
                                dvel(i) = random('Normal', 0, ...
                                    sqrt(obj.cfg_.quad.noise.vel(i,i)));
                                deuler(i) = random('Normal', 0, ...
                                    sqrt(obj.cfg_.quad.noise.euler(i,i)));
                            end
                            obj.pos_est_    =   obj.pos_est_ + dpos;
                            obj.vel_est_    =   obj.vel_est_ + dvel;
                            obj.euler_est_  =   obj.euler_est_ + deuler;
                            obj.pos_est_cov_    =   obj.cfg_.quad.noise.pos;
                            obj.vel_est_cov_    =   obj.cfg_.quad.noise.vel;
                            obj.euler_est_cov_  =   obj.cfg_.quad.noise.euler;
                        end
                        
                    else
                        warning('Quadrotor %i is not tracked! \n', obj.id_);
                    end
            end
            
        end
        
               
        %%  =======================================================================
        function getObsPredictedPath(obj)
            % Get predicted path of all moving obstacles
            % This function takes a long time
            
            for jObs = 1 : obj.nDynObs_
                msg = obj.obs_path_sub_{jObs}.LatestMessage;
                if ~isempty(msg)        % read only if the msg is not empty
                    obj.obs_path_(:, :, jObs) = reshape(msg.Data, 3, obj.N_);
                else
                    warning('Obstacle %i path is not obtained! \n', jObs);
                end
            end
            
        end
        
        
        %%  =======================================================================
        function getObsSize(obj)
            % Get predicted path of all moving obstacles
            % This function takes a long time
            
            for jObs = 1 : obj.nDynObs_
                msg = obj.obs_size_sub_{jObs}.LatestMessage;
                if ~isempty(msg)        % read only if the msg is not empty
                    obj.obs_size_(:, jObs) = reshape(msg.Data, 3, 1);
                else
                    warning('Obstacle %i size is not obtained! \n', jObs);
                end
            end
            
        end
        
        
        %%  =======================================================================
        function getObsPredictedPathCov(obj)
            % Get predicted path of all moving obstacles
            % This function takes a long time
            
            % path
            for jObs = 1 : obj.nDynObs_
                msg = obj.obs_path_sub_{jObs}.LatestMessage;
                if ~isempty(msg)        % read only if the msg is not empty
                    obj.obs_path_(:, :, jObs) = reshape(msg.Data, 3, obj.N_);
                else
                    warning('Obstacle %i path is not obtained! \n', jObs);
                end
            end
            
            % cov, using simulated first
            % TODO
            simNoisePos = obj.cfg_.obs.noise.pos; 
            for jObs = 1 : obj.nDynObs_
                for iStage = 1 : obj.N_
                    obj.obs_pathcov_(:, iStage, jObs) = ...
                        [simNoisePos(1, 1); simNoisePos(2, 2); simNoisePos(3, 3); ...
                         simNoisePos(1, 2); simNoisePos(2, 3); simNoisePos(1, 3)];
                end
            end
            
        end
        
        
        %%  =======================================================================
        function setOnlineParamters(obj)
            % Set the real-time parameter vector
            % pAll include parameters for all N stage
            
            %% prepare parameters
            envDim       = obj.cfg_.ws;
            startPos     = [obj.pos_est_; obj.euler_est_(3)];
            wayPoint     = obj.quad_goal_;
            egoSize      = obj.size_;
            weightStage  = obj.mpc_weights_(:, 1);
            weightN      = obj.mpc_weights_(:, 2);
            quadSize     = obj.cfg_.quad.size;
            obsSize      = obj.obs_size_;
            quadColl     = obj.mpc_coll_(1:2, 1);
            obsColl      = obj.mpc_coll_(1:2, 2);
            quadPath     = obj.quad_path_;
            obsPath      = obj.obs_path_;
            
            %% all stage parameters
            pStage = zeros(obj.npar_, 1);              % some stage
            obj.mpc_pAll_= repmat(pStage, [obj.N_, 1]);% all stage
            for iStage = 1 : obj.N_
                % general parameter
                pStage(obj.index_.p.envDim)   = envDim;   	% environment dimension
                pStage(obj.index_.p.startPos) = startPos; 	% start position with yaw
                pStage(obj.index_.p.wayPoint) = wayPoint;  	% waypoint
                pStage(obj.index_.p.size)     = egoSize;  	% size of the drone 
                pStage(obj.index_.p.weights)  = weightStage;% stage cost weights
                % obstacle information, including other quadrotors 
                % and moving obstacles, set other quad first
                idx = 1;
                for iQuad = 1 : obj.nQuad_
                    if iQuad == obj.id_
                        continue;
                    else
                        pStage(obj.index_.p.obsParam(obj.index_.p.obs.pos, idx)) = ...
                            quadPath(:, iStage, iQuad);
                        pStage(obj.index_.p.obsParam(obj.index_.p.obs.size, idx)) = ...
                            quadSize;
                        pStage(obj.index_.p.obsParam(obj.index_.p.obs.coll, idx)) = ...
                            quadColl;
                        idx = idx + 1;
                    end
                end
                for jObs = 1 : obj.nDynObs_
                    pStage(obj.index_.p.obsParam(obj.index_.p.obs.pos, idx)) = ...
                            obsPath(:, iStage, jObs);
                    pStage(obj.index_.p.obsParam(obj.index_.p.obs.size, idx)) = ...
                        obsSize(:, jObs);
                    pStage(obj.index_.p.obsParam(obj.index_.p.obs.coll, idx)) = ...
                        obsColl;
                    idx = idx + 1;
                end
                % change the last stage cost term weights
                if iStage == obj.N_
                    pStage(obj.index_.p.weights)  = weightN;
                end
                % insert into the all stage parameter
                obj.mpc_pAll_((1 + obj.npar_*(iStage-1)) : ...
                    (obj.npar_ + obj.npar_*(iStage-1))) = pStage;
            end
            
        end
        
        
        %%  =======================================================================
        function setOnlineParamtersCov(obj)
            % Set the real-time parameter vector
            % pAll include parameters for all N stage
            
            %% prepare parameters
            envDim       = obj.cfg_.ws;
            startPos     = [obj.pos_est_; obj.euler_est_(3)];
            wayPoint     = obj.quad_goal_;
            egoSize      = obj.size_;
            weightStage  = obj.mpc_weights_(:, 1);
            weightN      = obj.mpc_weights_(:, 2);
            quadSize     = obj.cfg_.quad.size;
            obsSize      = obj.cfg_.obs.size;
            quadColl     = obj.mpc_coll_(1:3, 1);
            obsColl      = obj.mpc_coll_(1:3, 2);
            quadPath     = obj.quad_path_;
            obsPath      = obj.obs_path_;
            quadPathCov  = obj.quad_pathcov_;
            obsPathCov   = obj.obs_pathcov_;
            
            %% all stage parameters
            pStage = zeros(obj.npar_, 1);              % some stage
            obj.mpc_pAll_= repmat(pStage, [obj.N_, 1]);% all stage
            for iStage = 1 : obj.N_
                % general parameter
                pStage(obj.index_.p.envDim)   = envDim;   	% environment dimension
                pStage(obj.index_.p.startPos) = startPos;  	% start position with yaw
                pStage(obj.index_.p.wayPoint) = wayPoint; 	% waypoint
                pStage(obj.index_.p.size)     = egoSize;  	% size of the drone 
                pStage(obj.index_.p.posCov)   = quadPathCov(:, iStage, obj.id_);
                pStage(obj.index_.p.weights)  = weightStage; % stage cost weights
                % obstacle information, including other quadrotors 
                % and moving obstacles, set other quad first
                idx = 1;
                for iQuad = 1 : obj.nQuad_
                    if iQuad == obj.id_
                        continue;
                    else
                        pStage(obj.index_.p.obsParam(obj.index_.p.obs.pos, idx)) = ...
                            quadPath(:, iStage, iQuad);
                        pStage(obj.index_.p.obsParam(obj.index_.p.obs.size, idx)) = ...
                            quadSize;
                        pStage(obj.index_.p.obsParam(obj.index_.p.obs.coll, idx)) = ...
                            quadColl;
                        pStage(obj.index_.p.obsParam(obj.index_.p.obs.posCov, idx)) = ...
                            quadPathCov(:, iStage, iQuad);
                        idx = idx + 1;
                    end
                end
                for jObs = 1 : obj.nDynObs_
                    pStage(obj.index_.p.obsParam(obj.index_.p.obs.pos, idx)) = ...
                            obsPath(:, iStage, jObs);
                    pStage(obj.index_.p.obsParam(obj.index_.p.obs.size, idx)) = ...
                        obsSize;
                    pStage(obj.index_.p.obsParam(obj.index_.p.obs.coll, idx)) = ...
                        obsColl;
                    pStage(obj.index_.p.obsParam(obj.index_.p.obs.posCov, idx)) = ...
                            obsPathCov(:, iStage, jObs);
                    idx = idx + 1;
                end
                % change the last stage cost term weights
                if iStage == obj.N_ 
                    pStage(obj.index_.p.weights)  = weightN;
                end
                % insert into the all stage parameter
                obj.mpc_pAll_((1 + obj.npar_*(iStage-1)) : ...
                    (obj.npar_ + obj.npar_*(iStage-1))) = pStage;
            end
            
        end

        
        %%  =======================================================================
        function solveMPC(obj)
           % Calling the solver to solve the mpc for collision avoidance

           problem.all_parameters = obj.mpc_pAll_;
           
           % set initial conditions
           obj.mpc_Xk_ = [obj.pos_est_; obj.vel_est_; obj.euler_est_];
           problem.xinit = obj.mpc_Xk_;
           
           % prepare initial guess
           if obj.mpc_exitflag_ == 1    % last step mpc feasible
               x0_temp = reshape([obj.mpc_ZPlan_(:, 2:obj.N_), ...
                   obj.mpc_ZPlan_(:, obj.N_)], obj.N_*obj.nvar_, 1);
           else                         % last step mpc infeasible
               x0_temp_stage = zeros(obj.nvar_, 1);
               x0_temp_stage([obj.index_.z.pos, obj.index_.z.vel, obj.index_.z.euler]) = obj.mpc_Xk_;
               x0_temp = repmat(x0_temp_stage, obj.N_, 1);
           end           
           
           problem.x0 = x0_temp;
           
           % call the NLP solver
           func_name = strcat('FORCESNLPsolver_', obj.cfg_.application, '_', ...
               num2str(obj.nQuad_ + obj.nDynObs_ - 1), '_', ...
               num2str(obj.N_), '_', num2str(1000*obj.dt_));
           NLPSolver = str2func(func_name);
           [output, exitflag, info] = NLPSolver(problem);
           
           % store mpc sovling information
           obj.mpc_exitflag_ = exitflag;
           obj.mpc_info_    = info;
           
           % store output
           for iStage = 1 : obj.N_
               obj.mpc_ZPlan_(:, iStage) = output.(['x', sprintf('%02d', iStage)]);
               obj.mpc_Path_(:, iStage)  = obj.mpc_ZPlan_(obj.index_.z.pos, iStage);
           end
           obj.mpc_Zk_  = obj.mpc_ZPlan_(:, 1);
           obj.mpc_Zk2_ = obj.mpc_ZPlan_(:, 2);
           
           % check the exitflag and get optimal control input
           if exitflag == 0                 % solver reaching maximum iterations
               warning('MPC: Max iterations reached!');
           elseif exitflag == -4
               warning('MPC: Wrong number of inequalities input to solver!');
           elseif exitflag == -5
               warning('MPC: Error occured during matrix factorization!');
           elseif exitflag == -6
               warning('MPC: NaN or INF occured during functions evaluations!');
           elseif exitflag == -7
               warning('MPC: Infeasible! The solver could not proceed!');
           elseif exitflag == -10
               warning('MPC: NaN or INF occured during evaluation of functions and derivatives!');
           elseif exitflag == -11
               warning('MPC: Invalid values in problem parameters!');
           elseif exitflag == -100
               warning('MPC: License error!');
           end
           
           if exitflag == 1
               % if mpc solved successfully
               obj.u_mpc_ = obj.mpc_Zk_(obj.index_.z.inputs);
           else
               % if infeasible
%                obj.u_mpc_ = zeros(4, 1);
                obj.u_mpc_ = -0.0 * obj.u_mpc_;
           end
           
           % transform u, check the usind dynamics model before doing this!
           yaw = obj.euler_est_(3);
           obj.u_body_    = obj.u_mpc_;
           obj.u_body_(1) = obj.u_mpc_(2)*sin(yaw) + obj.u_mpc_(1)*cos(yaw);
           obj.u_body_(2) = obj.u_mpc_(2)*cos(yaw) + obj.u_mpc_(1)*sin(yaw);
           
        end
        
        
        %%  =======================================================================
        function step(obj)
            % Send and execute the control command
            
            switch obj.cfg_.modeSim
                case 1                      % in simple simulation mode
                    % simulate one step in simple simulation mode
                    % current state and control
                    xNow = [obj.pos_real_; obj.vel_real_; obj.euler_real_];
                    u    = obj.u_mpc_;      % use u_mpc_ in simulation

                    % integrate one step
                    xNext = RK2( xNow, u, @bebop_dynamics, obj.dt_, [], 1);

                    % update the implicit real state
                    obj.pos_real_   = xNext(obj.index_.x.pos);
                    obj.vel_real_   = xNext(obj.index_.x.vel);
                    obj.euler_real_ = xNext(obj.index_.x.euler);
                    
                otherwise
                    % publish control command sent to the bebop 2
                    cmd_vel_msg = rosmessage('geometry_msgs/Twist');
                    % pitch to move along x direction
                    cmd_vel_msg.Linear.X =  obj.u_body_(2) / obj.maxPitch_;
                    % roll to move along y direction, pay attention to the 'negative'!
                    cmd_vel_msg.Linear.Y = -obj.u_body_(1) / obj.maxRoll_;	                                                      
                    cmd_vel_msg.Linear.Z = obj.u_body_(3) / obj.maxVz_;
%                     cmd_vel_msg.Angular.Z= obj.u_body_(4) / obj.maxYawRate_;
                    cmd_vel_msg.Angular.Z = 0;
                    obj.cmd_vel_pub_.send(cmd_vel_msg);
                    
            end
            
        end
        
        
        %%  =======================================================================
        function propagateStateCov(obj)
            % Propagate uncertainty covariance along the path  

            % model parameters
            g           =   9.81;
            kD_x        =   0.25;
            kD_y        =   0.33;
            tau_vz      =   0.3367;
            tau_phi     =   0.2368;
            tau_theta   =   0.2318;
            
            % current state uncertainty covariance
            S0 = zeros(9, 9);           
            S0(1:3, 1:3) = obj.pos_est_cov_;
            S0(4:6, 4:6) = obj.vel_est_cov_;
            S0(7:9, 7:9) = obj.euler_est_cov_;
            
            % uncertainty propagation
            S_Now = S0;
            for iStage = 1 : obj.N_
                % store path cov
                obj.mpc_PathCov_(:, iStage) = [S_Now(1,1); S_Now(2,2); ...
                    S_Now(3,3); S_Now(1,2); S_Now(2,3); S_Now(1,3)];
                % state transition matrix
                F_Now = zeros(9, 9);
                phi_Now   = obj.mpc_ZPlan_(obj.index_.z.euler(1), iStage);
                theta_Now = obj.mpc_ZPlan_(obj.index_.z.euler(2), iStage);
                F_Now(1, 1) = 1;
                F_Now(1, 4) = obj.dt_;
                F_Now(2, 2) = 1;
                F_Now(2, 5) = obj.dt_;
                F_Now(3, 3) = 1;
                F_Now(3, 6) = obj.dt_;
                F_Now(4, 4) = 1 - obj.dt_*kD_x;
                F_Now(4, 8) = g*obj.dt_ / (cos(theta_Now))^2;
                F_Now(5, 5) = 1 - obj.dt_*kD_y;
                F_Now(5, 7) = -g*obj.dt_ / (cos(phi_Now))^2;
                F_Now(6, 6) = 1 - obj.dt_/tau_vz;
                F_Now(7, 7) = 1 - obj.dt_/tau_phi;
                F_Now(8, 8) = 1 - obj.dt_/tau_theta;
                F_Now(9, 9) = 1;
                % uncertainty propagation
                S_Next = F_Now * S_Now * F_Now';
%                 S_Next = S_Now;     % for debugging
                % set next to now
                S_Now = S_Next;
            end 
            
        end
        
        
        %%  =======================================================================
        function predictPathConstantV(obj)
            % Predict quad path based on constant velocity assumption
            
            obj.pred_path_(:, 1)    = obj.pos_est_;
            obj.pred_pathcov_(:, 1) = [ obj.pos_est_cov_(1,1); ...
                                        obj.pos_est_cov_(2,2); ...
                                        obj.pos_est_cov_(3,3); ...
                                        obj.pos_est_cov_(1,2); ...
                                        obj.pos_est_cov_(2,3); ...
                                        obj.pos_est_cov_(1,3)];
            
            xpred = [obj.pos_est_; obj.vel_est_];
            Ppred = [obj.pos_est_cov_, zeros(3, 3); ...
                     zeros(3, 3), obj.vel_est_cov_];
            F = [1, 0, 0, obj.dt_, 0, 0; ...
                 0, 1, 0, 0, obj.dt_, 0; ...
                 0, 0, 1, 0, 0, obj.dt_; ...
                 0, 0, 0, 1, 0, 0; ...
                 0, 0, 0, 0, 1, 0; ...
                 0, 0, 0, 0, 0, 1];
                                    
            for iStage = 2 : obj.N_
                xpred = F * xpred;
                Ppred = F * Ppred * F';
                obj.pred_path_(:, iStage) = xpred(1:3);
                obj.pred_pathcov_(:, iStage) = [ Ppred(1,1); ...
                                                 Ppred(2,2); ...
                                                 Ppred(3,3); ...
                                                 Ppred(1,2); ...
                                                 Ppred(2,3); ...
                                                 Ppred(1,3)];
            end
            
        end
        

    end
    
end
