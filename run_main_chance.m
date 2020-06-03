% Script for simulating multi-quad collision avoidance using mpc

%% Clean workspace
clear
close all
clearvars
clearvars -global
clc


%% Initialization
initialize;


%% Generate solver
if getNewSolver 
    mpc_generator_chance;
end


%% Create quad objects and intialize ROS
for iQuad = 1 : model.nQuad
    Quadrotor(iQuad) = CDrone(iQuad, quad_ID(iQuad), cfg);
    Quadrotor(iQuad).initializeROS();
end


%% Create moving obstacle objects in simulation mode
if cfg.modeSim == 1
    for jObs = 1 : model.nDynObs
        DynObs(jObs) = CDynObs(jObs, cfg);
        DynObs(jObs).initializeROS();
        DynObs(jObs).initializeState(zeros(3,1), zeros(3,1), ...
                                     cfg.obs.noise.pos, cfg.obs.noise.vel);
        DynObs(jObs).randomState();
    end
end


%% Communication with gui
% quad
com_quad_state = zeros(9, model.nQuad);             % state
com_quad_goal  = cfg.quad.goal;                     % goal
com_quad_input = zeros(4, model.nQuad);             % input
com_quad_slack = zeros(2, model.nQuad);             % slack
com_quad_path  = zeros(3, model.N, model.nQuad);    % planned path
com_quad_pathcov= zeros(6, model.N, model.nQuad);   % path cov, 6 independent
% path cov initialization
simNoisePos = cfg.quad.noise.pos;
for iQuad = 1 : model.nQuad
    for iStage = 1 : model.N
        com_quad_path(:, iStage, iQuad) = quadStartPos(1:3, iQuad);
        com_quad_pathcov(:, iStage, iQuad) = ...
            [simNoisePos(1, 1); simNoisePos(2, 2); simNoisePos(3, 3); ...
             simNoisePos(1, 2); simNoisePos(2, 3); simNoisePos(1, 3)];
    end
end
% para
com_mpc_coll    = [cfg.quad.coll, cfg.obs.coll];
com_mpc_coll(3, 1) = erfinv(1-2.0*com_mpc_coll(3, 1));
com_mpc_coll(3, 2) = erfinv(1-2.0*com_mpc_coll(3, 2));
com_mpc_weights = [cfg.weightStage, cfg.weightN];
% obs 
com_obs_state = zeros(6, model.nDynObs);
com_obs_path  = zeros(3, model.N, model.nDynObs);
com_obs_pathcov = zeros(6, model.N, model.nDynObs);


%% Quadrotor coordination info
coor_quad_path    = com_quad_path;
coor_quad_pathcov = com_quad_pathcov;


%% Initialization quad simulated initial state and mpc plan
for iQuad = 1 : model.nQuad
    % coordination mode
    Quadrotor(iQuad).modeCoor_        = cfg.modeCoor;
    % initial state
    Quadrotor(iQuad).pos_real_(1:3)   = quadStartPos(1:3, iQuad);
    Quadrotor(iQuad).vel_real_(1:3)   = quadStartVel(1:3, iQuad);
    Quadrotor(iQuad).euler_real_(1:3) = zeros(3, 1);
    Quadrotor(iQuad).euler_real_(3)   = quadStartPos(4, iQuad);
    % for mpc
    x_start = [Quadrotor(iQuad).pos_real_; Quadrotor(iQuad).vel_real_; ...
        Quadrotor(iQuad).euler_real_];
    z_start = zeros(model.nvar, 1);
    z_start([index.z.pos, index.z.vel, index.z.euler]) = x_start;
    mpc_plan = repmat(z_start, 1, model.N);
    % initialize MPC
    Quadrotor(iQuad).initializeMPC(x_start, mpc_plan);
    % to avoid whom in prioritied planning
    if Quadrotor(iQuad).modeCoor_ == -1
        Quadrotor(iQuad).nQuad_ = iQuad;
        Quadrotor(iQuad).npar_  = 24 + ...
            (Quadrotor(iQuad).nQuad_ - 1 + Quadrotor(iQuad).nDynObs_)*model.nParamPerObs;
        Quadrotor(iQuad).quad_path_ = zeros(3, model.N, Quadrotor(iQuad).nQuad_);
        Quadrotor(iQuad).quad_pathcov_ = zeros(6, model.N, Quadrotor(iQuad).nQuad_);
    end
end


%% Initialization graphic communicator
GraphicCom = CGraphicCom(true, cfg, model.nQuad, model.nDynObs, model.N);
% initialize ROS
GraphicCom.initializeROS();
% set default quad and obs size
for iQuad = 1 : model.nQuad
    GraphicCom.quad_size_(:, iQuad) = cfg.quad.size;
end
for jObs = 1 : model.nDynObs
    GraphicCom.obs_size_(:, jObs) = cfg.obs.size;
end


%% Logging variables
logsize = 10000;
% time
log_time            = zeros(1, logsize);              % loop time
% quad
log_quad_goal       = zeros(4, logsize, model.nQuad); % quad goal
log_quad_state_real = zeros(9, logsize, model.nQuad); % quad real state
log_quad_state_est  = zeros(9, logsize, model.nQuad); % quad estimated state
log_quad_state_cov  = zeros(9, 9, logsize, model.nQuad);% quad state estimation cov
log_quad_input      = zeros(4, logsize, model.nQuad); % quad input
log_quad_path       = zeros(3, model.N, logsize, model.nQuad); % quad mpc path
log_quad_mpc_plan   = zeros(model.nvar, model.N, logsize, model.nQuad); % quad complete mpc plan
log_quad_mpc_info   = NaN(3, logsize, model.nQuad);   % quad mpc solving info
% obs
log_obs_state_est   = zeros(6, logsize, model.nDynObs); % obs estimated state
log_obs_state_cov   = zeros(6, 6, logsize, model.nDynObs);% obs state estimation cov
log_obs_path        = zeros(3, model.N, logsize, model.nDynObs); % obs predicted path


%% Main control loop
fprintf('[%s] Looping... \n',datestr(now,'HH:MM:SS'));
% timers
dt_loop      = model.dt;                % delta t of the loop
mpc_solve_time = 0;
% iter
iter_loop    = 0;                       % number of loops

while(true)
    %% in each control loop
    
    % timer, start of the loop
    t_loop_start = tic;
    % loop count
    iter_loop = iter_loop + 1;
    
    % print on screen every ten loops
    if(mod(iter_loop, 10) == 0)
        fprintf('Loop: %d, MPC time: %.3f s, Frequency: %.3f Hz, RTF: %.2f\n', ...
            iter_loop, mpc_solve_time, 1/dt_loop, dt_loop/model.dt);
    end
    
    %% publish dyn obs path in simulation mode
    if cfg.modeSim == 1
        for jObs = 1 : model.nDynObs
            if DynObs(jObs).pos_real_(1) < -cfg.ws(1) || ...
                    DynObs(jObs).pos_real_(1) > cfg.ws(1) || ...
                    DynObs(jObs).pos_real_(2) < -cfg.ws(2) || ...
                    DynObs(jObs).pos_real_(2) > cfg.ws(2) || ...
                    DynObs(jObs).pos_real_(3) < 0.4 || ...
                    DynObs(jObs).pos_real_(3) > 2.0
                DynObs(jObs).randomState();
            end
            DynObs(jObs).getEstimatedObsState();
            DynObs(jObs).predictPathConstantV();
            DynObs(jObs).sendPath();
        end
    end
    
    %% controller for each quad
    for iQuad = 1 : model.nQuad

        % ===== Get estimated state of the ego quad =====
        Quadrotor(iQuad).getEstimatedSystemState();
        
        % ===== Set configuration parameters =====
        Quadrotor(iQuad).quad_goal_   = com_quad_goal(1:4, iQuad);
        Quadrotor(iQuad).mpc_coll_    = com_mpc_coll;
        Quadrotor(iQuad).mpc_weights_ = com_mpc_weights;
        
        % ===== Get predicted obstacles path with cov ===== 
        Quadrotor(iQuad).getObsPredictedPathCov();
        
        % ===== Get path and cov of other quads ===== 
        if Quadrotor(iQuad).modeCoor_ == -1       % for prioritied planning
            Quadrotor(iQuad).quad_path_(:, :, :) = ...
                coor_quad_path(:, :, 1 : iQuad);
            Quadrotor(iQuad).quad_pathcov_(:, :, :) = ...
                coor_quad_pathcov(:, :, 1 : iQuad);
        else
            Quadrotor(iQuad).quad_path_ = coor_quad_path;
            Quadrotor(iQuad).quad_pathcov_ = coor_quad_pathcov;
        end
        
        % ===== Set online parameters for the MPC ===== 
        Quadrotor(iQuad).setOnlineParamtersCov();
        
        % ===== Solve the mpc problem ===== 
        Quadrotor(iQuad).solveMPC();
        mpc_solve_time = Quadrotor(iQuad).mpc_info_.solvetime;
        
        % ===== Send and execute the control command ===== 
        Quadrotor(iQuad).step();
        
        % ===== Propagate path uncertainty covariance ===== 
        Quadrotor(iQuad).propagateStateCov();
        
        % ===== Communicate the planned mpc path and cov ===== 
        if Quadrotor(iQuad).modeCoor_ == 0 ...
                || Quadrotor(iQuad).modeCoor_ == -1 % sequential (prioritied) planning
            coor_quad_path(:, :, iQuad) = Quadrotor(iQuad).mpc_Path_;
            coor_quad_pathcov(:, :, iQuad) = Quadrotor(iQuad).mpc_PathCov_;
        elseif Quadrotor(iQuad).modeCoor_ == 2  % path prediction based on constant v
            Quadrotor(iQuad).predictPathConstantV();
        end
        
        %  ===== Data logging ===== 
        log_quad_goal(:, iter_loop, iQuad) = Quadrotor(iQuad).quad_goal_;
        log_quad_state_real(:, iter_loop, iQuad) = [Quadrotor(iQuad).pos_real_; ...
                                                    Quadrotor(iQuad).vel_real_; ...
                                                    Quadrotor(iQuad).euler_real_];
        log_quad_state_est(:, iter_loop, iQuad) = [Quadrotor(iQuad).pos_est_; ...
                                                   Quadrotor(iQuad).vel_est_; ...
                                                   Quadrotor(iQuad).euler_est_];
        log_quad_state_cov(1:3, 1:3, iter_loop, iQuad) = Quadrotor(iQuad).pos_est_cov_;
        log_quad_state_cov(4:6, 4:6, iter_loop, iQuad) = Quadrotor(iQuad).vel_est_cov_;
        log_quad_state_cov(7:9, 7:9, iter_loop, iQuad) = Quadrotor(iQuad).euler_est_cov_;
        log_quad_input(:, iter_loop, iQuad) = Quadrotor(iQuad).u_body_;
        log_quad_path(:, :, iter_loop, iQuad) = Quadrotor(iQuad).mpc_Path_;
        log_quad_mpc_plan(:, :, iter_loop, iQuad) = Quadrotor(iQuad).mpc_ZPlan_;
        log_quad_mpc_info(:, iter_loop, iQuad) = ...
            [Quadrotor(iQuad).mpc_info_.solvetime; ...
             Quadrotor(iQuad).mpc_info_.it; ...
             Quadrotor(iQuad).mpc_info_.pobj];
        
    end
    
    %% communicate to gui for visualization
    % quad
    for iTemp = 1 : model.nQuad
        com_quad_state(:, iTemp) = [Quadrotor(iTemp).pos_real_; ...
                                    Quadrotor(iTemp).vel_real_; ...
                                    Quadrotor(iTemp).euler_real_];
        com_quad_input(:, iTemp)   = Quadrotor(iTemp).u_body_;
        com_quad_slack(1, iTemp)   =  10*Quadrotor(iTemp).mpc_Zk_(index.z.slack);
        com_quad_path(:, :, iTemp) =  Quadrotor(iTemp).mpc_Path_;
        com_quad_pathcov(:, :, iTemp) = Quadrotor(iTemp).mpc_PathCov_;
        if Quadrotor(iTemp).modeCoor_ == 1  % path communication (distributed)
            coor_quad_path(:, :, iTemp) = Quadrotor(iTemp).mpc_Path_;
            coor_quad_pathcov(:, :, iTemp) = Quadrotor(iTemp).mpc_PathCov_;
        elseif Quadrotor(iQuad).modeCoor_ == 2  % path prediction based on constant v
            coor_quad_path(:, :, iQuad) = Quadrotor(iQuad).pred_path_;
            coor_quad_pathcov(:, :, iQuad) = Quadrotor(iQuad).pred_pathcov_;
        end
    end
    % obs
    com_obs_path = Quadrotor(model.nQuad).obs_path_;
    com_obs_state(1:3, :) = com_obs_path(1:3, 1, :);
    com_obs_state(4:6, :) = diff(com_obs_path(1:3, 1:2, :), 1, 2) / model.dt;
    com_obs_pathcov = Quadrotor(model.nQuad).obs_pathcov_;
    
    % data
    GraphicCom.quad_state_ = com_quad_state;
    GraphicCom.quad_input_ = com_quad_input;
    GraphicCom.quad_slack_ = com_quad_slack;
    GraphicCom.quad_path_  = com_quad_path;
    GraphicCom.quad_pathcov_ = com_quad_pathcov;
    GraphicCom.obs_state_  = com_obs_state;
    GraphicCom.obs_path_   = com_obs_path;
    GraphicCom.obs_pathcov_  = com_obs_pathcov;
    % publishing
    GraphicCom.sendQuadState();
    GraphicCom.sendQuadInput();
    GraphicCom.sendQuadSlack();
    GraphicCom.sendQuadPath();
    GraphicCom.sendQuadPathCov();
    GraphicCom.sendObsState();
    GraphicCom.sendObsPath();
    GraphicCom.sendObsPathCov();
    
    % para
    if GraphicCom.cfg_.setParaGui == 0
        GraphicCom.quad_goal_  = com_quad_goal;
        com_mpc_coll(3, 1) = 0.5*(1-erf(com_mpc_coll(3, 1)));
        com_mpc_coll(3, 2) = 0.5*(1-erf(com_mpc_coll(3, 2)));
        GraphicCom.mpc_coll_   = com_mpc_coll;
        GraphicCom.mpc_weights_= com_mpc_weights;
        GraphicCom.sendQuadGoal();
        GraphicCom.sendMPCColl();
        GraphicCom.sendMPCWeights();
    else
        GraphicCom.getQuadGoal();
        GraphicCom.getMPCColl();
        GraphicCom.getMPCWeights();
        com_quad_goal   =   GraphicCom.quad_goal_;
        com_mpc_coll    =   GraphicCom.mpc_coll_;
        com_mpc_coll(3, 1) = erfinv(1-2.0*com_mpc_coll(3, 1));
        com_mpc_coll(3, 2) = erfinv(1-2.0*com_mpc_coll(3, 2));
        com_mpc_weights =   GraphicCom.mpc_weights_;
    end
    
    %% timer, end of the loop   
    dt_loop = toc(t_loop_start);
    
    %% simulate dyn obs in simulation mode
    if cfg.modeSim == 1
        for jObs = 1 : model.nDynObs
            DynObs(jObs).step(min(dt_loop, model.dt));
        end
    end
    
    log_obs_state_est(:, iter_loop, :) = com_obs_state;
    log_obs_path(:, :, iter_loop, :) = com_obs_path;
    log_time(iter_loop) = dt_loop;
    
end




