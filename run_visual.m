% Script for visualizing multi-quad collision avoidance

%% Clean workspace
clear
close all
clearvars
clearvars -global
clc


%% Initialization
% problem
initialize;


%% Initialize GUI
% creating gui
MainGui = main_gui(cfg);
movegui(MainGui, 'north');
% get all handles from gui
main_gui_handles   = guidata(MainGui);
h3d_plot  = main_gui_handles.fig_main;
h2d_plot1 = main_gui_handles.fig_topdown;
h2d_plot2 = main_gui_handles.fig_side;
view(h3d_plot, 3);


%% Create graphic communicator
GraphicCom = CGraphicCom(false, cfg, model.nQuad, model.nDynObs, model.N);
% initialize ROS
GraphicCom.initializeROS();
% set default quad and obs size
for iQuad = 1 : model.nQuad
    GraphicCom.quad_size_(:, iQuad) = cfg.quad.size;
end
for jObs = 1 : model.nDynObs
    GraphicCom.obs_size_(:, jObs) = cfg.obs.size;
end
% initialize setable running para
GraphicCom.quad_goal_   = cfg.quad.goal;
GraphicCom.mpc_coll_    = [cfg.quad.coll, cfg.obs.coll];
GraphicCom.mpc_weights_ = [cfg.weightStage, cfg.weightN];
% if new running para set
flagNewGuiPara          = 0;
% if keeping swapping
flagKeepSwap            = 0;
% if keeping random
flagKeepRandom          = 0;
% if eightswap
flagEightSwap           = 0;

%% Color map
color_map = cell(20, 1);
color_map(1:7) = {'r', 'g', 'b', 'm' , 'c', 'k', 'y'};
for iColor = 8 : 20
    color_map(iColor) = {rand(1,3)};
end


%% Create quad visualization objects
for iQuad = 1 : model.nQuad
    quad_size = cfg.quad.size;
    VisualQuad(iQuad) = CVisualQuad(iQuad, model.N, quad_size, cfg, h3d_plot, ...
        [h2d_plot1, h2d_plot2], color_map{iQuad}, 2*cfg.quad.size(1));
end


%% Create obs visulization objects
for jObs = 1 : model.nDynObs
    obs_size = cfg.obs.size;
    VisualObs(jObs) = CVisualDynObs(jObs, model.N, obs_size, h3d_plot, ...
        [h2d_plot1, h2d_plot2], color_map{jObs});
end


%% =======================================================================
%% Main loop
fprintf('[%s] Looping... \n',datestr(now,'HH:MM:SS'));
% timers
dt_loop      = model.dt;                % delta t of the loop
t_start      = seconds(rostime('now')); % start time before the entire loop
% iter
iter_loop    = 0;                       % number of loops

pause(0.5);
while(true)
    %% in each control loop
    
    % timer, start of the loop
    t_loop_start = tic;
    % elapsed time
    t_now = seconds(rostime('now'));
    t_elapsed = t_now - t_start;
    % loop count
    iter_loop = iter_loop + 1;
    
    % print on screen every ten loops
    if(mod(iter_loop, 10) == 0)
        fprintf('Loop: %d, Frequency: %.3f Hz, RTF: %.2f\n', ...
            iter_loop, 1/dt_loop, dt_loop/model.dt);
    end
    
    %% Get current GUI data handles
    main_gui_handles = guidata(MainGui);
    flagNewGuiPara   = main_gui_handles.UserData.flagNewGuiPara;
    flagKeepSwap     = main_gui_handles.UserData.flagKeepSwap;
    flagKeepRandom   = main_gui_handles.UserData.flagKeepRandom;
    flagEightSwap    = main_gui_handles.UserData.flagEightSwap;
    
    %% Communicate
    % quad
    GraphicCom.getQuadState();
    GraphicCom.getQuadPath();
    GraphicCom.getQuadPathCov();
    GraphicCom.getQuadInput();
    GraphicCom.getQuadSlack();
    % para
    if GraphicCom.cfg_.setParaGui == 0
        GraphicCom.getQuadGoal();
        GraphicCom.getMPCColl();
        GraphicCom.getMPCWeights();
    else
        % goal is further determined
        if flagKeepSwap == 0 && flagKeepRandom == 0 && flagEightSwap == 0
            GraphicCom.quad_goal_   = main_gui_handles.UserData.quad_goal;
        elseif flagKeepSwap == 1
            if mod(t_elapsed, 4) <= 0.2
                GraphicCom.quad_goal_(1:2, :) = -GraphicCom.quad_goal_(1:2, :);
                flagNewGuiPara = 1;
            end
        elseif flagKeepRandom == 1
            if mod(t_elapsed, 4) <= 0.2
                GraphicCom.quad_goal_(1:3, :) = RandWayPoints(1:3, iter_loop, :);
                flagNewGuiPara = 1;
            end
        elseif flagEightSwap == 1
            flagNewGuiPara = 1;
            % Eight figure
            ta = 1.5;
            tb = 0.8;
            tperiod = 18; %s
            set_x = ta*cos(2*pi*t_elapsed/tperiod);
            set_y = tb*sin(4*pi*t_elapsed/tperiod);
            GraphicCom.quad_goal_(1:2, 1) = [set_x; set_y];
            if mod(t_elapsed, 4) <= 0.2
                GraphicCom.quad_goal_(1:2, 2) = -GraphicCom.quad_goal_(1:2, 2);
            end
        end
        GraphicCom.mpc_coll_    = main_gui_handles.UserData.mpc_coll;
        GraphicCom.mpc_weights_ = main_gui_handles.UserData.mpc_weights;
        GraphicCom.sendQuadGoal();
        GraphicCom.sendMPCColl();
        GraphicCom.sendMPCWeights();
    end
    % obs
    GraphicCom.getObsSize();
    GraphicCom.getObsState();
    GraphicCom.getObsPath();
    GraphicCom.getObsPathCov();
    
    %% collision check and warning
    coll_mtx_all = collision_check(GraphicCom.quad_state_(1:3, :), ...
                                   GraphicCom.quad_size_(1:3, :), ...
                                   GraphicCom.obs_state_(1:3, :), ...
                                   GraphicCom.obs_size_(1:3, :), ...
                                   model.nQuad, model.nDynObs);
    if sum(sum(coll_mtx_all)) > 0
        warning('Collision Happens!')
    end
    
    %% Table update in GUI
    % ===== quad state =====
    table_quad_state_temp = GraphicCom.quad_state_(1:6, :);     % to deg
    for iTemp = 1 : size(table_quad_state_temp, 2)
        table_quad_state_temp(7, iTemp) = norm(table_quad_state_temp(4:6, iTemp));
    end
    set(main_gui_handles.uitable_quad_state, 'Data', table_quad_state_temp, ...
        'ColumnFormat', repmat({'bank'}, 1, model.nQuad));
    % ===== quad euler =====
    set(main_gui_handles.uitable_quad_euler, 'Data', ...
        rad2deg(GraphicCom.quad_state_(7:9, :)), ...
        'ColumnFormat', repmat({'bank'}, 1, model.nQuad));
    % ===== quad commands =====
    table_quad_input_temp(1:4, :) = rad2deg(GraphicCom.quad_input_);    % to deg
    table_quad_input_temp(3, :)   = GraphicCom.quad_input_(3, :);
    table_quad_input_temp(5:6, :) = GraphicCom.quad_slack_;
    set(main_gui_handles.uitable_quad_input, 'Data', table_quad_input_temp, ...
        'ColumnFormat', repmat({'bank'}, 1, model.nQuad));
    % ===== quad goal =====
    if GraphicCom.cfg_.setParaGui == 0 || flagNewGuiPara == 1
        table_quad_goal_temp = GraphicCom.quad_goal_;               % to deg
        table_quad_goal_temp(4, :) = rad2deg(GraphicCom.quad_goal_(4, :));
        set(main_gui_handles.uitable_quad_goal, 'Data', table_quad_goal_temp, ...
            'ColumnFormat', repmat({'bank'}, 1, model.nQuad));
    end
    % ===== mpc coll =====
    if GraphicCom.cfg_.setParaGui == 0 || flagNewGuiPara == 1
        set(main_gui_handles.uitable_mpc_coll, 'Data', GraphicCom.mpc_coll_);
    end
    % ===== mpc weights =====
    if GraphicCom.cfg_.setParaGui == 0 || flagNewGuiPara == 1
        set(main_gui_handles.uitable_mpc_weights, 'Data', GraphicCom.mpc_weights_);
    end
    % ===== Flag needs to set to 0 after updating table =====
    main_gui_handles.UserData.flagNewGuiPara = 0;
    guidata(MainGui, main_gui_handles);
    flagNewGuiPara = 0;
    
    %% Update visualization for each quad
    for iQuad = 1 : model.nQuad
        % update real-time info
        VisualQuad(iQuad).cfg_        = main_gui_handles.UserData.cfg;
        VisualQuad(iQuad).quad_state_ = GraphicCom.quad_state_(:, iQuad);
        VisualQuad(iQuad).quad_size_  = GraphicCom.quad_size_(:, iQuad);
        VisualQuad(iQuad).quad_goal_  = GraphicCom.quad_goal_(:, iQuad);
        VisualQuad(iQuad).quad_path_  = GraphicCom.quad_path_(:, :, iQuad);
        % cov
        for iStage = 1 : GraphicCom.N_
            cov_temp = GraphicCom.quad_pathcov_(:, iStage, iQuad);   % 6x1
            VisualQuad(iQuad).quad_path_cov_(:, :, iStage) = ...
                [cov_temp(1), cov_temp(4), cov_temp(6); ...
                 cov_temp(4), cov_temp(2), cov_temp(5); ...
                 cov_temp(6), cov_temp(5), cov_temp(3)];
        end
        VisualQuad(iQuad).quad_state_cov_(1:3, 1:3) = ...
            VisualQuad(iQuad).quad_path_cov_(1:3, 1:3, 1);
        % update visual
        VisualQuad(iQuad).setPose();
    end
    
    %% Update visualization for each obs
    for jObs = 1 : model.nDynObs
        % update real-time infor
        VisualObs(jObs).obs_state_    = GraphicCom.obs_state_(:, jObs);
        VisualObs(jObs).obs_size_     = GraphicCom.obs_size_(:, jObs);
        VisualObs(jObs).obs_path_     = GraphicCom.obs_path_(:, :, jObs);
        % update visual 
        VisualObs(jObs).setPose();
    end
    
    drawnow limitrate
    
    % timer, end of the loop
%     pause(0.005);
    dt_loop = toc(t_loop_start);
    
end
