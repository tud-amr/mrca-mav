function cost = mpc_objectiveN_basic(z, p, nObs)

    % define horizon terminal cost function for mpc

    global index pr                         % global index information
    
    %% obtaining necessary information
    % cost terms weights
    w_wp        =   p(index.p.weights(1));  % waypoint cost weight
    w_input     =   p(index.p.weights(2));  % control inputs cost weight
    w_coll      =   p(index.p.weights(3));  % collision cost weight
    w_slack     =   p(index.p.weights(4));  % slack cost weight
    % ego mav 
    ego_pos     =   z(index.z.pos);         % current stage position [x, y, z]
    ego_yaw     =   z(index.z.euler(3));    % current yaw [psi]
    ego_input   =   z(index.z.inputs);      % control input [phi_c, theta_c, vz_c, psi_rate_c]
    ego_size    =   p(index.p.size);        % size [a0, b0, c0]
    ego_start   =   p(index.p.startPos);    % start [x0, y0, z0, psi0]
    ego_goal    =   p(index.p.wayPoint);    % goal [xg, yg, zg, psig]
    % slacks
    slack_env   =   z(index.z.slack(1));
    slack_coll  =   z(index.z.slack(2));
    
    %% waypoint cost
    % also normalization first
    lenToGoal   =   (ego_goal(1:3) - ego_start(1:3))' ...
        * (ego_goal(1:3) - ego_start(1:3)); % length between current start
                                            % and goal position, using
                                            % quadratic form
    lenToGoal   =   max(lenToGoal, 1);      % in case arriving at goal posistion
    % the weighting matrix and cost of position navigation
    Q_wp_pos    =   w_wp * diag([1.0; 1.0; 1.0]);
    cost_wp_pos =   (ego_goal(1:3) - ego_pos)' * Q_wp_pos ...
        * (ego_goal(1:3) - ego_pos) / lenToGoal;
    % the cost of yaw tracking
    cost_wp_yaw =   w_wp * (ego_goal(4) - ego_yaw)^2;
    % total navigation cost
    cost_wp = cost_wp_pos + cost_wp_yaw;
    
    %% control input cost
    % normalize the input to [-1, 1]
    ego_input_normalized = [ego_input(1)/pr.input.maxRoll; ...
        ego_input(2)/pr.input.maxPitch; ego_input(3)/pr.input.maxVz; ...
        ego_input(3)/pr.input.maxYawRate];
    % the weighting matrix
    Q_input = w_input * diag([1.0; 1.0; 1.0; 1.0]);     % can be ajusted to
                                                        % penalize more to
                                                        % some input
    % the cost
    cost_input = ego_input_normalized' * Q_input * ego_input_normalized;
    
    %% collision potential cost
    % define an empty cost vector
    obs_coll_cost = [];
    % cost for each obstacle
    for jObs = 1 : nObs
        % obtain obstacle information
        p_obs = p(index.p.obsParam(:, jObs));   % parameters of the obstacle
        obs_pos  = p_obs(index.p.obs.pos);      % position
        obs_size = p_obs(index.p.obs.size);     % size
        obs_lambda = p_obs(index.p.obs.coll(1)); % sigmoid function, lambda
        obs_buffer = p_obs(index.p.obs.coll(2)); % sigmoid function, buffer
        % approximated minkovski sum (ellipsoid)
        a = ego_size(1) + obs_size(1);
        b = ego_size(2) + obs_size(2);
        c = ego_size(3) + obs_size(3);
        % pseudu distance to the obstacle (should be larger than 1)
        d_vec = ego_pos - obs_pos;
        d = sqrt(d_vec(1)^2/a^2 + d_vec(2)^2/b^2 + d_vec(3)^2/c^2);
        % logistic (sigmoid) function cost (buffer should be larger than 1)
        jObs_coll_cost = 1 / (1+exp(obs_lambda*(d - obs_buffer)));
        % add to the vector
        obs_coll_cost = [obs_coll_cost; jObs_coll_cost];
    end
    % the weight matrix
    Q_coll = w_coll * eye(nObs);
    % the cost
    if nObs > 1
        cost_coll = obs_coll_cost' * Q_coll * obs_coll_cost;
    else
        cost_coll = 0;
    end
    
    %% slack cost
    cost_slack = w_slack*10*slack_env^2 + w_slack*slack_coll^2;
    
    %% combine all cost
    cost = cost_wp + cost_input + cost_coll + cost_slack;
    

end
