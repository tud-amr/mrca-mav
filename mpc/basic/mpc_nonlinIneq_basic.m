function ineq = mpc_nonlinIneq_basic(z, p, nObs)

    % define nonlinear inequalities for mpc

    global index                            % global index information
    
    %% obtaining necessary information
    % environment dim
    env_dim     =   p(index.p.envDim);      % [xdim, ydim, zdim]
    % ego mav 
    ego_pos     =   z(index.z.pos);         % current stage position [x, y, z]
    ego_size    =   p(index.p.size);        % size
    % slacks
    slack_env   =   z(index.z.slack(1));
    slack_coll  =   z(index.z.slack(2));

    %% environment boundary constraint
    cons_env    =   [ego_pos(1)/env_dim(1); ego_pos(2)/env_dim(2); ego_pos(3)/env_dim(3)] + slack_env;
    
    %% collision avoidance constraints
    cons_coll   =   [];
    for jObs = 1 : nObs
        % obtain obstacle information
        p_obs = p(index.p.obsParam(:, jObs));   % parameters of the obstacle
        obs_pos  = p_obs(index.p.obs.pos);      % position
        obs_size = p_obs(index.p.obs.size);     % size
        % approximated minkovski sum (ellipsoid)
        a = ego_size(1) + obs_size(1);
        b = ego_size(2) + obs_size(2);
        c = ego_size(3) + obs_size(3);
        % collision avoidance constraint, d^2 - 1 + slack >= 0, (slack >= 0)
        d = ego_pos - obs_pos;                  % relative position
        cons_obs = sqrt(d(1)^2/a^2 + d(2)^2/b^2 + d(3)^2/c^2) -1 + slack_coll;
        % add for all obstacles
        cons_coll = [cons_coll; cons_obs];
    end

    %% combine inequality constraints
    ineq = [cons_env; cons_coll];
end