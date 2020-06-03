function ineq = mpc_nonlinIneq_chance(z, p, nObs)

    % define nonlinear inequalities for mpc

    global index                            % global index information
    
    %% obtaining necessary information
    % environment dim
    env_dim     =   p(index.p.envDim);      % [xdim, ydim, zdim]
    % ego mav 
    ego_pos     =   z(index.z.pos);         % current stage position [x, y, z]
    ego_size    =   p(index.p.size);        % size
    % ego position uncertainty covariance
    ego_pos_cov_vec = p(index.p.posCov);
    ego_pos_cov = [ ego_pos_cov_vec(1), ego_pos_cov_vec(4), ego_pos_cov_vec(6); ...
                    ego_pos_cov_vec(4), ego_pos_cov_vec(2), ego_pos_cov_vec(5); ...
                    ego_pos_cov_vec(6), ego_pos_cov_vec(5), ego_pos_cov_vec(3) ];
    % slacks
    slack_env   =   z(index.z.slack(1));

    %% environment boundary constraint
    cons_env    =   [ego_pos(1)/env_dim(1); ego_pos(2)/env_dim(2); ego_pos(3)/env_dim(3)] + slack_env;
    
    %% collision avoidance constraints
    cons_coll   =   [];
    for jObs = 1 : nObs
        % obtain obstacle information
        p_obs = p(index.p.obsParam(:, jObs));   % parameters of the obstacle
        obs_pos  = p_obs(index.p.obs.pos);      % position
        obs_size = p_obs(index.p.obs.size);     % size
        obs_delta_aux = p_obs(index.p.obs.coll(3)); % aux parameter for chance threshold
        obs_pos_cov_vec = p_obs(index.p.obs.posCov);% position uncertainty covariance
        obs_pos_cov = [ obs_pos_cov_vec(1), obs_pos_cov_vec(4), obs_pos_cov_vec(6); ...
                        obs_pos_cov_vec(4), obs_pos_cov_vec(2), obs_pos_cov_vec(5); ...
                        obs_pos_cov_vec(6), obs_pos_cov_vec(5), obs_pos_cov_vec(3) ];
        % approximated minkovski sum (ellipsoid)
        a = ego_size(1) + obs_size(1);
        b = ego_size(2) + obs_size(2);
        c = ego_size(3) + obs_size(3);
        % construct chance constraint
        Omega_root = [ 1/a, 0  , 0  ; ...
                       0  , 1/b, 0  ; ...
                       0  , 0  , 1/c ];
        pos_io = Omega_root * (ego_pos - obs_pos);
        cov_io = transpose(Omega_root) * (ego_pos_cov + obs_pos_cov) * Omega_root;
        pos_io_norm = sqrt(transpose(pos_io)*pos_io);
        c_io   = obs_delta_aux * sqrt(2*transpose(pos_io)*cov_io*pos_io) / pos_io_norm;
        cons_obs = pos_io_norm - 1 - c_io;
        % add for all obstacles
        cons_coll = [cons_coll; cons_obs];
    end

    %% combine inequality constraints
    ineq = [cons_env; cons_coll];
end