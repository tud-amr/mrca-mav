function collision_mtx = collision_check(pos_quad, size_quad, ...
    pos_obs, size_obs, nQuad, nObs)
    % 
    % Check if collision happends among multiple agents 
    % Input:
    %   -- pos:   3 x n
    %   -- size:  3 x n
    % Output:
    %   -- collision_mtx: n x n
    
    collision_mtx_quad = zeros(nQuad, nQuad);
    for i = 1 : nQuad-1
        for j = (i+1) : nQuad
            pos_ij = pos_quad(:, i) - pos_quad(:, j);
            size_ij = size_quad(:, i) + size_quad(:, j);
            d_ij = pos_ij(1)^2/size_ij(1)^2 + pos_ij(2)^2/size_ij(2)^2 + ...
                pos_ij(3)^2/size_ij(3)^2;
            if d_ij < 1 && d_ij > 0.001
                collision_mtx_quad(i, j) = 1;
            end
        end
    end
    
    collision_mtx_obs = zeros(nQuad, nObs);
    for i = 1 : nQuad
        for j = 1 : nObs
            pos_ij = pos_quad(:, i) - pos_obs(:, j);
            size_ij = size_quad(:, i) + size_obs(:, j);
            d_ij = pos_ij(1)^2/size_ij(1)^2 + pos_ij(2)^2/size_ij(2)^2 + ...
                pos_ij(3)^2/size_ij(3)^2;
            if d_ij < 1 && d_ij > 0.001
                collision_mtx_obs(i, j) = 1;
            end
        end
    end
    
    collision_mtx = [collision_mtx_quad, collision_mtx_obs];
    
end


