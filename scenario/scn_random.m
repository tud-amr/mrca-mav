function [quadStartPos, quadStartVel, quadEndPos] = scn_random(nQuad, xDim, yDim, zDim)

    %% quad initial and end positions (yaw)
    quadStartPos    =   zeros(4, nQuad);
    quadStartVel    =   zeros(3, nQuad);
    quadEndPos      =   zeros(4, nQuad);
    
    %% generate random starting and end position
    resolution = 0.5;
    % discrete the workspace
    xL = xDim(2) - xDim(1);
    yL = yDim(2) - yDim(1);
    zL = zDim(2) - zDim(1);
    % number of discrete points in each dimension
    xN = floor(xL / resolution);
    yN = floor(yL / resolution);
    zN = max(floor(zL / 0.1), 2);
    % non-repeating integer sequence
    xidx_rand = randperm(xN);
    yidx_rand = randperm(yN);
%     zidx_rand = randperm(zN);
    % random starting and end pos
    for iQuad = 1 : nQuad
        xN_i = xidx_rand(iQuad);
        yN_i = yidx_rand(iQuad);
        zN_i = randi([0, zN]);
        quadStartPos(1, iQuad) = xDim(1) + resolution*xN_i;
        quadStartPos(2, iQuad) = yDim(1) + resolution*yN_i;
        quadStartPos(3, iQuad) = zDim(1) + 0.1*zN_i;
    end
    xidx_rand = randperm(xN);
    yidx_rand = randperm(yN);
%     zidx_rand = randperm(zN);
    % random starting and end pos
    for iQuad = 1 : nQuad
        xN_i = xidx_rand(iQuad);
        yN_i = yidx_rand(iQuad);
        zN_i = randi([0, zN]);
        quadEndPos(1, iQuad) = xDim(1) + resolution*xN_i;
        quadEndPos(2, iQuad) = yDim(1) + resolution*yN_i;
        quadEndPos(3, iQuad) = zDim(1) + 0.1*zN_i;
    end

end