function [quadStartPos, quadStartVel, quadEndPos] = scn_circle(nQuad, R)

    %% quad initial and end positions (yaw)
    quadStartPos    =   zeros(4, nQuad);
    quadStartVel    =   zeros(3, nQuad);
    quadEndPos      =   zeros(4, nQuad);
    
    angle = 2.0*pi / nQuad;
    for iQuad = 1 : nQuad
        % initial
        angle_i = deg2rad(0) + angle*(iQuad-1);
        quadStartPos(1:2, iQuad) = [R*cos(angle_i); R*sin(angle_i)];
        quadStartPos(3, iQuad)   = 1.2;          % flying height
        quadStartPos(4, iQuad)   = deg2rad(0);   % yaw
        
        % end
        quadEndPos(1:2, iQuad)   = -quadStartPos(1:2, iQuad);
        quadEndPos(3, iQuad)     =  quadStartPos(3, iQuad);
        quadEndPos(4, iQuad)     =  quadStartPos(4, iQuad);
    end
    
end