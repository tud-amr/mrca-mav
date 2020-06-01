function points = pointsCircle(center, radius)
    % Create points for 2D circle
    theta = 0: 0.1 : 2*pi+0.1;
    v = [1, 0 ; 0, 1];
    points = repmat(center', 1, ...
        size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
end
