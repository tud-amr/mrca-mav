function points = pointsCircle3D(center, normal, radius)
    % Create points for 3D circle
    theta = 0 : 0.1 : 2*pi+0.1;
    v = null(normal);               % Null space (normal*v = 0);
    points = repmat(center', 1, ...
        size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
end
