function points = pointsEllipse(center, a, b)
    % Create points for 2D ellipse
    theta = 0: 0.1 : 2*pi+0.1;
    v = [a, 0 ; 0, b];
    points = repmat(center', 1, ...
        size(theta,2))+(v(:,1)*cos(theta)+v(:,2)*sin(theta));
end
