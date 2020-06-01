function [xData, yData, zData] = pointsErrorEllipsoid(mu, covP, dis_m, radius)
% Input:  mean vector (3x1), covariance matrix (3x3) and Mahalanobis distance
% Output: three-dimensional surface data

% 1-sigma ellipsoid
[eigvec,eigval] = eig(covP);
[xData,yData,zData] = ellipsoid(0,0,0,1,1,1);
XYZ = [xData(:),yData(:),zData(:)]*sqrt(eigval)*eigvec';

% enlarge with Mahalanobis distance
XYZ(:,1) = dis_m*XYZ(:,1);      % x data
XYZ(:,2) = dis_m*XYZ(:,2);      % y data
XYZ(:,3) = dis_m*XYZ(:,3);      % z data

% adding extra robot radius
for i = 1 : length(XYZ(:,1))
    dis_e = norm(XYZ(i,:));
    XYZ(i,:) = XYZ(i,:)*((dis_e+radius)/dis_e);
end

% moving with mean vector
xData(:) = XYZ(:,1) + mu(1);
yData(:) = XYZ(:,2) + mu(2);
zData(:) = XYZ(:,3) + mu(3);

end
