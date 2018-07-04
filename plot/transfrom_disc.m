%transfromDisc(x,y,z,T) transforms disc edge points x,y,z of a cylinder to 
%a pose, defined by the transformation matrix T. The edge points can be 
%create using the matlab cylinder function
function [X,Y,Z]=transfromDisc(x,y,z,T)
% translation vector
p = T(1:3,4);
% rotationmatrix
R = T(1:3,1:3);

% transform all edge points
X = zeros(size(x));
Y = X;
Z = X;
for k=1:size(x,2)
    for n=1:size(x,1)
        tmp = p+R*[x(n,k);y(n,k);z(n,k)];
        X(n,k) = tmp(1);
        Y(n,k) = tmp(2);
        Z(n,k) = tmp(3);
    end
end
    