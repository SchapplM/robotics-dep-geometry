clear; close all;
% define cylinder radius and height
h=100;
r=30;
% define cylinder to draw disc
[X,Y,Z]= cylinder(r);
Z = Z*h;
% define transformationmatrix
T = [     0    0.9400   -0.3400  -50.0000
    -0.9700   -0.0900   -0.2400   20.0000
    -0.2600    0.3300    0.9100   30.0000
          0         0         0    1.0000];
% transform cylinder
[x,y,z]=transform_disc(X,Y,Z,T);
% define plot color
C = [0.5,0.5,0.5];
% plot transformed cylinder
figure;
surf(x,y,z,'FaceColor',C);
% plot top and bottom
hold on; 
fill3(x(1,:),y(1,:),z(1,:),C)
fill3(x(2,:),y(2,:),z(2,:),C)
