% Berechne Kollision und Abstand von zwei Kugeln
% 
% [dist, kol, pkol] = Kollision_Kugel_Kugel(Kug1, Kug2)
% 
% 
% Eingabe:
% Kug1, Kug2: 1x4 Kugeldarstellung (Mittelpunkt, Radius)
% 
% Ausgabe:
% dist: Abstand zwischen den Kugeln (Abstand der Außenflächen
% kol: 1 falls Kollision, sonst 0
% pkol: 2x3; 2 Punkte auf den Kugelhüllen des kürzesten Abstandes
% d_min: 1x1;  Minimaler Abstand (negativ) der beiden Körper

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2013-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [dist, kol, pkol, d_min] = collision_sphere_sphere(Kug1, Kug2)

%% Coder Information
%#codegen

assert(isa(Kug1,'double') && isreal(Kug1) && all(size(Kug1) == [1 4]) && ... 
       isa(Kug2,'double') && isreal(Kug2) && all(size(Kug2) == [1 4])); 

%% Algorithmus       
% kleinsten Abstand zwischen Kugel-Mittelpunkten
[dnorm, d] = distance_point_point(Kug1(1:3), Kug2(1:3));
% Abstand mit Radien vergleichen
d_min = - (Kug1(4) + Kug2(4));
dist = dnorm - (Kug1(4) + Kug2(4));
if dist < 0
    kol = 1;
else
    kol = 0;
end
pkol = zeros(2, 3);
pkol(1, :) = Kug1(1:3)+d'/dnorm*Kug1(4);
pkol(2, :) = Kug2(1:3)-d'/dnorm*Kug2(4);
