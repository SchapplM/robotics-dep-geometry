% Berechne Kollision und Abstand von zwei Kugeln
% 
% Eingabe:
% Kug1, Kug2 [1x4]
%   Kugeldarstellung (Mittelpunkt, Radius)
% 
% Ausgabe:
% dist
%   Abstand zwischen den Kugeln (Abstand der Außenflächen)
% kol
%   true falls Kollision, sonst false
% pkol [2x3]
%   2 Punkte des kürzesten Abstandes auf den Kugelhüllen
% d_min [1x1]
%   Minimal möglicher Abstand (negativ) der beiden Körper (maximal mögliche
%   Eindringtiefe; dient zur möglichen Normierung der Ausgabe dist)

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
if dist <= 0 % Kollision
  kol = true;
  if dnorm > 1e-12 % normaler Fall (nicht exakt identischer Mittelpunkt)
    % (bei dnorm nahe eps treten numerische Probleme auf)
    v = d';
    vnorm = dnorm;
  else % Kugeln exakt mit selbem Mittelpunkt. Kürzester Abstand nicht eindeutig
    v = [0,0,1]; % beliebiger Vektor
    vnorm = 1;
  end
else % keine Kollision
  kol = false;
  v = d'; % Abstand ist immer eindeutig. Keine Unterscheidung wie oben notwendig.
  vnorm = dnorm;
end
pkol = NaN(2,3);
pkol(1, :) = Kug1(1:3)+v/vnorm*Kug1(4);
pkol(2, :) = Kug2(1:3)-v/vnorm*Kug2(4);
