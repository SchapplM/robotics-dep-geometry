% Berechne Kollision und Abstand einer Kapsel und einer Kugel
% 
% [dist, kol, pkol] = Kollision_Zylinder_Kugel(Kap, Kug)
% Eine Kapsel ist ein Zylinder mit Halbkugeln als Enden.
% 
% 
% Eingabe:
% Zyl: 1x7 Kapsel-Darstellung (Pkt 1, Pkt 2, Radius)
% Kug: 1x4 Kugel-Darstellung (Mittelpunkt, Radius)
% 
% Ausgabe:
% dist: Abstand der Hüllen von Kugel und Kapsel
% kol: 1 falls Kollision, sonst 0
% pkol: 2x3; 2 Punkte auf den Hüllflächen mit den kürzesten Abständen
%        Achtung: Dies entspricht nicht den tatsächlichen
%        Durchtrittspunkten der Kapsel durch die Kugel. Reicht
%        aber zur Kollisionserkennung
% d_min: 1x1;  Minimaler Abstand (negativ) der beiden Körper

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2013-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [dist, kol, pkol, d_min] = collision_capsule_sphere(Kap, Kug)

%% Coder Information
%#codegen
assert(isa(Kap,'double') && isreal(Kap) && all(size(Kap) == [1 7]) && ... 
     isa(Kug,'double') && isreal(Kug) && all(size(Kug) == [1 4])); 
   
%% Algorithmus
rg = Kap(1:3)'; % Anfangspunkt der Geraden
ug = Kap(4:6)'-Kap(1:3)';% Richtungsvektoren der Geraden
rk = Kug(4);
rz = Kap(7); % Radien von Kugel und Zylinder
Gerade = [rg', ug']; Punkt = Kug(1:3);  
% Prüfe kürzeste Entfernung zur Ersatz-Geraden
[dnorm, d, lambda, pg] = distance_line_point(Gerade, Punkt);
pkol = zeros(2, 3);
% Kugel liegt am ersten Ende des Zylinders
if lambda <= 0
  [dnorm, d] = distance_point_point(Kug(1:3), rg');
  pkol(2, :) = rg' - d'/dnorm*rz; % Punkt auf Zylinder-Anfang
  pkol(1, :) = Kug(1:3) + d'/dnorm*rk; % Punkt auf Kugeloberfläche
% am zweiten Ende
elseif lambda >= 1
  [dnorm, d] = distance_point_point(Kug(1:3), rg'+ug');
  pkol(2, :) = rg' + ug' - d'/dnorm*rz; % Punkt auf Zylinder-Ende
  pkol(1, :) = Kug(1:3) + d'/dnorm*rk; % Punkt auf Kugeloberfläche
% Dazwischen
else
  pkol(2, :) = pg' + d'/dnorm*rz; % Punkt auf dem Zylinder-Mantel
  pkol(1, :) = Kug(1:3)-d'/dnorm*rk; % Punkt auf Kugeloberfläche
end

% Abstand mit Radien vergleichen
d_min = - (rk + rz);
dist = dnorm - (rk + rz);
if dist < 0
  kol = 1;
else
  kol = 0;
end

