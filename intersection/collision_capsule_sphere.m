% Berechne Kollision und Abstand einer Kapsel und einer Kugel
% Eine Kapsel ist ein Zylinder mit Halbkugeln als Enden.
% 
% Eingabe:
% Kap [1x7]
%   Kapsel-Darstellung (Pkt 1, Pkt 2, Radius)
% Kug [1x4]
%   Kugel-Darstellung (Mittelpunkt, Radius)
% 
% Ausgabe:
% dist [1x1]
%   Abstand der Hüllen von Kugel und Kapsel (bei größter Annäherung)
% kol [1x1 logical]
%   true falls Kollision, sonst false
% pkol [2x3]
%   2 Punkte auf den Hüllflächen mit den kürzesten Abständen
%   Achtung: Dies entspricht nicht (immer) den tatsächlichen
%   Durchtrittspunkten der Kapsel durch die Kugel. Reicht
%   aber zur Kollisionserkennung
% d_min [1x1]
%   Minimal möglicher Abstand (negativ) der beiden Körper (maximal mögliche
%   Eindringtiefe; dient zur möglichen Normierung der Ausgabe dist)

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
% Kugel liegt an einem Ende des Zylinders. Kollision mit Halbkugel möglich
if lambda <= 0 || lambda >= 1
  if lambda <= 0
    p = rg; % Erste Halbkugel der Kapsel -> setze lambda=0
  else
    p = rg+ug; % Zweite Halbkugel der Kapsel -> setze lambda=1
  end
  % Siehe collision_sphere_sphere. Hier aber Anpassung der
  % Durchdringungsrichtung je nach Ausrichtung der Kapsel.
  % Ansonsten kann die Verbindung nur innerhalb der Kapsel liegen ohne
  % Verbindung mit der Außenfläche.
  % Richtung in collision_sphere_sphere bei Identität nicht definiert.
  [dnorm, d] = distance_point_point(p', Kug(1:3));
  d_min = - (rz+rk);
  dist = dnorm - (rz+rk);
  if dist <= 0 % Kollision zwischen Halbkugel und Kugel
    kol = true;
    if dnorm > 1e-12 % normaler Fall (nicht exakt identischer Mittelpunkt)
      % (bei dnorm nahe eps treten numerische Probleme auf)
      v = -d'; % eindeutiger Abstandsvektor bestimmt
      vnorm = dnorm;
    else % Kugeln exakt mit selbem Mittelpunkt. Kürzester Abstand nicht eindeutig
      v = ug'; % Vektor entspricht Kapsel-Ausrichtung
      vnorm = norm(ug);
    end
  else % keine Kollision
    kol = false;
    v = -d'; % Abstand ist immer eindeutig. Keine Unterscheidung wie oben notwendig.
    vnorm = dnorm;
  end
  pkol = NaN(2,3);
  pkol(1, :) = Kug(1:3)+v/vnorm*rk; % Kollisionspunkt an Kugel
  pkol(2, :) = p'-v/vnorm*rz; % Punkt an Halbkugel der Kapsel
% Kugel liegt zwischen Enden des Zylinders. Kollision mit Mantelfläche ist
% möglich
else
  if dnorm > 1e-12 % alles in Ordnung.
    v = d';
    vnorm = dnorm;
  else
    % Falls die Kugel zufälligerweise genau auf der Mittellinie ist, ist
    % die Richtung nicht bestimmt. Nehme beliebige Richtung senkrecht auf
    % der Zylinderachse. Diese Richtung hat auch keinen Einfluss auf das
    % Ergebnis (Kugel ist ja genau in der Mitte.
    if ug(1)~=ug(3) % so senkrecht mit folgender Operation
      v = cross(ug, flipud(ug))';
    else
      v = ug([1 3 2]); % beliebiger anderer Vektor, da erster nicht geht
    end
    vnorm = norm(v);
  end
  pkol(2, :) = pg' + v/vnorm*rz; % Punkt auf dem Zylinder-Mantel
  pkol(1, :) = Kug(1:3)-v/vnorm*rk; % Punkt auf Kugeloberfläche

  % Abstand mit Radien vergleichen
  d_min = - (rk + rz);
  dist = dnorm - (rk + rz);
  if dist < 0, kol = true;
  else,        kol = false; end
end
