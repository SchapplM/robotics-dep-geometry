% Berechne Kollision und Abstand zweier Kapseln
% Eine Kapsel ist ein Zylinder mit Halbkugeln als Enden
% 
% Eingabe:
% Kap11, Kap2 [1x7]
%   Kapsel-Darstellung (Pkt 1, Pkt 2, Radius)
% 
% Ausgabe:
% dist
%   Abstand der Kapsel-Hüllen am Punkt größter Annäherung
% kol
%   true falls Kollision, sonst false
% pkol [2x3]
%   2 Punkte des kürzesten Abstandes
% d_min [1x1]
%   Minimal möglicher Abstand (negativ) der beiden Körper (maximal mögliche
%   Eindringtiefe; dient zur möglichen Normierung der Ausgabe dist)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2013-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [dist, kol, pkol, d_min] = collision_capsule_capsule(Kap1, Kap2)

%% Coder Information
%#codegen
assert(isa(Kap1,'double') && isreal(Kap1) && all(size(Kap1) == [1 7]) && ... 
       isa(Kap2,'double') && isreal(Kap2) && all(size(Kap2) == [1 7]));
%% Algorithmus
% Umwandlung in Geraden in Parameterform
rg = Kap1(1:3)'; rh = Kap2(1:3)'; % Anfangspunkte der Geraden
ug = Kap1(4:6)'-Kap1(1:3)'; uh = Kap2(4:6)'-Kap2(1:3)'; % Richtungsvektoren der Geraden
rad1 = Kap1(7); rad2 = Kap2(7); % Radien der Zylinder
[dnorm, d, lambda, mu, pg, ph] = distance_line_line([rg', ug'], [rh', uh']);

% Punkte des kürzesten Abstandes liegen auf Zylindermanteln.
% Das heißt: mu und lambda sind (beide) zwischen 0 und 1
% Benutze Abfrage >=1, da Ergebnis lambda=1+eps möglich ist. Dann
% divergiert die Lösung aus der mex-Datei mit der Matlab-Funktion.
if all([mu, lambda]>=0 & [mu, lambda]<=1)
  if dnorm > 1e-12 % alles in Ordnung.
    v = d';
    vnorm = dnorm;
  else
    % Falls die Kugel zufälligerweise genau auf der Mittellinie ist, ist
    % die Richtung nicht bestimmt. Nehme beliebige Richtung senkrecht auf
    % der ersten Zylinderachse. Die Richtung hat hier Einfluss auf das
    % Ergebnis. TODO: Die Wahl ist vielleicht noch nicht sinnvoll, da nicht
    % die Längste Verbindung gewählt wird.
    if ug(1)~=ug(3) % so senkrecht mit folgender Operation
      v = cross(ug, flipud(ug))';
    else
      v = ug([1 3 2]); % beliebiger anderer Vektor, da erster nicht geht
    end
    vnorm = norm(v);
  end
  pkol = [pg'+v/vnorm*Kap1(7); ph'-v/vnorm*Kap2(7)];
  % Abstand der Linien mit Radien der Zylinder vergleichen
  d_min = - (Kap1(7) + Kap2(7)); 
  dist = dnorm - (Kap1(7) + Kap2(7));
  if dist < 0
    kol = true;
  else
    kol = false;
  end

% Beide Punkte des kürzesten Abstandes liegen auf 
% Deckel- oder Bodenfläche der Zylinder bzw. der Halbkugeln
elseif all([lambda, mu]<=0 | [lambda, mu] >=1)
  % Prüfe, welcher Endpunkt am nächsten an einem anderen Endpunkt liegt
  [dnorm11, ~] = distance_point_point(rg', rh');
  [dnorm21, ~] = distance_point_point(rg'+ug', rh');
  [dnorm12, ~] = distance_point_point(rg', rh'+uh');
  [dnorm22, ~] = distance_point_point(rg'+ug', rh'+uh');
  [~, I] = min([dnorm11;dnorm21;dnorm12;dnorm22]);
  switch I
    case 1, Punkt1 = rg';     Punkt2 = rh';
    case 2, Punkt1 = rg'+ug'; Punkt2 = rh';
    case 3, Punkt1 = rg';     Punkt2 = rh'+uh';
    case 4, Punkt1 = rg'+ug'; Punkt2 = rh'+uh';
    otherwise, Punkt1 = NaN(3,1); Punkt2 = NaN(3,1); % Nur für Kompilieren. Kann nicht auftreten
  end
  % Berechne Kollision aus den beiden begrenzenden (Halb-)Kugeln
  [dist, kol, pkol, d_min] = collision_sphere_sphere([Punkt1,rad1],[Punkt2,rad2]);
  
% Ein Punkt des kürzesten Abstandes liegt auf Zylindermantel, der
% andere auf Anfangs- oder Endhalbkugel der anderen Kapsel
else
  % Prüfe Abstand eines Zylinders zu den Endpunkten

  % erster Endpunkt von Gerade 1 und Punkt auf Gerade 2 (h, mu)
  if lambda <= 0 && (0 < mu && mu < 1)
    Kapsel = Kap2; Kugel = Kap1([1:3, 7]);
  % zweiter Endpunkt von Gerade 1 und Punkt auf Gerade 2 (h, mu)
  elseif lambda >= 1 && (0 < mu && mu < 1)
    Kapsel = Kap2; Kugel = Kap1([4:6, 7]);
  % erster Endpunkt von Gerade 2 und Punkt auf Gerade 1 (g, lambda)
  elseif (0 < lambda && lambda < 1) && mu  <= 0
    Kapsel = Kap1; Kugel = Kap2([1:3, 7]);
  % zweiter Endpunkt von Gerade 2 und Punkt auf Gerade 1 (g, lambda)
  else % (0 < lambda && lambda < 1) && mu  >= 1
    Kapsel = Kap1; Kugel = Kap2([4:6, 7]);
  end
  [dist, kol, pkol, d_min] = collision_capsule_sphere(Kapsel, Kugel);
end
