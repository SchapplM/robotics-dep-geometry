% Berechne Kollision und Abstand eines Zylinders und eines Punktes
% 
% Eingabe:
% Zyl [1x7]
%   Zylinder-Darstellung (Pkt 1, Pkt 2, Radius)
% Punkt [1x3]
%   Punkt-Darstellung (Koordinaten)
% 
% Ausgabe:
% dist [1x1]
%   Abstand des Punktes von der Zylinderhülle
% kol [1x1 logical]
%   true falls Kollision, sonst false
% pkol [1x3]
%   Punkte auf der Hüllfläche des Zylinders mit dem kürzesten Abstand
% d_min [1x1]
%   Minimal möglicher Abstand (negativ) der beiden Körper (maximal mögliche
%   Eindringtiefe; dient zur möglichen Normierung der Ausgabe dist)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [dist, kol, pkol, d_min] = collision_cylinder_point(Zyl, Punkt)

%% Coder Information
%#codegen
assert(isa(Zyl,'double') && isreal(Zyl) && all(size(Zyl) == [1 7]) && ... 
       isa(Punkt,'double') && isreal(Punkt) && all(size(Punkt) == [1 3])); 
   
%% Algorithmus
rg = Zyl(1:3)'; % Anfangspunkt der Geraden
ug = Zyl(4:6)'-Zyl(1:3)';% Richtungsvektoren der Geraden
rz = Zyl(7); % Radius des Zylinders
Gerade = [rg', ug'];
% Prüfe kürzeste Entfernung zur Ersatz-Geraden
[dnorm, d, lambda] = distance_line_point(Gerade, Punkt);

if lambda < 0 || lambda > 1 % Punkt liegt jenseits des Endes des Zylinders.
  if lambda <= 0
    p = rg; % Erste Deckelfläche -> setze lambda=0
  else
    p = rg+ug; % Zweite Deckelfläche -> setze lambda=1
  end
  % Prüfen ob auf Höhe der Kreisfläche (Deckel) oder Näher am Zylinder-Rand
  % Prüfe Abstand zu Ebenen-Darstellung mit Aufpunkt und Normalenvektor
  [dnorm, ~, pe] = distance_plane_point2([p', ug'], Punkt);
  v = -p+pe; % Vektor auf der Deckelfläche zum kürzesten Abstand des Punktes
  vnorm = norm(v);
  if vnorm > rz
    % Punkt ist außerhalb des Radius der Deckelfläche. Kürzeste Verbindung
    % am Rand
    pkol = (p+v/vnorm*rz)';
    dist = norm(pkol-Punkt);
  else
    % Punkt ist oberhalb der Deckelfläche. Kürzeste Verbindung von oben
    pkol = pe';
    dist = dnorm;
  end
  kol = false;
else % Kugel liegt zwischen Enden des Zylinders.
  dist = dnorm-rz; % wird negativ, falls Punkt im Zylinder
  if dnorm > 1e-12 % alles in Ordnung.
    v = d; % Vektor von Mittellinie zum Punkt
    vnorm = dnorm;
  else
    % Punkt liegt genau auf Mittellinie
    if ug(1)~=ug(3) % so senkrecht mit folgender Operation
      v = cross(ug, flipud(ug));
    else
      v = ug([1 3 2]); % beliebiger anderer Vektor, da erster nicht geht
    end
    vnorm = norm(v);
  end
  
  % Punkt auf dem Mantel des Zylinders finden
  pkol = (rg+lambda*ug+v/vnorm*rz)';
  if dnorm > rz
    % Punkt liegt außerhalb des Zylinders
    kol = false;
  else
    % Punkt innerhalb des Zylinders
    kol = true;
    % Kürzester Abstand zum Deckel finden. Kann besser sein, als den
    % kürzesten Abstand zum Mantel zu definieren
    if lambda > 0.5 % Oberseite
      vv = ug*(1-lambda); % Vektor zum nächsten Deckel (vom Punkt aus)
    else % Unterseite
      vv = -ug*lambda;
    end
    dist2 = norm(vv); % Abstand zum Deckel (statt zum Mantel)
    if dist2 < -dist % Abstand zum Deckel ist kürzer. Nehme diesen.
      dist = -dist2;
      pkol = Punkt+vv';
    end
  end
end
d_min = - rz; % Bei sehr kurzen Zylindern ist die Länge das begrenzende. Reicht als Näherung