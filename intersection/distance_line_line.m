% Berechne den kürzesten Abstand zwischen zwei Geraden
% 
% Berechnet die kürzeste Entfernung als Entfernung und Vektor sowie
% die Punkte auf den Geraden, in kartesischen Koordinaten und in
% Parameterdarstellung
% 
% Eingabe:
% Gerade1, Gerade2 [1x6]
%   Darstellung zweier Geraden (Punkt 1, Richtungsvektor)
% 
% Ausgabe:
% dnorm
%   Betrag des kleinsten Abstandes
% d [3x1]
%   Vektor des kleinsten Abstandes von Gerade1 zu 2
% lambda
%   Parameter zum Punkt des kürzesten Abstandes auf Gerade 1:
%   Punkt = OV + lambda*RV
% mu
%   Parameter zum Punkt des kürzesten Abstandes auf Gerade 2:
%   Punkt = OV + mu*RV
% pg [3x1]
%   Punkt des kürzesten Abstandes auf Gerade 1
% ph [3x1]
%   Punkt des kürzesten Abstandes auf Gerade 2
% 
% Siehe auch: 
% distanceLines3d aus geom3d Toolbox (ähnlicher Algorithmus)
% distance_line_point, distance_point_point,
% collision_capsule_capsule, collision_capsule_sphere

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2013-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [dnorm, d, lambda, mu, pg, ph] = distance_line_line(Gerade1, Gerade2)

%% Coder Information
%#codegen
assert(isa(Gerade1,'double') && isreal(Gerade1) && all(size(Gerade1) == [1 6]) && ... 
       isa(Gerade2,'double') && isreal(Gerade2) && all(size(Gerade2) == [1 6])); 

%% Algorithmus
rg = Gerade1(1:3)'; rh = Gerade2(1:3)'; % Anfangspunkte der Geraden
ug = Gerade1(4:6)'; uh = Gerade2(4:6)'; % Richtungsvektoren der Geraden
% Lösung des Gleichungssysteme mit Invertierung der 2x2-Matrix [a b; c d]
a = ug'*uh; b = -uh'*uh; c = ug'*ug; d = -uh'*ug;
den = (a*d-b*c); % Determinante
if abs(den) < 1e-12
  % Geraden sind parallel
  pg = rg; % Alle Punkte haben den gleichen Abstand. Nehme den ersten Aufpunkt
  lambda = 0; % Eigenschaft des Aufpunkts
  [dnorm, d, mu, ph] = distance_line_point(Gerade2, pg');
  return
end

ml = [d, -b; -c, a] / den * [(rh-rg)'*uh; (rh-rg)'*ug];
% Parameter der Punkte mit minimalem Abstand zur anderen Geraden
lambda = ml(1);
mu = ml(2);
% Abstand der beiden Geraden
d = -(rg-rh+ml(1)*ug-ml(2)*uh); % vom Punkt auf Gerade1 (g) zum Punkt auf Gerade2 (h)
dnorm = norm(d);
% Punkte des kürzesten Abstands:
pg = (rg+ml(1)*ug); % Punkte auf Gerade 1 (lambda-Gerade)
ph = (rh+ml(2)*uh); % Punkte auf Gerade 2 (mu-Gerade)
