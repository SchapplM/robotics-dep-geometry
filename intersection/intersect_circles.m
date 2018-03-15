% Berechne Schnittpunkt zweier Kreise mit einfacher Formel
%
% Eingabe:
% r_p1, r_p2 [2x1]
%   Kreismittelpunkte
% r1, r2  [1x1]
%   Radien der Kreise
% 
% Ausgabe:
% r_S [2x2]
%   Kreisschnittpunkte (Punkte nebeneinander als Spaltenvektoren)
% err [1x1]
%   Fehlerwert: 0 = kein Fehler, 1 = kein Schnittpunkt
% 
% Quelle:
% [1] https://de.wikipedia.org/wiki/Schnittpunkt#Schnittpunkte_zweier_Kreise

% Moritz Schappler, schappler@irt.uni-hannover.de, 2015-11
% (c) Institut für Regelungstechnik, Leibniz Universität Hannover

function [r_S, err] = intersect_circles(r_p1, r_p2, r1, r2)

%% Init
%#codegen
% Coder Information
assert(isa(r_p1,'double') && isreal(r_p1) && all(size(r_p1) == [2 1]), ...
  'Punkt P1 gefordert als [2x1] double');
assert(isa(r_p2,'double') && isreal(r_p2) && all(size(r_p2) == [2 1]), ...
  'Punkt P1 gefordert als [2x1] double');
assert(isa(r1,'double') && isreal(r1) && all(size(r1) == [1 1]), ...
  'Radius r1 gefordert als [2x1] double');
assert(isa(r2,'double') && isreal(r2) && all(size(r2) == [1 1]), ...
  'Radius r2 gefordert als [2x1] double');

r_S = NaN(2,2);

%% Berechnung

% Koeffizienten der Geradengleichung
a = 2*(r_p2(1) - r_p1(1));
b = 2*(r_p2(2) - r_p1(2));
c = r1^2 - r_p1(1)^2 - r_p1(2)^2 - r2^2 + r_p2(1)^2 + r_p2(2)^2;

% Schnittpunkt dieser Geraden mit ersten Kreis
d = c - a*r_p1(1) - b*r_p1(2);

% Wurzel-Ausdruck
radikant = r1^2*(a^2+b^2)-d^2;
if radikant < 0
  err = 1;
  return;
end

x_s = NaN(1,2);
x_s(1) = r_p1(1) + (a*d + b*sqrt(radikant))/(a^2+b^2);
x_s(2) = r_p1(1) + (a*d - b*sqrt(radikant))/(a^2+b^2);

y_s = NaN(1,2);
y_s(1) = r_p1(2) + (b*d - a*sqrt(radikant))/(a^2+b^2);
y_s(2) = r_p1(2) + (b*d + a*sqrt(radikant))/(a^2+b^2);

r_S = [x_s; y_s];
err = 0;