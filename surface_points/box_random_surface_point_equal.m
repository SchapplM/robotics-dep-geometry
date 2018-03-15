% Berechne einen zufälligen Punkt auf der Oberfläche eines Quaders
% Die Punktdichte soll auf dem gesamten Quader gleich sein
% 
% Eingabe:
% Par_j [1x12] double
%   Parametrierung des Quaders
%   1-3:   Aufpunkt zur ersten Ecke
%   4-6:   Vektor von erster zu zweiter Ecke (entlang Kante)
%   7-9:   Vektor von erster zu dritter Ecke (entlang Kante)
%   10-12: Vektor von erster zu vierter Ecke (entlang Kante)
% 
% Ausgabe:
% p_i [3x1] double
%   Ortsvektor zu zufälligem Punkt auf Oberfläche

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

function p_i = box_random_surface_point_equal(Par_j)
%#codegen
assert(isa(Par_j,'double') && isreal(Par_j) && all(size(Par_j) == [1 12]), ...
  'box_random_surface_point_equal: Par_j has to be [1x12] double');  

% Flächenverteilung
r_0_Q1Q2 = Par_j(4:6);
r_0_Q1Q3 = Par_j(7:9);
r_0_Q1Q4 = Par_j(10:12);

A3 = norm( cross(r_0_Q1Q2, r_0_Q1Q3) );
A2 = norm( cross(r_0_Q1Q2, r_0_Q1Q4) );
A1 = norm( cross(r_0_Q1Q3, r_0_Q1Q4) );
A_Gesamt = A1*2 + A2*2 + A3*2;

P1 = A1/A_Gesamt;
P2 = A2/A_Gesamt;

pp = rand(1); % Gleichverteilt von 0-1

if pp < 2*P1
  % Seite 1
  Seite = uint8(1);
elseif pp < (2*P1+2*P2)
  % Seite 2
  Seite = uint8(2);
else
  Seite = uint8(3);
end

p_i = box_random_surface_point(Par_j, Seite);