% Berechne einen zufälligen Punkt auf der Oberfläche eines Zylinders
% Die Punktdichte soll auf dem gesamten Zylinder gleich sein
% 
% Eingabe:
% Par_j [1x7] double
%   Parametrierung des Quaders
%   1-3:   Ortsvektor zum ersten Zylinderdeckelmittelpunkt
%   4-6:   Ortsvektor zum zweiten Zylinderdeckelmittelpunkt
%   7:     Radius
% 
% Ausgabe:
% p_i [3x1] double
%   Ortsvektor zu zufälligem Punkt auf Oberfläche

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

function p_i = cylinder_random_surface_point_equal(Par_j)
%#codegen
assert(isa(Par_j,'double') && isreal(Par_j) && all(size(Par_j) == [1 7]), ...
  'cylinder_random_surface_point_equal: Par_j has to be [1x7] double');  

% Radius
R = Par_j(7);

% Hoch-Achse
zeta = (Par_j(4:6)'-Par_j(1:3)');

% Flächen
A_Deckel = pi*R^2;
A_Mantel = norm(zeta) * pi * 2*R;
A_Gesamt = 2*A_Deckel + A_Mantel;

% Wahrscheinlichkeit für einen Punkt auf einem Deckel
P0 = A_Deckel/A_Gesamt;

pp = rand(1); % Gleichverteilt von 0-1

% Berechnung der Fälle
if pp < P0
  % Deckelfläche 1
  p_i = cylinder_random_surface_point(Par_j, uint8(0));
elseif pp < 2*P0
  % Deckelfläche 2
  p_i = cylinder_random_surface_point(Par_j, uint8(1));
else
  % Mantel
  p_i = cylinder_random_surface_point(Par_j, uint8(2));
end