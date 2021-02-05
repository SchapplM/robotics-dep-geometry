% Berechne einen zufälligen Punkt auf der Oberfläche einer Kapsel
% Die Punktdichte soll auf der gesamten Kapsel gleich sein
% 
% Eingabe:
% Par_j [1x7] double
%   Parametrierung der Kapsel
%   1-3:   Ortsvektor zum ersten Kapseldeckelmittelpunkt
%   4-6:   Ortsvektor zum zweiten Kapseldeckelmittelpunkt
%   7:     Radius
% 
% Ausgabe:
% p_i [3x1] double
%   Ortsvektor zu zufälligem Punkt auf Oberfläche

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-02
% (c) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function p_i = capsule_random_surface_point_equal(Par_j)
%#codegen
assert(isa(Par_j,'double') && isreal(Par_j) && all(size(Par_j) == [1 7]), ...
  'capsule_random_surface_point_equal: Par_j has to be [1x7] double');  

% Radius
R = Par_j(7);

% Hoch-Achse
zeta = (Par_j(4:6)'-Par_j(1:3)');

% Flächen
A_Halbkugel = 2*pi*R^2;
A_Mantel = norm(zeta) * pi * 2*R;
A_Gesamt = 2*A_Halbkugel + A_Mantel;

% Wahrscheinlichkeit für einen Punkt auf einer Halbkugel
P0 = A_Halbkugel/A_Gesamt;

pp = rand(1); % Gleichverteilt von 0-1

% Berechnung der Fälle
if pp < P0
  % Halbkugel 1
  p_i = capsule_random_surface_point(Par_j, uint8(0));
elseif pp < 2*P0
  % Halbkugel 2
  p_i = capsule_random_surface_point(Par_j, uint8(1));
else
  % Mantel
  p_i = capsule_random_surface_point(Par_j, uint8(2));
end