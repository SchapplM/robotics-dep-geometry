% Berechne den kürzesten Abstand zwischen einer Ebenen und einem Punkt
% 
% Eingabe:
% Ebene [1x6]
%   Darstellung einer Ebenen (Punkt 1, Normalenvektor)
% Punkt [1x3]
%   Darstellung eines Punktes
% 
% Ausgabe:
% dnorm [1x1]
%   Betrag des kleinsten Abstandes
% d [3x1]
%   Vektor des kleinsten Abstandes von Ebene zu Punkt
% pe [3x1]
%   Punkt des kürzesten Abstandes auf der Ebenen
% 
% Siehe auch: distance_line_line, distance_line_point, distance_plane_point
% 
% Quelle:
% https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_plane
% #Closest_point_and_distance_for_a_hyperplane_and_arbitrary_point

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [dnorm, d, pe] = distance_plane_point2(Ebene, Punkt)

%% Coder Information
%#codegen
assert(isa(Ebene,'double') && isreal(Ebene) && all(size(Ebene) == [1 6]) && ... 
       isa(Punkt,'double') && isreal(Punkt) && all(size(Punkt) == [1 3]));

%% Algorithmus
y = Punkt';
p = Ebene(1:3)';
a = Ebene(4:6)';

d = dot((y-p),a)/dot(a,a)*a; % von der Ebenen zum Punkt
dnorm = norm(d);
pe = y-d; % Punkt auf der Ebenen
