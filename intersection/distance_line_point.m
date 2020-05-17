% Berechne den kürzesten Abstand zwischen einer Geraden und einem Punkt
% 
% Berechnet die kürzeste Entfernung als Entfernung und Vektor und
% den Punkt auf der Geraden, in kartesischen Koordinaten und in
% Parameterdarstellung
% 
% Eingabe:
% Gerade [1x6]
%   Darstellung einer Geraden (Punkt 1, Richtungsvektor)
% Punkt [1x3]
%   Darstellung eines Punktes
% 
% Ausgabe:
% dnorm [1x1]
%   Betrag des kleinsten Abstandes
% d [3x1]
%   Vektor des kleinsten Abstandes von Gerade1 zu 2
% lambda [1x1]
%   Parameter zum Punkt des kürzesten Abstandes auf der Geraden:
%   Punkt = OV + lambda*RV
% pg [3x1]
%   Punkt des kürzesten Abstandes auf Gerade 1
% 
% Siehe auch: distance_line_line, distance_point_point,
% collision_capsule_capsule, collision_capsule_sphere

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2013-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [dnorm, d, lambda, pg] = distance_line_point(Gerade, Punkt)

%% Coder Information
%#codegen
assert(isa(Gerade,'double') && isreal(Gerade) && all(size(Gerade) == [1 6]) && ... 
       isa(Punkt,'double') && isreal(Punkt) && all(size(Punkt) == [1 3]));

%% Algorithmus
rg = Gerade(1:3)'; ug = Gerade(4:6)'; rq = Punkt';
lambda = (rq-rg)'*ug / (ug'*ug);
d = rq-rg-lambda*ug; % Von der Geraden zum Punkt
dnorm = norm(d);
pg = rg + lambda*ug;
