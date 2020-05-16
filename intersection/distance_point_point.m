% Berechne den Abstand zwischen zwei Punkten
% 
% [dnorm, d] = Abstand_Punkt_Punkt(Punkt1, Punkt2)
% Berechnet den Abstand als Entfernung und Vektor
% 
% Eingabe:
% Punkt1, Punkt2: 1x3 Darstellung zweier Punkte
% 
% Ausgabe:
% dnorm 1x1: Betrag des Abstandes
% d 3x1: Vektor des Abstandes von Punkt1 zu 2
% 
% 
% Siehe auch: Abstand_Gerade_Punkt,
% Kollision_Kugel_Kugel, Kollision_Kapsel_Kugel

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2013-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover


function [dnorm, d] = distance_point_point(Punkt1, Punkt2)

%% Coder Information
%#codegen
assert(isa(Punkt1,'double') && isreal(Punkt1) && all(size(Punkt1) == [1 3]) && ... 
       isa(Punkt2,'double') && isreal(Punkt2) && all(size(Punkt2) == [1 3]));

%% Algorithmus
rp = Punkt1; rq = Punkt2;
d = -rp' + rq'; % von Punkt1 (P) nach Punkt2 (Q)
dnorm = norm(d);
