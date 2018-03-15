% Berechne einen zufälligen Punkt auf der Oberfläche eines Quaders
% Alle Seitenflächen des Quaders sind gleich häufig vertreten
% 
% Eingabe:
% Par_j [1x12] double
%   Parametrierung des Quaders
%   1-3:   Aufpunkt zur ersten Ecke
%   4-6:   Vektor von erster zu zweiter Ecke (entlang Kante)
%   7-9:   Vektor von erster zu dritter Ecke (entlang Kante)
%   10-12: Vektor von erster zu vierter Ecke (entlang Kante)
% Seite [1x1] uint8
%   0: Zufällige Seite (keine gleiche Punktdichte)
%   1-3: Aufpunkt zur Fläche des Quaders, auf der der Punkt liegt
% 
% Ausgabe:
% p_i [3x1] double
%   Ortsvektor zu zufälligem Punkt auf Oberfläche

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

function p_i = box_random_surface_point(Par_j, Seite)
%#codegen
assert(isa(Par_j,'double') && isreal(Par_j) && all(size(Par_j) == [1 12]), ...
  'box_random_surface_point: Par_j has to be [1x12] double');  
assert(isa(Seite,'uint8') && all(size(Seite) == [1 1]), ...
  'box_random_surface_point: Seite has to be [1x1] uint8');  

iq = uint8(zeros(3,1)); % Der letzte Eintrag bestimmt den Vektor, zu der Fläche, auf der der Punkt liegt
if Seite == 0
  % Zufällige Seite des Quaders
  iq = uint8(randperm(3,3)); % Drei verschiedene Zufallszahlen zwischen 1 und 3
else
  iq(3) = Seite;
  Verbleibend = uint8([1 2 3]);
  Verbleibend(Verbleibend==Seite) = 0;
  iq(1:2) = Verbleibend(Verbleibend~=0);
end
  
% Punkt auf dem Quader
xy_rand = rand(2,1); % Zufallsposition auf zufällig gewählter Seitenfläche
IP1 = 3 + ( ((iq(1)-1)*3+1):(iq(1))*3 );
IP2 = 3 + ( ((iq(2)-1)*3+1):(iq(2))*3 );
IP3 = 3 + ( ((iq(3)-1)*3+1):(iq(3))*3 );

p_i = Par_j(1:3)'  + ...
    Par_j(IP1)' * xy_rand(1) + ...
    Par_j(IP2)' * xy_rand(2) + ...
    Par_j(IP3)' * (randperm(2,1)-1);