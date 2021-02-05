% Berechne einen zufälligen Punkt auf der Oberfläche einer Kapsel
% 
% Eingabe:
% Par_j [1x7] double
%   Parametrierung der Kapsel
%   1-3:   Ortsvektor zum ersten Kapseldeckelmittelpunkt
%   4-6:   Ortsvektor zum zweiten Kapseldeckelmittelpunkt
%   7:     Radius
% Seite [1x1] uint8
%   0:     Kapselende (Halbkugel) bei Punkt 1
%   1:     Kapselende (Halbkugel) bei Punkt 2
%   2:     Zylindermantel
% 
% Ausgabe:
% p_i [3x1] double
%   Ortsvektor zu zufälligem Punkt auf Oberfläche
% 
% Quellen
% [1] https://www.jasondavies.com/maps/random-points/

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-02
% (c) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function p_i = capsule_random_surface_point(Par_j, Seite)
%#codegen
assert(isa(Par_j,'double') && isreal(Par_j) && all(size(Par_j) == [1 7]), ...
  'capsule_random_surface_point: Par_j has to be [1x7] double');  
assert(isa(Seite,'uint8') && all(size(Seite) == [1 1]), ...
  'capsule_random_surface_point: Seite has to be [1x1] uint8');  

% Hoch-Achse
zeta = (Par_j(4:6)'-Par_j(1:3)');

% Bestimme zufälligen Vektor v senkrecht zur Zylinder-Achse (im
% Körper-KS i)
v = NaN(3,1);
[~,Izetamax] = max(abs(zeta)); % Teile durch das betragsgrößte Element
if Izetamax == 3
  v(1:2) = rand(2,1)*2-1;
  v(3) = - (v(1)*zeta(1)+v(2)*zeta(2))/zeta(3);
elseif Izetamax == 2
  v([1 3]) = rand(2,1)*2-1;
  v(2) = - (v(1)*zeta(1)+v(3)*zeta(3))/zeta(2);
else
  v([2 3]) = rand(2,1)*2-1;
  v(1) = - (v(2)*zeta(2)+v(3)*zeta(3))/zeta(1);
end
% normieren
v = v/norm(v);

% Kontakt auf Mantel- oder Deckelfläche
if Seite == 0 || Seite  == 1
  % Kugelkoordinaten zufällig bestimmen. Siehe 1.
  lambda = -pi/2 + pi*rand(1);
  x = rand(1);
  phi = -pi/2+acos(2*x-1);
  r_Mi = Par_j(1:3)' + (Seite==1)*zeta; % Vektor zum Mittelpunkt des Endes
  r_Mi_Pi = (-1)^(Seite~=1) * rotx(lambda)*roty(phi)*Par_j(7)*zeta/norm(zeta);
  p_i = r_Mi + r_Mi_Pi;
  
elseif Seite == 2 % Zylindermantel
  % Auf Mantel
  hp_rand = rand(1); % Prozentualer Wert der Höhe
  p_i = Par_j(1:3)' + hp_rand*zeta + Par_j(7)*v;
else
  error('Nicht definiert');
end