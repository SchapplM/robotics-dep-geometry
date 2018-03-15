% Berechne einen zufälligen Punkt auf der Oberfläche eines Zylinders
% 
% Eingabe:
% Par_j [1x7] double
%   Parametrierung des Quaders
%   1-3:   Ortsvektor zum ersten Zylinderdeckelmittelpunkt
%   4-6:   Ortsvektor zum zweiten Zylinderdeckelmittelpunkt
%   7:     Radius
% Seite [1x1] uint8
%   0:     Zylinderdeckel bei Punkt 1
%   1:     Zylinderdeckel bei Punkt 2
%   2:     Zylindermantel
% 
% Ausgabe:
% p_i [3x1] double
%   Ortsvektor zu zufälligem Punkt auf Oberfläche
% 
% Quellen
% [1] http://www.anderswallin.net/2009/05/uniform-random-points-in-a-circle-using-polar-coordinates/

% Moritz Schappler, schappler@irt.uni-hannover.de, 2016-09
% (c) Institut für Regelungstechnik, Universität Hannover

function p_i = cylinder_random_surface_point(Par_j, Seite)
%#codegen
assert(isa(Par_j,'double') && isreal(Par_j) && all(size(Par_j) == [1 7]), ...
  'cylinder_random_surface_point: Par_j has to be [1x7] double');  
assert(isa(Seite,'uint8') && all(size(Seite) == [1 1]), ...
  'cylinder_random_surface_point: Seite has to be [1x1] uint8');  

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
  % zufälliger Radius (gleiche Punktdichte, siehe [1])
  r_rand = Par_j(7) * sqrt( rand(1) );
  p_i = Par_j(1:3)' + (Seite==1)*zeta + r_rand*v;
  
elseif Seite == 2 % Zylindermantel, 2 fuer Deckelflaeche
  % Auf Mantel
  hp_rand = rand(1); % Prozentualer Wert der Höhe

  p_i = Par_j(1:3)' + hp_rand*zeta + Par_j(7)*v;
else
  error('Nicht definiert');
end