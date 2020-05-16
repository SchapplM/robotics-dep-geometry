% Berechne Kollision und Abstand zweier Kapseln
% 
% [dist, kol, pkol] = Kollision_Kapsel_Kapsel(Kap1, Kap2)
% Eine Kapsel ist ein Zylinder mit Halbkugeln als Enden
% 
% Eingabe:
% Zyl1, Zyl2: 1x7 Kapsel-Darstellung (Pkt 1, Pkt 2, Radius)
% 
% Ausgabe:
% dist: Abstand der Kapsel-Hüllen
% kol: 1 falls Kollision, sonst 0
% pkol: 2x3; 2 Punkte des kürzesten Abstandes
% d_min: 1x1;  Minimaler Abstand (negativ) der beiden Körper

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2013-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover


function [dist, kol, pkol, d_min] = collision_capsule_capsule(Kap1, Kap2)

%% Coder Information
%#codegen
assert(isa(Kap1,'double') && isreal(Kap1) && all(size(Kap1) == [1 7]) && ... 
     isa(Kap2,'double') && isreal(Kap2) && all(size(Kap2) == [1 7]));
   
%% Algorithmus

% Umwandlung in Geraden in Parameterform
rg = Kap1(1:3)'; rh = Kap2(1:3)'; % Anfangspunkte der Geraden
ug = Kap1(4:6)'-Kap1(1:3)'; uh = Kap2(4:6)'-Kap2(1:3)'; % Richtungsvektoren der Geraden
rad1 = Kap1(7); rad2 = Kap2(7); % Radien der Zylinder
[dnorm, d, lambda, mu, pg, ph] = distance_line_line([rg', ug'], [rh', uh']);

% Punkte des kürzesten Abstandes liegen auf Zylindermanteln
if ~any([lambda, mu]<0) && ~any([lambda, mu]>1)
  pkol = [pg'+d'/dnorm*Kap1(7); ph'-d'/dnorm*Kap2(7)];
  % Abstand der Linien mit Radien der Zylinder vergleichen
  d_min = - (Kap1(7) + Kap2(7)); 
  dist = dnorm - (Kap1(7) + Kap2(7));
  if dist < 0
    kol = 1;
  else
    kol = 0;
  end
  
  
% Beide Punkte des kürzesten Abstandes liegen auf 
% Deckel- oder Bodenfläche der Zylinder
elseif all( uint8([lambda, mu]<=0) + uint8([lambda, mu] >=1) )
  % Prüfe Abstand von zwei Endpunkten
  if lambda <= 0
    Punkt1 = rg';
  else % lambda >= 1
    Punkt1 = rg'+ug';
  end
  if mu <= 0
    Punkt2 = rh';
  else % mu >= 1
    Punkt2 = rh'+uh';
  end
  [dnorm, d] = distance_point_point(Punkt1, Punkt2);
  % Punkte des kürzesten Abstandes:
  pkol = [Punkt1+d'/dnorm*rad1; Punkt2-d'/dnorm*rad2];
  % Abstand der Linien mit Radien der Zylinder vergleichen
  d_min = - (Kap1(7) + Kap2(7));
  dist = dnorm - (Kap1(7) + Kap2(7));
  if dist < 0
    kol = 1;
  else
    kol = 0;
  end
  
% Ein Punkt des kürzesten Abstandes liegt auf Zylindermantel, der
% andere auf Anfangs- oder Endhalbkugel des anderen Zylinders
else
  % Prüfe Abstand eines Zylinders zu den Endpunkten

  % erster Endpunkt von Gerade 1 und Punkt auf Gerade 2 (h, mu)
  if lambda <= 0 && (0 < mu && mu < 1)
    Kapsel = Kap2; Kugel = Kap1([1:3, 7]);
  % zweiter Endpunkt von Gerade 1 und Punkt auf Gerade 2 (h, mu)
  elseif lambda >= 1 && (0 < mu && mu < 1)
    Kapsel = Kap2; Kugel = Kap1([4:6, 7]);
  % erster Endpunkt von Gerade 2 und Punkt auf Gerade 1 (g, lambda)
  elseif (0 < lambda && lambda < 1) && mu  <= 0
    Kapsel = Kap1; Kugel = Kap2([1:3, 7]);
  % zweiter Endpunkt von Gerade 2 und Punkt auf Gerade 1 (g, lambda)
  else % (0 < lambda && lambda < 1) && mu  >= 1
    Kapsel = Kap1; Kugel = Kap2([4:6, 7]);
  end
  [dist, kol, pkol, d_min] = collision_capsule_sphere(Kapsel, Kugel);
end
