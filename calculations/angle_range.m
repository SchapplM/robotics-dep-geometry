% Berechne die Spannbreite einer Menge von Winkeln
% Die Spannbreite berücksichtigt die Definition der Winkel auf -pi bis pi.
% Es wird also das kleinstmögliche Kreissegment gewählt, welches alle
% übergebenen Winkel enthält
% 
% Eingabe:
% theta [nxm]
%   m Vektoren mit jeweils n Winkeln
% 
% Ausgabe:
% theta_range [1xm]
%   Spannbreite der m Winkelsätze

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function theta_range = angle_range(theta)
theta_range = Inf(1, size(theta, 2));
for i = 1:size(theta, 2)
  % Prüfe für jeden Winkel im Vektor den Abstand zu jedem anderen (angdiff).
  % Die Differenz des größten und kleinsten Abstandes ist die Spannbreite
  % für diesen Winkel. Der kleinste Wert für alle Winkel entspricht der
  % Spannbreite des Vektors
  % TODO: Dieser Algorithmus ist O(n^2). Es müsste noch eine schnellere
  % Variante geben.
  for j = 1:size(theta,1)
    theta_range_cand = diff( minmax2(angdiff(theta(:,i), theta(j,i))') );
    if theta_range_cand < theta_range(i)
      theta_range(i) = theta_range_cand;
    end
  end
end