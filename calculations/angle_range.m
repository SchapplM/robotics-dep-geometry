% Berechne die Spannweite einer Menge von Winkeln
% Die Spannweite berücksichtigt die Definition der Winkel auf -pi bis pi.
% Es wird also das kleinstmögliche Kreissegment gewählt, welches alle
% übergebenen Winkel enthält. Durch die 2-Pi-Periodizität kann nicht die
% Definition der Spannweite für metrische Intervallskalen verwendet werden
% 
% Eingabe:
% theta [nxm]
%   m Vektoren mit jeweils n Winkeln
% normalized (1x1 logical)
%   falls true liegen die Winkel schon normalisiert [-pi, pi) vor
% 
% Ausgabe:
% theta_range [1xm]
%   Spannweite der m Winkelsätze

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function theta_range = angle_range(theta, normalized)
%% Init
%#codegen
%$cgargs {coder.newtype('double',[inf,inf]), true}
if nargin == 1
  normalized = false;
end
theta_range = Inf(1, size(theta, 2));
%% Berechnung
for i = 1:size(theta, 2)
  % Sortiere die normalisierten Winkel.
  if normalized
    theta_i_sort = sort(theta(:,i));
  else
    theta_i_sort = sort(normalizeAngle(theta(:,i)));
  end
  % Bestimme den Abstand jedes Winkels zum nächsthöheren Winkel (gehe den
  % Einheitskreis links herum)
  theta_i_diff = angdiff(theta_i_sort(1:end), theta_i_sort([2:end,1]));
  % Bestimme den maximalen Abstand zwischen zwei Winkeln. Dieser Abstand
  % entspricht der Spannweite aller Winkel
  [theta_range_cand,I_maxrange] = max(abs(theta_i_diff));
  if theta_i_diff(I_maxrange) < 0
    % Nächsthöherer Wert liegt zur linken. 
    % Die Lücke ist also bestimmt durch den aktuell betrachteten Abstand.
    % Der Wertebereich ist der andere Teil des Vollkreises.
    theta_range_cand = 2*pi - theta_range_cand;
  else
    % Der Abstand liegt zum nächsten Winkel liegt zur rechten. Das heißt,
    % dass der nächsthöhere Winkel auf dem Kreis links herum weiter weg ist
    % als rechts herum. Der berechnete Wert schließt also bereits alle
    % anderen Winkel mit ein und ist die Spannweite.
  end
  theta_range(i) = theta_range_cand;
end