% Testskript für die Funktion `angle_range` zur Berechnung von
% Winkelspannweiten
% Verschiedene (zunächst) intuitive Berechnungsansätze werden auch
% getestet. Diese liefern aber das falsche Ergebnis.
% 
% Anwendung: Berechnung der Ausnutzung von Gelenkwinkelgrenzen
% 
% Ergebnis: Funktion liefert das erwartete (richtige) Ergebnis

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover


clear
clc
close all
%% Manuell berechnete Minimalbeispiele
% Sätze von Winkeln, für die per Augenmaß die Spannweite ermittelt wurde
theta = cell(5,1); % Winkel-Kombinationen
delta = NaN(5,1); % Manuell bestimmte Spannweite
theta{1} = pi/180*[10, 20, 50, 80, -20, -30];
delta(1) = pi/180*110;
theta{2} = pi/180*[10, 20, 150, 80, -20, -30];
delta(2) = pi/180*180;
theta{3} = pi/180*[100, 120, 150, -100, -150];
delta(3) = pi/180*160; % von -100 bis 0 (100) und von 0 bis 
theta{4} = pi/180*[-20, 160, 130, -50, -70];
delta(4) = pi/180*210; % 130 bis 20 sind raus -> 180+(180-150)=210 sind drin.
theta{5} = pi/180*[17  -167  -110   -94    17];
delta(5) = pi/180*184; % 167+17 = 184 sind drin

for i = []%1:5
  fprintf('Konfiguration %d. Winkel: [%s]\n', i, disp_array(180/pi*theta{i}, '%1.0f'));
  delta_i_test = NaN(1,3);
  
  % Ansatz 1: Zu testende Funktion
  delta_i_test(1) = angle_range(theta{i}');
  
  % Ansatz 2: Intuitive Differenz (zulässig bei metrischen Intervallskalen)
  delta_i_test(2) = diff(minmax2(theta{i}));
  
  % Ansatz 3:
  delta_i_test(3) = angdiff(minmax2(theta{i}));
  
  % Ansatz 4:
  delta_i_test(4) = max(angdiff(theta{i}, theta{i}(1)));
  
  % Ansatz 5:
  delta_i_test(5) = diff( (minmax2( ...
    angdiff(theta{i}, min(theta{i}(1))) ...
    ) ) );

  % Ansatz 6
  delta_i_test(6) = diff( (minmax2(  ...
    angdiff(theta{i}, min(theta{i})) ...
    )));
  
  % Ansatz 7: Winkel sortieren, aus Abstand zu Nachbarn die Lücke bestimmen
  % (Ansatz 1 aus Funktion)
  theta_sort = sort(normalizeAngle(theta{i}));
  diff_links = angdiff( theta_sort([1:end]), theta_sort([2:end,1]) );
  [delta_i_test(7),I_madl] = max(abs(diff_links));
  if diff_links(I_madl) < 0
    % nächster Nachbar liegt zur linken und nicht zur rechten. 
    % Wenn der Nachbar zur rechten liegt, umschließt der größte Wert als
    % Spannweite alle anderen Winkel
    delta_i_test(7) = 2*pi-delta_i_test(7);
  end
  
  % Ansatz 8: Für jeden Winkel alle Abstände prüfen
  delta_i_test(8) = Inf;
  for j = 1:length(theta{i})
    theta_range_cand = diff( minmax2(angdiff(theta{i}(:), theta{i}(j))') );
    if theta_range_cand < delta_i_test(8)
      delta_i_test(8) = theta_range_cand;
    end
  end
  
  % Vergleiche mit berechneter Lösung
  I_res = abs(delta_i_test-delta(i)) < 1e-8;
  
  % Ausgabe der Auswertung
  fprintf('Konfiguration %d; Korrekte Ansätze: [%s]\n', ...
    i, disp_array(find(I_res), '%d'));
  
  figure(i);clf;hold all
  drawCircle(0,0,1);
  for j = 1:length(theta{i})
    plot([0;cos(theta{i}(j))], [0; sin(theta{i}(j))], '-');
    text(cos(theta{i}(j)), sin(theta{i}(j)), sprintf('%1.0f / %1.0f', ...
      180/pi*theta{i}(j), 180/pi*normalizeAngle(theta{i}(j))));
  end
  grid on;
  title(sprintf('Spannweite: %1.0f; Erg.: [%s]. Korrekt: [%s]', ...
    180/pi*delta(i), disp_array(180/pi*delta_i_test, '%1.0f'), disp_array(find(I_res), '%d')));
  
  if ~I_res(1)
    error('Der Ansatz aus angle_range.m funktioniert nicht. Sollte er aber!');
  end
  if ~I_res(8)
    error('Der Ansatz Nr. 8 muss funktionieren, da er alle Möglichkeiten untersucht. Tut er aber nicht.');
  end
end
fprintf('Funktion angle_range erfolgreich gegen manuell geprüfte Beispiele getestet\n');

%% Automatischer Test mit Zufallswerten
% Funktion kompilieren
matlabfcn2mex({'angle_range'});
n = 1e4;
na = 5;
for i = 1:n
  theta_i = 2*pi*rand(na,1);
  
  % Bestimme Spannweite durch ausprobieren (teuer)
  delta = Inf;
  for j = 1:length(theta_i)
    theta_range_cand = diff( minmax2(angdiff(theta_i(:), theta_i(j))') );
    if theta_range_cand < delta
      delta = theta_range_cand;
    end
  end
  % Vergleiche mit Funktion
  delta_fcn = angle_range(theta_i, true);
  if abs(delta-delta_fcn) > 1e-8
    error('Spannweite aus permutatorischem Ansatz stimmt nicht mit Funktion überein');
  end
  delta_mex = angle_range_mex(theta_i, true);
  if abs(delta_mex-delta_fcn) > 1e-8
    error('Berechnung aus mex-Funktion stimmt nicht');
  end
end
fprintf('Funktion angle_range erfolgreich gegen Zufallswerte getestet\n');
