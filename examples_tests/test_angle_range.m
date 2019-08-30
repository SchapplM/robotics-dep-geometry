% Testskript für die Funktion `angle_range` zur Berechnung von
% Winkelspannbreiten
% Verschiedene intuitive Berechnungsansätze werden auch getestet. Diese
% liefern aber das falsche Ergebnis
% 
% Anwendung: Berechnung der Ausnutzung von Gelenkwinkelgrenzen
% 
% Ergebnis: Funktion liefert das erwartete Ergebnis

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover


clear
clc
close all
%% Minimalbeispiele
% Sätze von Winkeln, für die per Augenmaß die Spannbreite ermittelt wurde
theta = cell(5,1); % Winkel-Kombinationen
delta = NaN(5,1); % Manuell bestimmte Spannbreite
theta{1} = [10, 20, 50, 80, -20, -30];
delta(1) = 110;
theta{2} = [10, 20, 150, 80, -20, -30];
delta(2) = 180;
theta{3} = [100, 120, 150, -100, -150];
delta(3) = 160; % von -100 bis 0 (100) und von 0 bis 
theta{4} = [-20, 160, 130, -50, -70];
delta(4) = 210; % 130 bis 20 sind raus -> 180+(180-150)=210 sind drin.
theta{5} = [17  -167  -110   -94    17];
delta(5) = 184; % 167+17 = 184 sind drin

for i = 1:5
  fprintf('Konfiguration %d. Winkel: [%s]\n', i, disp_array(theta{i}, '%1.0f'));
  delta_i_test = NaN(1,3);
  
  % Ansatz 1: Zu testende Funktion
  delta_i_test(1) = 180/pi*angle_range(pi/180*theta{i}');
  
  % Ansatz 2: Intuitive Differenz (zulässig bei kartesischen Positionen)
  delta_i_test(2) = diff(minmax2(theta{i}));
  
  % Ansatz 3:
  delta_i_test(3) = 180/pi*angdiff(pi/180*minmax2(theta{i}));
  
  % Ansatz 4:
  delta_i_test(4) = 180/pi*max(angdiff(pi/180*theta{i}, pi/180*theta{i}(1)));
  
  % Ansatz 5:
  delta_i_test(5) = 180/pi*diff( (minmax2( ...
    angdiff(pi/180*theta{i}, min(pi/180*theta{i}(1))) ...
    ) ) );

  % Ansatz 6
  delta_i_test(6) = 180/pi*diff( (minmax2(  ...
    angdiff(pi/180*theta{i}, min(pi/180*theta{i})) ...
    )));
  
  % Vergleiche mit berechneter Lösung
  I_res = abs(delta_i_test-delta(i)) < 1e-8;
  
  % Ausgabe der Auswertung
  fprintf('Konfiguration %d; Korrekte Ansätze: [%s]\n', ...
    i, disp_array(find(I_res), '%d'));
  
  figure(i);clf;hold all
  drawCircle(0,0,1);
  for j = 1:length(theta{i})
    plot([0;cos(pi/180*theta{i}(j))], [0; sin(pi/180*theta{i}(j))], '-');
    text(cos(pi/180*theta{i}(j)), sin(pi/180*theta{i}(j)), sprintf('%1.0f', theta{i}(j)));
  end
  grid on;
  title(sprintf('Spannbreite: %1.0f; Erg.: [%s]. Korrekt: [%s]', ...
    delta(i), disp_array(delta_i_test, '%1.0f'), disp_array(find(I_res), '%d')));
  
  if ~I_res(1)
    error('Der Ansatz aus angle_range.m funktioniert nicht. Sollte er aber!');
  end
end
fprintf('Funktion angle_range erfolgreich getestet\n');