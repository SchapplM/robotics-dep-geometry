% Teste Kollision zweier Kugeln
% 
% Ergebnis:
% Für alle erdachten Testfälle sieht das Ergebnis plausibel aus.
% Gezeichnet werden die jeweiligen Kollisionsobjekte und ihre größte
% Durchdringung

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Kompiliere alle Funktionen. Dadurch werden Syntax-Fehler erkannt
matlabfcn2mex({ ...
  'distance_point_point', ...
  'collision_sphere_sphere', ...
  });

%% Teste Kollision aus Kugel und Kugel
for i = 1:3
  %% Parameter einstellen
  p1 = [0.0;0.0;0.0];
  r1  = 0.1;
  switch i
  case 1 % Keine Kollision
    p2 = [0.2;0.3;0.2];
    r2  = 0.15;
    kol_groundtruth = false;
  case 2 % Kollision
    p2 = [0.1;0.2;0.2];
    r2  = 0.3;
    kol_groundtruth = true;
  case 3 % Vollständige Durchdringung
    p2 = [0.1;0.2;0.2];
    r2  = 0.5;
    kol_groundtruth = true;
  end
  %% Kollision berechnen
  [dist, kol, pkol, d_min] = collision_sphere_sphere([p1',r1], [p2',r2]);
  %% Zeichnen
  figure(1); clf; hold on;
  drawSphere([p1; r1]','FaceColor', 'b', 'FaceAlpha', 0.3)
  drawSphere([p2; r2]','FaceColor', 'r', 'FaceAlpha', 0.3, 'linestyle', ':');
  plot3(pkol(:,1), pkol(:,2), pkol(:,3), '-kx', 'MarkerSize', 5, 'LineWidth', 3)
  xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
  view(3); grid on;
  %% Prüfung
  % Rechnerische Prüfung der Ergebnisse
  assert(abs(norm(pkol(1, :) - pkol(2, :)) - abs(dist)) < 1e-12, ...
    'Abstand und Kollisionspunkte stimmen nicht überein');
  assert((dist<0) == kol, 'Negativer Abstand ohne gemeldete Kollision (oder umgekehrt)');
  % Prüfung aus vorheriger Visueller Überprüfung
  assert(kol == kol_groundtruth, 'Erkannte Kollision stimmt nicht aus händischer Prüfung');

  % Prüfung gegen Ausgabe der mex-Funktion
  [dist2, kol2, pkol2, d_min2] = collision_sphere_sphere_mex([p1',r1], [p2',r2]);
  assert(abs(dist - dist2) < 1e-12, ...
    'Ausgabevariable dist stimmt nicht mit mex-Funktion überein');
  assert(abs(kol - kol2) < 1e-12, ...
    'Ausgabevariable kol stimmt nicht mit mex-Funktion überein');
  assert(all(abs(pkol(:) - pkol2(:)) < 1e-12), ...
    'Ausgabevariable pkol stimmt nicht mit mex-Funktion überein');
  assert(abs(d_min - d_min2) < 1e-12, ...
    'Ausgabevariable d_min stimmt nicht mit mex-Funktion überein');
end
