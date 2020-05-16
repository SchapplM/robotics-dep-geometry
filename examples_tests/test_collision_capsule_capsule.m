% Teste Kollision zweier Kapseln

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Kompiliere alle Funktionen. Dadurch werden Syntax-Fehler erkannt
matlabfcn2mex({ ...
  'distance_point_point', ...
  'distance_line_point', ... 
  'distance_line_line', ...
  'collision_sphere_sphere', ...
  'collision_capsule_sphere', ...
  'collision_capsule_capsule', ...
  });

for i = 1:4
  for j = 1:4

    p1A = [0.0;0.0;0.0];
    p1B = [0.0;0.0;0.4];
    r1  = 0.1;
    % Vertausche die Reihenfolge der Punkte, um mehr Code in Test abzudecken.
    % Darf keinen Einfluss auf Ergebnis haben
    if j <= 2
      Kap1 = [p1A; p1B; r1]';
    else
      Kap1 = [p1B; p1A; r1]';
    end
    switch i
      case 1 % Keine Kollision
        p2A = [0.2;0.3;0.2];
        p2B = [0.5;0.4;0.3];
        r2  = 0.2;
        kol_groundtruth = false;
      case 2 % Kollision Zylinder-Kugelende
        p2A = [0.1;0.2;0.2];
        p2B = [0.4;0.3;0.3];
        r2  = 0.2;
        kol_groundtruth = true;
      case 3 % Kollision der Zylinder
        p2A = [ 0.2;0.1;0.2];
        p2B = [-0.1;0.2;0.3];
        r2  = 0.08;
        kol_groundtruth = true;
      case 4 % Kollision mit Zylinder und Kugelende
        p2A = [0.1;0.2;0.35];
        p2B = [0.4;0.3;0.45];
        r2  = 0.2;
        kol_groundtruth = true;
      case 5 % Kollision mit Zylinder und Kugelende. TODO: Noch fehlerhaft
        p2A = [0.1;0.1;0.6];
        p2B = [0.1;0.1;1.2];
        r2  = 0.3;
        kol_groundtruth = true;
    end
    % Nochmal vertauschen der Punkte zur größeren Testabdeckung
    if any(j == [1 3])
      Kap2 = [p2A; p2B; r2]';
    else
      Kap2 = [p2B; p2A; r2]';
    end

    %% Kollision berechnen
    [dist, kol, pkol, d_min] = collision_capsule_capsule(Kap1, Kap2);

    %% Zeichnen
    if j == 1 % Nur für ersten Fall zeichnen.
      figure(1); clf; hold on;
      drawSphere([p1A; r1]','FaceColor', 'b', 'FaceAlpha', 0.3)
      drawCylinder([p1A;p1B;r1]','FaceColor', 'b', 'FaceAlpha', 0.3)
      drawSphere([p1B; r1]','FaceColor', 'b', 'FaceAlpha', 0.3)

      drawSphere([p2A; r2]','FaceColor', 'r', 'FaceAlpha', 0.3)
      drawCylinder([p2A;p2B;r2]','FaceColor', 'r', 'FaceAlpha', 0.3)
      drawSphere([p2B; r2]','FaceColor', 'r', 'FaceAlpha', 0.3)

      plot3(pkol(:,1), pkol(:,2), pkol(:,3), '-kx', 'MarkerSize', 5, 'LineWidth', 3)

      xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
    end

    %% Prüfung
    % Rechnerische Prüfung der Ergebnisse
%     assert(~isnan([dist(:), ...
%       'Abstand und Kollisionspunkte stimmen nicht überein');
    assert(abs(norm(pkol(1, :) - pkol(2, :)) - abs(dist)) < 1e-12, ...
      'Abstand und Kollisionspunkte stimmen nicht überein');
    assert((dist<0) == kol, 'Negativer Abstand ohne gemeldete Kollision (oder umgekehrt)');
    % Prüfung aus vorheriger Visueller Überprüfung
    assert(kol == kol_groundtruth, 'Erkannte Kollision stimmt nicht aus händischer Prüfung');
  end
end